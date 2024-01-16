use std::{cmp::PartialEq, fmt::Debug};

use crate::{
    communication::{ComError, Connection},
    kinematics::{Position, Joint, Motion},
    logging::warn,
};
use gilrs::{Axis, Button, Gamepad};

// servo angles
pub const MAX_ANGLE: f64 = 180.0;
pub const MIN_ANGLE: f64 = 0.0;

// microseconds for arduino
pub const MAX_SERVO: u16 = 2400;
pub const MIN_SERVO: u16 = 250;

/// Defines a robot and its physical properties
#[derive(Debug)]
pub struct Robot {
    pub position: Position,
    pub target_position: Option<Position>,

    // as a velocity vector
    pub velocity: Position,
    pub max_velocity: Position,
    pub target_velocity: Position,

    // maximum change in velocity per second
    pub acceleration: f64,

    pub angles: Arm,
    pub upper_arm: f64,
    pub lower_arm: f64,
    pub square_sum: f64,
    pub claw_open: bool,
    pub connection: Connection,
}



/// Very specific names for servos
#[derive(Debug)]
pub struct Arm {
    pub base: Joint,
    pub shoulder: Joint,
    pub elbow: Joint,
    pub claw: Joint,
}

/// quirky arm
#[derive(Debug, Copy, Clone)]
pub struct Servos {
    pub base: u16,
    pub shoulder: u16,
    pub elbow: u16,
    pub claw: u16,
}

impl Robot {
    pub fn new(lower_arm: f64, upper_arm: f64) -> Robot {
        Robot {
            position: Position::default(),
            target_position: None,
            velocity: Position::default(),
            max_velocity: Position::default(),
            target_velocity: Position::default(),
            acceleration: 1.,
            angles: Arm::default(),
            upper_arm,
            lower_arm,
            square_sum: upper_arm * upper_arm + lower_arm * lower_arm,
            claw_open: false,
            connection: Connection::default(),
        }
    }

    pub fn update_gamepad(&mut self, gamepad: &Gamepad) {
        let right_stick_axis_y = gamepad.value(Axis::RightStickY) as f64;
        let left_stick_axis_x = gamepad.value(Axis::LeftStickX) as f64;
        let left_stick_axis_y = gamepad.value(Axis::LeftStickY) as f64;

        self.target_velocity.z = self.max_velocity.z * left_stick_axis_y;
        self.target_velocity.x = self.max_velocity.x * left_stick_axis_x;
        self.target_velocity.y = self.max_velocity.y * right_stick_axis_y;

        if gamepad.is_pressed(Button::Start) {
            panic!("Start button pressed");
        }
    }

    pub fn update_position(&mut self, delta: f64) {
        // actual acceleration for this update step
        let acceleration = self.acceleration * delta;

        // the changle in velocity we need
        let mut delta_velocity = self.target_velocity - self.velocity;

        // limit change to maximum acceleration
        delta_velocity.cube_clamp(-acceleration, acceleration);

        // update position and velocity
        self.velocity += delta_velocity;
        self.position += self.velocity * delta;

        // limit position to not be outside of the range of motion
        let mut sphere = self.position.to_sphere();

        // clamp distance from origin
        sphere.dst = sphere.dst.min(self.upper_arm + self.lower_arm);

        // convert back to 3d cordinates and apply new position
        self.position = sphere.to_position();
    }

    pub fn update_ik(&mut self) {
        let angles = self
            .position
            .inverse_kinematics(self.upper_arm, self.lower_arm);

        match angles {
            Ok(angles) => {
                    self.angles.base.angle = angles.0;
                    self.angles.shoulder.angle = angles.1;
                    self.angles.elbow.angle = angles.2;
                }

            Err(()) => {
                warn("Could not calculate inverse kinematics");
            }
        }
    }

    pub fn update(&mut self, gamepad: &Gamepad, delta: f64) -> Result<(), ComError> {
        self.update_gamepad(gamepad);
        self.update_position(delta);
        self.update_ik();
        let data = self.angles.to_servos().to_message();
        self.connection.write(&data, true)
    }
}

/// convert servo position represented as an angle into values understod by the servo
impl Joint {
    fn into_servo(&self) -> u16 {
        let factor = (self.motion.get_pivot_angle(self.angle) - MIN_ANGLE) / MAX_ANGLE;
        ((MAX_SERVO - MIN_SERVO) as f64 * factor + MIN_SERVO as f64) as u16
    }
}

impl PartialEq for Joint {
    fn eq(&self, other: &Self) -> bool {
        let left = (self.angle * 10.0f64.powi(4)).round() / 10.0f64.powi(4);
        let right = (other.angle * 10.0f64.powi(4)).round() / 10.0f64.powi(4);
        left == right
    }
}

impl PartialEq for Arm {
    fn eq(&self, other: &Self) -> bool {
        self.base == other.base
            && self.shoulder == other.shoulder
            && self.elbow == other.elbow
            && self.claw == other.claw
    }
}

/// he's average alright
impl Default for Arm {
    fn default() -> Self {
        Self {
            base: Joint::default(),
            shoulder: Joint::default(),
            elbow: Joint::default(),
            claw: Joint::default(),
        }
    }
}

/// Arm functions
impl Arm {
    pub fn to_servos(&self) -> Servos {
        Servos {
            base: self.base.into_servo(),
            shoulder: self.shoulder.into_servo(),
            elbow: self.elbow.into_servo(),
            claw: self.claw.into_servo(),
        }
    }
}

impl Servos {
    pub fn to_message(&self) -> Vec<u8> {
        unsafe { std::mem::transmute::<Box<Servos>, &[u8; 8]>(Box::new(*self)) }.to_vec()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    pub fn servos_to_message() {
        let servos = Servos {
            base: 100,
            shoulder: 200,
            elbow: 50,
            claw: 1,
        };

        let actual = servos.to_message();
        let expected: Vec<u8> = vec![100, 0, 200, 0, 50, 0, 1, 0];

        assert_eq!(actual, expected);
    }
}
