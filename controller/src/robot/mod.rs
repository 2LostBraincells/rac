use std::cmp::PartialEq;
use crate::{
    communication::{ComError, Connection},
    kinematics::position::CordinateVec,
    kinematics::joints::Joint,
    logging::warn,
};

use gilrs::{Axis, Button, Gamepad};
pub mod arm;

/// Defines a robot and its physical properties
#[derive(Debug)]
pub struct Robot {
    /// current head position in units
    ///
    /// Represents the current position of the head in 3 dimensions
    ///
    /// NOTE: This value should almost never be set directly, see [`Robot::target_position`]
    pub position: CordinateVec,

    /// target position in units
    ///
    /// Represents a target position for the head to move to
    pub target_position: Option<CordinateVec>,

    /// velocity in units/s
    ///
    /// Represents the current volicity of the head in 3 dimensions
    ///
    /// NOTE: This value should almost never be set directly, see [`Robot::target_velocity`]
    pub velocity: CordinateVec,

    /// maximum velocity in units/s
    ///
    /// Represents the maximum velocity the gamepad can command the head too travel at
    pub max_velocity: CordinateVec,
    pub target_velocity: CordinateVec,

    /// acceleration in units/s^2
    ///
    /// Represents the maximum acceleration the arm can use when moving
    pub acceleration: f64,

    pub arm: arm::Arm,
    pub upper_arm: f64,
    pub lower_arm: f64,
    pub claw_open: bool,
    pub connection: Connection,
}

impl Robot {
    /// handles input from a gamepad axis
    ///
    /// Returns 0 if the axis is within the deadzone
    /// the output value is scaled to be between -1 and 1
    /// when the axis is on the edge of the deadzone the output should not be deadzone + x but 0 + x
    pub fn parse_gamepad_axis(&mut self, input: f64, deadzone: f64) -> f64 {
        if input.abs() < deadzone {
            return 0.;
        }

        input.signum() * {
            let input = input.abs() - deadzone;
            input / (1. - deadzone)
        }
    }

    /// Handles input, updating the relevant values. Change this function to add controller
    /// functionality
    pub fn update_gamepad(&mut self, gamepad: &Gamepad) {
        let right_axis_y = gamepad.value(Axis::RightStickY) as f64;
        let left_axis_x = gamepad.value(Axis::LeftStickX) as f64;
        let left_axis_y = gamepad.value(Axis::LeftStickY) as f64;

        self.target_position = None;

        self.target_velocity = self.max_velocity
            * CordinateVec {
                x: self.parse_gamepad_axis(left_axis_x, 0.2),
                y: self.parse_gamepad_axis(left_axis_y, 0.2),
                z: self.parse_gamepad_axis(right_axis_y, 0.2),
            };

        if gamepad.is_pressed(Button::Start) {
            panic!("Start button pressed, there is only death now");
        }
    }

    /// Set target velocity if a target position is set
    ///
    /// Accelerate towards the target position until within the distance required to stop
    ///
    /// If the target position is reached, set target position to None
    pub fn target_position_update(&mut self, target: CordinateVec) {
        let delta = target - self.position;
        let mut sphere = delta.to_sphere();
        let acceleration = CordinateVec::new(self.acceleration, self.acceleration, self.acceleration);
        let velocity = self.velocity.dst();

        // distance needed to stop at current velocity
        let breaking_distance = dbg!(velocity.powi(2) / (2. * acceleration.dst()));

        // conntineously accelerate until we reach the breaking point
        if sphere.distance < breaking_distance {
            // breake
            self.target_velocity = CordinateVec::new(0., 0., 0.);

            if sphere.distance < 0.04 && velocity < 0.07 {
                // we have reached the target
                self.position = target;
                self.velocity = CordinateVec::new(0., 0., 0.);
                self.target_velocity = CordinateVec::new(0., 0., 0.);
                self.target_position = None;
            }
        } else {
            // accelerate
            sphere.update_dst(10000.);
            self.target_velocity = sphere.to_position();
        }
    }

    /// Update velocity based on acceleration and target velocity
    pub fn update_velocity(&mut self, delta: f64) {
        // actual acceleration for this update step
        let acceleration = self.acceleration * delta;

        // the changle in velocity we need
        let mut delta_velocity = self.target_velocity - self.velocity;

        // limit change to maximum acceleration
        delta_velocity.cube_clamp(-acceleration, acceleration);

        // update position and velocity
        self.velocity += delta_velocity;
    }

    /// Use current velocity to update position
    pub fn update_position(&mut self, delta: f64) {
        self.position += self.velocity * delta;

        // limit position to not be outside of the range of motion
        let mut sphere = self.position.to_sphere();

        // clamp distance from origin
        if sphere.distance >= self.upper_arm + self.lower_arm {
            sphere.update_dst(self.upper_arm + self.lower_arm);
            self.position = sphere.to_position();
        }
    }

    pub fn update_ik(&mut self) {
        let angles = self
            .position
            .inverse_kinematics(self.upper_arm, self.lower_arm);

        match angles {
            Ok(angles) => {
                self.arm.base.angle = angles.0;
                self.arm.shoulder.angle = angles.1;
                self.arm.elbow.angle = angles.2;
            }

            Err(()) => warn("Could not calculate inverse kinematics"),
        }
    }

    /// Runs all of the necessary function in order to update controller and move the robot
    pub fn update(&mut self, delta: f64) -> Result<(), ComError> {
        match self.target_position {
            Some(target) => self.target_position_update(target),
            None => {}
        }

        self.update_velocity(delta);
        self.update_position(delta);
        self.update_ik();

        let data = self.arm.to_servos().to_message();
        self.connection.write(&data, true)
    }
}

// microseconds for arduino
const MAX_SERVO: u16 = 2400;
const MIN_SERVO: u16 = 250;
/// quirky arm
#[derive(Debug, Copy, Clone)]
pub struct Servos {
    pub base: u16,
    pub shoulder: u16,
    pub elbow: u16,
    pub claw: u16,
}


/// convert servo position represented as an angle into values understod by the servo
impl Joint {
    fn into_servo(&self) -> u16 {
        let factor = (self.motion.get_pivot_angle(self.angle) - self.min) / self.max;
        ((MAX_SERVO - MIN_SERVO) as f64 * factor + self.min as f64) as u16
    }
}

impl PartialEq for Joint {
    fn eq(&self, other: &Self) -> bool {
        let left = (self.angle * 10.0f64.powi(4)).round() / 10.0f64.powi(4);
        let right = (other.angle * 10.0f64.powi(4)).round() / 10.0f64.powi(4);
        left == right
    }
}


impl Servos {
    pub fn to_message(&self) -> Vec<u8> {
        unsafe { std::mem::transmute::<Box<Servos>, &[u8; 8]>(Box::new(*self)) }.to_vec()
    }
}

#[cfg(test)]
mod test {
    use crate::arm::Arm;
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

    #[test]
    pub fn parse_gamepad() {
        let mut robo = Robot {
            position: CordinateVec::new(0., 0., 0.),
            target_position: None,
            velocity: CordinateVec::new(0., 0., 0.),
            max_velocity: CordinateVec::new(100., 100., 100.),
            target_velocity: CordinateVec::new(0., 0., 0.),
            acceleration: 100.,
            arm: Arm::default(),
            upper_arm: 100.,
            lower_arm: 100.,
            claw_open: false,
            connection: Connection::default(),
        };

        assert_eq!(0., robo.parse_gamepad_axis(0.1, 0.2));
        assert_eq!(0., robo.parse_gamepad_axis(0.2, 0.2));
        assert_eq!(1., robo.parse_gamepad_axis(1., 0.2));
    }
}
