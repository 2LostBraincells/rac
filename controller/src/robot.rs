use crate::{communication::{ComError, Connection}, kinematics::Position};
use gilrs::{Axis, Button, Gamepad};

// controller constants pub const MAX_SPEED: f64 = 0.25;
pub const DEAD_ZONE: f64 = 0.2;
pub const MAX_SPEED: f64 = 10.;

// servo angles
pub const MAX_ANGLE: f64 = 180.0;
pub const MIN_ANGLE: f64 = 0.0;

// microseconds for arduino
pub const MAX_SERVO: u16 = 2400;
pub const MIN_SERVO: u16 = 250;

/// Defines a robot and its physical properties
pub struct Robot {
    pub position: Position,
    pub angles: Arm,
    pub upper_arm: f64,
    pub lower_arm: f64,
    pub square_sum: f64,
    pub claw_open: bool,
    pub connection: Connection,
}

/// Defines a servo angle, but with more functions on it
#[derive(Debug, Copy, Clone)]
pub struct Angle(pub f64);

/// Very specific names for servos
#[derive(Debug, Copy, Clone)]
pub struct Arm {
    pub base: Angle,
    pub shoulder: Angle,
    pub elbow: Angle,
    pub claw: Angle,
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
            angles: Arm::default(),
            upper_arm,
            lower_arm,
            square_sum: upper_arm * upper_arm + lower_arm * lower_arm,
            claw_open: false,
            connection: Connection::default(),
        }
    }

    pub fn update_position(&mut self, gamepad: &Gamepad, delta: f64) {
        let previous_position = self.position.clone();

        let right_stick_axis_y = gamepad.value(Axis::RightStickY) as f64;
        let left_stick_axis_x = gamepad.value(Axis::LeftStickX) as f64;
        let left_stick_axis_y = gamepad.value(Axis::LeftStickY) as f64;

        let z_speed = MAX_SPEED * delta * left_stick_axis_y;
        let x_speed = MAX_SPEED * delta * left_stick_axis_x;
        let y_speed = MAX_SPEED * delta * right_stick_axis_y;

        if left_stick_axis_y.abs() > DEAD_ZONE {
            self.position.z += z_speed;
        }
        if left_stick_axis_x.abs() > DEAD_ZONE {
            self.position.x += x_speed;
        }
        if right_stick_axis_y.abs() > DEAD_ZONE {
            self.position.y += y_speed;
        }
        if gamepad.is_pressed(Button::LeftTrigger2) {
            self.claw_open = !self.claw_open;
        }
        if gamepad.is_pressed(Button::Start) {
            panic!("Start button pressed");
        }

        if self.position.to_sphere().dst > self.upper_arm + self.lower_arm {
            self.position = previous_position;
        }
    }

    pub fn update_ik(&mut self) {
        self.angles = Arm {
            claw: self.angles.claw,
            ..self.position.inverse_kinematics(self.upper_arm, self.lower_arm)
        }

    }

    pub fn update(&mut self, gamepad: &Gamepad, delta: f64) -> Result<(), ComError> {
        self.update_position(gamepad, delta);
        self.update_ik();
        let data = self.angles.to_servos().to_message();
        self.connection.write(&data, true)
    }
}


/// convert servo position represented as an angle into values understod by the servo
impl Into<u16> for Angle {
    fn into(self) -> u16 {
        let factor = (self.0 - MIN_ANGLE) / MAX_ANGLE;
        ((MAX_SERVO - MIN_SERVO) as f64 * factor + MIN_SERVO as f64) as u16
    }
}

impl Angle {
    pub fn inner(&self) -> &f64 {
        &self.0
    }

    pub fn inner_mut(&mut self) -> &mut f64 {
        &mut self.0
    }
}

/// he's average alright
impl Default for Arm {
    fn default() -> Self {
        Self {
            base: Angle(0.),
            shoulder: Angle(0.),
            elbow: Angle(0.),
            claw: Angle(0.),
        }
    }
}

/// Arm functions
impl Arm {
    pub fn to_servos(&self) -> Servos {
        Servos {
            base: self.base.into(),
            shoulder: self.shoulder.into(),
            elbow: self.elbow.into(),
            claw: self.claw.into(),
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
