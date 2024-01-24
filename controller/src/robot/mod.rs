use crate::{
    communication::{ComError, Connection},
    kinematics::joints::Joint,
};
use gilrs::{Button, Gamepad};
use std::cmp::PartialEq;

pub mod arm;
pub mod movement;

/// Defines a robot and its physical properties
#[derive(Debug)]
pub struct Robot {
    pub movement: movement::Movement,

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
    pub fn parse_gamepad_axis(input: f64, deadzone: f64) -> f64 {
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
        self.movement.update_inputs((
            Self::parse_gamepad_axis(gamepad.value(gilrs::Axis::LeftStickX) as f64, 0.2),
            Self::parse_gamepad_axis(gamepad.value(gilrs::Axis::LeftStickY) as f64, 0.2),
            Self::parse_gamepad_axis(gamepad.value(gilrs::Axis::RightStickX) as f64, 0.2),
        ));

        if gamepad.is_pressed(Button::Start) {
            panic!("Start button pressed, there is only death now");
        }
    }

    /// Runs all of the necessary function in order to update controller and move the robot
    pub fn update(&mut self, delta: f64) -> Result<(), ComError> {
        self.movement.update(delta);
        match self.movement.get_arm(self.upper_arm, self.lower_arm) {
            Ok((base, shoulder, elbow)) => {
                self.arm.base.angle = base;
                self.arm.shoulder.angle = shoulder;
                self.arm.elbow.angle = elbow;
            }
            Err(_) => return Ok(()),
        };

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
        dbg!(((MAX_SERVO - MIN_SERVO) as f64 * factor + self.min as f64) as u16)
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
        assert_eq!(0., Robot::parse_gamepad_axis(0.1, 0.2));
        assert_eq!(0., Robot::parse_gamepad_axis(0.2, 0.2));
        assert_eq!(1., Robot::parse_gamepad_axis(1., 0.2));
    }
}
