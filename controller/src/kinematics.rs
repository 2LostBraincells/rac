use std::cmp::max;

use crate::robot::*;
use gilrs::{Button, Event, Gamepad, Gilrs, Axis};

const MAX_SPEED: f32 = 0.25;
const DEAD_ZONE: f32 = 0.1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_test() {
        let mut robot = Robot::new(10., 10.);
    }
}

pub struct Position {
    x: f32,
    y: f32,
    z: f32,
}

pub struct Robot {
    pub position: Position,
    pub angles: Arm,
    pub upper_arm: f32,
    pub lower_arm: f32,
    pub square_sum: f32,
    pub claw_open: bool,
}

impl Default for Position {
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }
}

impl Robot {
    pub fn new(lower_arm: f32, upper_arm: f32) -> Robot {
        Robot {
            position: Position::default(),
            angles: Arm::default(),
            upper_arm,
            lower_arm,
            square_sum: upper_arm * upper_arm + lower_arm * lower_arm,
            claw_open: false,
        }
    }

    pub fn inverse_kinematics(&mut self) {
        let x = &self.position.x;
        let y = &self.position.y;
        let z = &self.position.z;

        let distance = (x*x + z*z).sqrt();

        let beta;
        let alpha;
        let theta;

        theta = (x / z).atan().to_degrees();
        beta = ((self.square_sum + distance * distance + y * y) / (2. * self.upper_arm * self.lower_arm))
            .acos()
            .to_degrees();
        alpha = ((distance / y).atan() + ((self.upper_arm * beta.sin()) / (distance * distance + y * y).sqrt()).asin())
            .to_degrees();

        self.angles.base = Angle(theta);
        self.angles.shoulder = Angle(alpha);
        self.angles.elbow = Angle(beta);
    }

    pub fn update_position(&mut self, gamepad: Gamepad) {
        let right_stick_axis_x = gamepad.value(Axis::RightStickX);
        let right_stick_axis_y = gamepad.value(Axis::RightStickY);
        let left_stick_axis_x = gamepad.value(Axis::LeftStickX);
        let left_stick_axis_y = gamepad.value(Axis::LeftStickY);

        if right_stick_axis_x.abs() > DEAD_ZONE { self.position.z += MAX_SPEED * right_stick_axis_x; }
        if right_stick_axis_y.abs() > DEAD_ZONE { self.position.x += MAX_SPEED * right_stick_axis_y; }
        if left_stick_axis_y.abs() > DEAD_ZONE { self.position.y += MAX_SPEED * left_stick_axis_x; }
        if gamepad.is_pressed(Button::LeftTrigger2) { self.claw_open = !self.claw_open; }
    }

    
}
