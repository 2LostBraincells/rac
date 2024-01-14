use crate::communication::{ComError, Connection};
use gilrs::{Axis, Button, Gamepad};

// controller constants
pub const MAX_SPEED: f32 = 0.25;
pub const DEAD_ZONE: f32 = 0.1;

// servo angles
pub const MAX_ANGLE: f32 = 180.0;
pub const MIN_ANGLE: f32 = 0.0;

// microseconds for arduino
pub const MAX_SERVO: u16 = 2400;
pub const MIN_SERVO: u16 = 250;

/// Defines a robot and its physical properties
pub struct Robot {
    pub position: Position,
    pub angles: Arm,
    pub upper_arm: f32,
    pub lower_arm: f32,
    pub square_sum: f32,
    pub claw_open: bool,
    pub connection: Connection,
}

/// Defines a position in 3d space
#[derive(Debug, Copy, Clone)]
pub struct Position {
    /// Width
    pub x: f32,

    /// Height
    pub y: f32,

    /// Depth
    pub z: f32,
}

/// Defines a servo angle, but with more functions on it
#[derive(Debug, Copy, Clone)]
pub struct Angle(pub f32);

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
    pub fn new(lower_arm: f32, upper_arm: f32) -> Robot {
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

    pub fn update_position(&mut self, gamepad: &Gamepad) {
        let right_stick_axis_x = gamepad.value(Axis::RightStickX);
        let right_stick_axis_y = gamepad.value(Axis::RightStickY);
        let left_stick_axis_x = gamepad.value(Axis::LeftStickX);
        let left_stick_axis_y = gamepad.value(Axis::LeftStickY);

        if right_stick_axis_x.abs() > DEAD_ZONE {
            self.position.z += MAX_SPEED * right_stick_axis_x;
        }
        if right_stick_axis_y.abs() > DEAD_ZONE {
            self.position.x += MAX_SPEED * right_stick_axis_y;
        }
        if left_stick_axis_y.abs() > DEAD_ZONE {
            self.position.y += MAX_SPEED * left_stick_axis_x;
        }
        if gamepad.is_pressed(Button::LeftTrigger2) {
            self.claw_open = !self.claw_open;
        }
    }

    pub fn update(&mut self, gamepad: &Gamepad) -> Result<(), ComError> {
        self.update_position(gamepad);
        self.inverse_kinematics();
        let data = self.angles.to_servos().to_message();
        self.connection.write(&data)
    }
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

/// convert servo position represented as an angle into values understod by the servo
impl Into<u16> for Angle {
    fn into(self) -> u16 {
        let factor = (self.0 - MIN_ANGLE) / MAX_ANGLE;
        ((MAX_SERVO - MIN_SERVO) as f32 * factor + MIN_SERVO as f32) as u16
    }
}

impl Angle {
    pub fn inner(&self) -> &f32 {
        &self.0
    }

    pub fn inner_mut(&mut self) -> &mut f32 {
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
