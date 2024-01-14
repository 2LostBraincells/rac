use crate::communication::{ComError, Connection};
use gilrs::{Axis, Button, Gamepad};

// controller constants pub const MAX_SPEED: f64 = 0.25;
pub const DEAD_ZONE: f64 = 0.2;
pub const MAX_SPEED: f64 = 10.;
pub const MAX_TRAVEL_SPEED: f64 = 10.;

// servo angles
pub const MAX_ANGLE: f64 = 180.0;
pub const MIN_ANGLE: f64 = 0.0;

// microseconds for arduino
pub const MAX_SERVO: u16 = 2400;
pub const MIN_SERVO: u16 = 250;

/// Defines a robot and its physical properties
pub struct Robot {
    pub position: Position,
    pub target_position: Position,
    pub angles: Arm,
    pub upper_arm: f64,
    pub lower_arm: f64,
    pub square_sum: f64,
    pub claw_open: bool,
    pub claw_button_pressed: bool,
    pub connection: Connection,
    pub macros: Macros,
}

pub struct Macros(Position, Position, Position, Position);

/// Defines a position in 3d space
#[derive(Debug, Copy, Clone)]
pub struct Position {
    /// Width
    pub x: f64,

    /// Height
    pub y: f64,

    /// Depth
    pub z: f64,
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
            target_position: Position::default(),
            angles: Arm::default(),
            upper_arm,
            lower_arm,
            square_sum: upper_arm * upper_arm + lower_arm * lower_arm,
            claw_open: false,
            claw_button_pressed: false,
            connection: Connection::default(),
            macros: Macros::default(),
        }
    }

    pub fn update_controller(&mut self, gamepad: &Gamepad, delta: f64) {
        let previous_position = self.target_position.clone();

        let right_stick_axis_x = gamepad.value(Axis::RightStickX) as f64;
        let right_stick_axis_y = gamepad.value(Axis::RightStickY) as f64;
        let left_stick_axis_x = gamepad.value(Axis::LeftStickX) as f64;
        let left_stick_axis_y = gamepad.value(Axis::LeftStickY) as f64;

        let z_speed = MAX_SPEED * delta * left_stick_axis_y;
        let x_speed = MAX_SPEED * delta * left_stick_axis_x;
        let y_speed = MAX_SPEED * delta * right_stick_axis_y;

        if left_stick_axis_y.abs() > DEAD_ZONE {
            self.target_position.z += z_speed;
        }
        if left_stick_axis_x.abs() > DEAD_ZONE {
            self.target_position.x += x_speed;
        }
        if right_stick_axis_y.abs() > DEAD_ZONE {
            self.target_position.y += y_speed;
        }
        if gamepad.is_pressed(Button::LeftTrigger2) {
            if !self.claw_button_pressed {
                self.claw_open = !self.claw_open;
            }
            self.claw_button_pressed = true;
        }
        if !gamepad.is_pressed(Button::LeftTrigger2) {
            self.claw_button_pressed = false;
        }

        if gamepad.is_pressed(Button::DPadUp) {
            if gamepad.is_pressed(Button::Select) {
                self.macros.0 = self.position.clone();
            } else {
                self.target_position = self.macros.0.clone();
            }
        }
        if gamepad.is_pressed(Button::DPadRight) {
            if gamepad.is_pressed(Button::Select) {
                self.macros.1 = self.position.clone();
            } else {
                self.target_position = self.macros.1.clone();
            }
        }
        if gamepad.is_pressed(Button::DPadDown) {
            if gamepad.is_pressed(Button::Select) {
                self.macros.2 = self.position.clone();
            } else {
                self.target_position = self.macros.2.clone();
            }
        }
        if gamepad.is_pressed(Button::DPadLeft) {
            if gamepad.is_pressed(Button::Select) {
                self.macros.3 = self.position.clone();
            } else {
                self.target_position = self.macros.3.clone();
            }
        }

        if gamepad.is_pressed(Button::Start) {
            panic!(
                "Start button pressed \
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
FU§CK SHIT FUC KSHIRTA SSHIUTw\
"
            );
        }

        if self.target_position.dst() > self.upper_arm + self.lower_arm
            && self.target_position.polar() < 0.
            && self.target_position.azmut() > 0.
            && self.target_position.azmut() < 180.
        {
            self.target_position = previous_position;
        }
    }

    pub fn update_position(&mut self) {
        let mut delta_sphere = (self.target_position - self.position).to_sphere();
        delta_sphere.dst = MAX_TRAVEL_SPEED.min(delta_sphere.dst);
        self.position += delta_sphere.to_position();
    }

    pub fn update_ik(&mut self) {
        self.angles = Arm {
            claw: self.angles.claw,
            ..self
                .position
                .inverse_kinematics(self.upper_arm, self.lower_arm)
        };
    }

    pub fn update_claw(&mut self) {
        if self.claw_open {
            self.angles.claw = Angle(180.);
        } else {
            self.angles.claw = Angle(0.);
        }
    }

    pub fn update(&mut self, gamepad: &Gamepad, delta: f64) -> Result<(), ComError> {
        self.update_controller(gamepad, delta);
        self.update_position();
        self.update_ik();
        self.update_claw();
        let data = self.angles.to_servos().to_message();
        self.connection.write(&data, true)
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

impl Default for Macros {
    fn default() -> Self {
        Self(
            Default::default(),
            Default::default(),
            Default::default(),
            Default::default(),
        )
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
