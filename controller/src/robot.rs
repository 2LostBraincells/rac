use crate::communication::{Connection, ComError};


// servo angles
pub const MAX_ANGLE: f32 = 180.0;
pub const MIN_ANGLE: f32 = 0.0;

// microseconds for arduino
pub const MAX_SERVO: u16 = 2400;
pub const MIN_SERVO: u16 = 250;

/// Defines a servo angle, but with more functions on it
#[derive(Debug, Copy, Clone)]
pub struct Angle(pub f32);


/// Very specific names for servos
#[derive(Debug)]
pub struct Arm {
    pub base: Angle,
    pub shoulder: Angle,
    pub elbow: Angle,
    pub claw: Angle,
    pub connection: Connection
}

/// convert servo position represented as an angle into values understod by the servo
impl Into<u16> for Angle {
    fn into(self) -> u16 {
        let factor = (self.0 - MIN_ANGLE) / MAX_ANGLE;
        ((MAX_SERVO - MIN_SERVO) as f32 * factor + MIN_SERVO as f32) as u16
    }
}

/// he's average alright
impl Default for Arm {
    fn default() -> Self {
        Self {
            connection: Connection::default(),
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

    pub fn update(&mut self) -> Result<(), ComError> {
        let data = self.to_servos().to_message();
        self.connection.write(&data)
    }
}

/// quirky arm
#[derive(Debug, Copy, Clone)]
pub struct Servos {
    pub base: u16,
    pub shoulder: u16,
    pub elbow: u16,
    pub claw: u16,
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
