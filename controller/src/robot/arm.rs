use crate::{Joint, Servos};

/// Defines the arm of the robot
///
#[derive(Debug)]
pub struct Arm {
    /// Horizontal rotation (or azmut)
    pub base: Joint,

    /// Vertical joint connected to [`Arm::base`] for elevation and outward translation
    pub shoulder: Joint,

    /// Vertical joint connected to [`Arm::shoulder`] using a arm segment,
    /// also for elevation and outward translation
    pub elbow: Joint,

    /// Claw joint for opening and closing the claw
    pub claw: Joint,
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
