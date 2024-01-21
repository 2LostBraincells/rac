use crate::robot::arm::Arm;

use super::{full::Full, medium::Medium, MovementMode};

/// Controlls the joints of the robot individualy
#[derive(Debug, Copy, Clone)]
pub struct NoAssist {
    pub position: (f64, f64, f64),
    pub velocity: (f64, f64, f64),
    pub target_velocity: (f64, f64, f64),
    pub target_position: Option<(f64, f64, f64)>,
}

impl MovementMode for NoAssist {
    fn update(&mut self, delta: f64, acceleration: f64) {
        todo!()
    }

    fn update_inputs(&mut self, inputs: (f64, f64, f64)) {
        self.target_velocity = inputs;
    }

    fn get_arm(&self, upper_arm: f64, lower_arm: f64) -> Result<(f64, f64, f64), ()> {
        Ok(self.position)
    }

    fn into_full(self) -> Full {
        todo!()
    }

    fn into_medium(self) -> Medium {
        todo!()
    }

    fn into_noassist(self) -> NoAssist {
        self
    }

    fn display(&self) -> String {
        todo!()
    }
}
