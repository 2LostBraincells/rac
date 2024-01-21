use crate::{kinematics::position::MixedVec, robot::arm::Arm};

use super::{full::Full, noassist::NoAssist, MovementMode};

/// Direct control of the arms azimuth. shoulder and elbow are calculated using inverse kinematics
#[derive(Debug, Copy, Clone)]
pub struct Medium {
    pub position: MixedVec,
    pub velocity: MixedVec,
    pub target_velocity: MixedVec,
    pub target_position: Option<MixedVec>,
}

impl MovementMode for Medium {
    fn update(&mut self, delta: f64, acceleration: f64) {
        todo!()
    }

    fn get_arm(&self, upper_arm: f64, lower_arm: f64) -> Result<(f64, f64, f64), ()> {
        self.position.to_position().inverse_kinematics(upper_arm, lower_arm).clone()
    }

    fn update_inputs(&mut self, inputs: (f64, f64, f64)) {
        todo!()
    }

    fn into_full(self) -> Full {
        todo!()
    }

    fn into_medium(self) -> Medium {
        todo!()
    }

    fn into_noassist(self) -> NoAssist {
        todo!()
    }

    fn display(&self) -> String {
        todo!()
    }
}
