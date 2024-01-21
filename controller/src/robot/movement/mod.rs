use gilrs::Gamepad;

use crate::arm::Arm;

pub mod full;
pub mod medium;
pub mod noassist;

#[derive(Debug, Copy, Clone)]
pub struct Movement {
    pub mode: Mode,
    pub acceleration: f64,
}

#[derive(Debug, Copy, Clone)]
pub enum Mode {
    Full(full::Full),
    Medium(medium::Medium),
    NoAssist(noassist::NoAssist),
}

impl Movement {
    pub fn new(mode: Mode, acceleration: f64) -> Self {
        Self {
            mode,
            acceleration,
        }
    }

    pub fn get_arm(&self, upper_arm: f64, lower_arm: f64) -> Result<(f64,f64,f64), ()> {
        match &self.mode {
            Mode::Full(mode) => mode.get_arm(upper_arm, lower_arm),
            Mode::Medium(mode) => mode.get_arm(upper_arm, lower_arm),
            Mode::NoAssist(mode) => mode.get_arm(upper_arm, lower_arm),
        }
    }

    pub fn update_inputs(&mut self, inputs: (f64, f64, f64)) {
        match &mut self.mode {
            Mode::Full(mode) => mode.update_inputs(inputs),
            Mode::Medium(mode) => mode.update_inputs(inputs),
            Mode::NoAssist(mode) => mode.update_inputs(inputs),
        }
    }

    pub fn update(&mut self, delta: f64) {
        match &mut self.mode {
            Mode::Full(mode) => mode.update(delta, self.acceleration),
            Mode::Medium(mode) => mode.update(delta, self.acceleration),
            Mode::NoAssist(mode) => mode.update(delta, self.acceleration),
        }
    }

    pub fn display(&self) -> String {
        match &self.mode {
            Mode::Full(mode) => mode.display(),
            Mode::Medium(mode) => mode.display(),
            Mode::NoAssist(mode) => mode.display(),
        }
    }
}

pub trait MovementMode {
    fn display(&self) -> String;

    /// Update velocity and position
    fn update(&mut self, delta: f64, acceleration: f64);

    /// Calculate the arm position that the robot is in
    fn get_arm(&self, upper_arm: f64, lower_arm: f64) -> Result<(f64, f64, f64), ()>;

    /// update values form user input
    fn update_inputs(&mut self, inputs: (f64, f64, f64));

    /// Convert to full mode
    fn into_full(self) -> full::Full;

    /// Convert to medium mode
    fn into_medium(self) -> medium::Medium;

    /// Convert to no assist mode
    fn into_noassist(self) -> noassist::NoAssist;
}
