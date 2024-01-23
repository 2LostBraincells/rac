use std::ops::{Mul, SubAssign, Sub, AddAssign};

use crate::robot::arm::Arm;

use super::{full::Full, medium::Medium, MovementMode};

#[derive(Debug, Copy, Clone)]
struct Joints {
    base: f64,
    shoulder: f64,
    elbow: f64,
}

/// Controlls the joints of the robot individualy
#[derive(Debug, Copy, Clone)]
pub struct NoAssist {
    pub position: Joints,
    pub velocity: Joints,
    pub target_velocity: Joints,
    pub target_position: Option<Joints>,
}

impl NoAssist {
    fn update_position(&mut self, delta: f64, acceleration: f64) {
        let mut velocity_delta = self.target_velocity - self.velocity;
        velocity_delta.cube_clamp(-acceleration * delta*0.5, acceleration * delta*0.5);

        self.velocity += velocity_delta;
        self.position += self.velocity * delta;
        self.velocity += velocity_delta;
    }

    fn goto_target(&mut self, time_delta: f64, acceleration: f64, target: Joints) {
        let delta = target - self.position;

        let update = |position: &mut f64, velocity: &mut f64, delta: f64| {
            let mut delta_velocity = delta - *velocity;
            delta_velocity = delta_velocity.clamp(-acceleration * time_delta * 0.5, acceleration * time_delta * 0.5);

            *velocity += delta_velocity;
            *position += *velocity * time_delta;
            *velocity += delta_velocity;
        };
    }
}

impl MovementMode for NoAssist {
    fn update(&mut self, delta: f64, acceleration: f64) {
        todo!()
    }

    fn update_inputs(&mut self, inputs: (f64, f64, f64)) {
        self.target_velocity = Joints::from_tuple(inputs);
    }

    fn get_arm(&self, upper_arm: f64, lower_arm: f64) -> Result<(f64, f64, f64), ()> {
        Ok(self.position.to_tupple())
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

impl Joints {
    pub fn cube_clamp(&mut self, min: f64, max: f64) {
        self.base = self.base.clamp(min, max);
        self.shoulder = self.shoulder.clamp(min, max);
        self.elbow = self.elbow.clamp(min, max);
    }

    pub fn to_tupple(&self) -> (f64, f64, f64) {
        (self.base, self.shoulder, self.elbow)
    }

    pub fn from_tuple(tuple: (f64, f64, f64)) -> Self {
        Joints {
            base: tuple.0,
            shoulder: tuple.1,
            elbow: tuple.2,
        }
    }
}


impl Mul<f64> for Joints {
    type Output = Joints;

    fn mul(self, rhs: f64) -> Self::Output {
        Joints {
            base: self.base * rhs,
            shoulder: self.shoulder * rhs,
            elbow: self.elbow * rhs,
        }
    }
}

impl Sub for Joints {
    type Output = Joints;

    fn sub(self, rhs: Self) -> Self::Output {
        Joints {
            base: self.base - rhs.base,
            shoulder: self.shoulder - rhs.shoulder,
            elbow: self.elbow - rhs.elbow,
        }
    }
}

impl AddAssign for Joints {
    fn add_assign(&mut self, rhs: Self) {
        self.base += rhs.base;
        self.shoulder += rhs.shoulder;
        self.elbow += rhs.elbow;
    }
}

impl SubAssign for Joints {
    fn sub_assign(&mut self, rhs: Self) {
        self.base -= rhs.base;
        self.shoulder -= rhs.shoulder;
        self.elbow -= rhs.elbow;
    }
}
