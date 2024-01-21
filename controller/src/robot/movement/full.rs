use std::fmt::Debug;

use crate::kinematics::position::CordinateVec;

use super::{medium::Medium, noassist::NoAssist, MovementMode};

/// All joints are controlled using inverse kinematics
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Full {
    pub position: CordinateVec,
    pub velocity: CordinateVec,
    pub target_velocity: CordinateVec,
    pub max_velocity: CordinateVec,
    pub target_position: Option<CordinateVec>,
}

impl Full {
    /// Update positon and velocity based on acceleration and target velocity
    pub fn update_position(&mut self, delta: f64, acceleration: f64) {
        // actual acceleration for this update step
        let acceleration = acceleration * delta;

        // the changle in velocity we need
        let mut delta_velocity = self.target_velocity - self.velocity;

        // limit change to maximum acceleration
        delta_velocity.cube_clamp(-acceleration, acceleration);

        // update position and velocity
        self.velocity += delta_velocity * 0.5;
        self.position += self.velocity * delta;
        self.velocity += delta_velocity * 0.5;
    }

    /// Move towards a target position
    pub fn goto_target(&mut self, time_delta: f64, acceleration: f64, target: CordinateVec) {
        let delta = target - self.position;
        let distance = delta.dst();
        let velocity = self.velocity.dst();

        // Check if we are within the required distance to stop
        if distance > velocity.powi(2) / (2.0 * acceleration) {
            // we are not close enough to the target to start breaking
            // accelerate towards the target

            let delta_velocity = {
                let mut delta_velocity = delta.to_sphere();
                delta_velocity.update_dst(acceleration * time_delta * 0.5);
                delta_velocity.to_position()
            };

            self.velocity += delta_velocity;
            self.position += self.velocity * time_delta;
            self.velocity += delta_velocity;

            return;
        }

        // we are close enough to the target to start breaking

        if distance < 0.04 && velocity < acceleration * time_delta {
            // we have reached the target
            self.position = target;
            self.velocity = CordinateVec::new(0.0, 0.0, 0.0);
            self.target_velocity = CordinateVec::new(0.0, 0.0, 0.0);
            self.target_position = None;

            return;
        }

        let delta_velocity = {
            let mut delta_velocity = self.velocity.to_sphere();
            delta_velocity.update_dst(acceleration * time_delta * 0.5);
            delta_velocity.to_position()
        };

        self.velocity -= delta_velocity;
        self.position += self.velocity * time_delta;
        self.velocity -= delta_velocity;
    }
}

impl MovementMode for Full {
    fn update(&mut self, delta: f64, acceleration: f64) {
        match self.target_position {
            Some(target) => self.goto_target(delta, acceleration, target),
            None => self.update_position(delta, acceleration),
        }
    }

    fn update_inputs(&mut self, inputs: (f64, f64, f64)) {
        self.target_velocity = {
            CordinateVec {
                x: self.max_velocity.x * inputs.0,
                y: self.max_velocity.y * inputs.1,
                z: self.max_velocity.z * inputs.2,
            }
        };
    }

    fn get_arm(&self, upper_arm: f64, lower_arm: f64) -> Result<(f64, f64, f64), ()> {
        self.position.inverse_kinematics(upper_arm, lower_arm)
    }

    fn into_full(self) -> Full {
        todo!()
    }

    /// Convert to medium mode
    ///
    /// # Examples
    /// ```rust
    /// # use robot::movement::{MovementMode, full::Full};
    /// # use robot::movement::medium::Medium;
    /// # use robot::kinematics::position::CordinateVec;
    /// let full = Full {
    ///    position: CordinateVec::new(1.0, 1.0, 1.0),
    ///    velocity: CordinateVec::new(0.0, 0.0, 0.0),
    ///    target_velocity: CordinateVec::new(0.0, 0.0, 0.0),
    ///    target_position: None,
    /// };
    ///
    /// let back: Medium = full.into_medium().into_full();
    ///
    /// assert_eq!(back, full);
    /// ```
    fn into_medium(self) -> Medium {
        todo!("Implement into_medium for Full");
    }

    fn into_noassist(self) -> NoAssist {
        todo!()
    }

    fn display(&self) -> String {
        let mut string = String::new();
        string
    }
}
