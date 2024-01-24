use crate::kinematics::triangle;
use core::{f64::consts::PI, fmt::Debug};

/// A arm joint with limits and functions for calculating pivot angle
pub struct Joint {
    pub angle: f64,
    pub min: f64,
    pub max: f64,
    pub motion: MotionField,
}

/// Type association for Motion trait that implements debug
pub type MotionField = Box<dyn Motion>;

/// A double linkage based motion system
///
/// The controlled angle is connected to the arm using two rods.
/// One of the rods is tied to the controlled pivot point.
/// the other is connected between the first rod and the arm
pub struct DoubleLinkage {
    /// Distance from the pivot to the connection point
    pub connection_radial_offset: f64,

    /// distance from the centerline of the arm to the connection point
    pub connection_linear_offset: f64,

    /// how far behind the controlled pivot is from the arm pivot
    pub controll_pivot_horizontal_offset: f64,
    /// how far above the controlled pivot is from the arm pivot
    pub controll_pivot_vertical_offset: f64,

    /// Length or rod connected to controller pivot
    pub controller_pivot_rod_length: f64,

    /// Length of rod connecting `controller_pivot_rod_length` to connection
    pub connection_rod_length: f64,
}

/// A direct drive based motion system
///
/// The controlled angle is directly connected to the arm
pub struct DirectDrive {}

/// A direct drive motions system with a offset
///
/// The controlled angle is directly connected to the arm but with a offset
pub struct DirectDriveOffset {
    pub offset: f64,
}

/// A gear drive based motion system
///
/// The controlled angle is connected to the arm and a gear ratio is used when calculating the
/// controlled angle
pub struct GearDrive {
    pub gear_ratio: f64,
}

/// Trait for join motion
pub trait Motion {
    fn get_pivot_angle(&self, target: f64) -> f64;
}

impl DirectDrive {
    pub fn new() -> DirectDrive {
        DirectDrive {}
    }
}

impl DoubleLinkage {
    pub fn new(
        connection_radial_offset: f64,
        connection_linear_offset: f64,
        controll_pivot_horizontal_offset: f64,
        controll_pivot_vertical_offset: f64,
        controller_pivot_rod_length: f64,
        connection_rod_length: f64,
    ) -> Self {
        Self {
            connection_radial_offset,
            connection_linear_offset,
            controll_pivot_horizontal_offset,
            controll_pivot_vertical_offset,
            controller_pivot_rod_length,
            connection_rod_length,
        }
    }

    /// Calculate the angle and distance between the arm pivot and the arm connection point
    ///
    /// # Returns
    /// (angle, distance)
    /// angle in radians
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::DoubleLinkage;
    /// let linkage = DoubleLinkage::new(1., 1., 1., 1., 1., 1.);
    /// let (angle, distance) = linkage.connection_offset();
    /// ```
    pub fn connection_offset(&self) -> (f64, f64) {
        let angle = (self.connection_linear_offset / self.connection_radial_offset).atan();
        let distance =
            (self.connection_radial_offset.powi(2) + self.connection_linear_offset.powi(2)).sqrt();

        (angle, distance)
    }

    /// Calculate the angle and distance between the arm pivot and the arm connection point
    ///
    /// # Returns
    /// (angle, distance)
    /// angle in radians
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::DoubleLinkage;
    ///
    /// let linkage = DoubleLinkage::new(1., 1., 1., 1., 1., 1.);
    /// let (angle, distance) = linkage.controller_offset();
    /// ```
    pub fn controller_offset(&self) -> (f64, f64) {
        let angle =
            (self.controll_pivot_vertical_offset / self.controll_pivot_horizontal_offset).atan();
        let distance = (self.controll_pivot_horizontal_offset.powi(2)
            + self.controll_pivot_vertical_offset.powi(2))
        .sqrt();

        (angle, distance)
    }
}

impl Joint {
    pub fn new(min: f64, max: f64, motion: MotionField) -> Self {
        Self {
            angle: 0.,
            min,
            max,
            motion,
        }
    }
}

impl Motion for DirectDrive {
    fn get_pivot_angle(&self, target: f64) -> f64 {
        target
    }
}

impl Motion for DoubleLinkage {
    fn get_pivot_angle(&self, target: f64) -> f64 {
        let connection = self.connection_offset();
        let controller = self.controller_offset();

        let inner_target_angle = PI - (target.to_radians() + connection.0 + controller.0);

        let connection_to_controller = triangle::length_from_two_lengths_and_angle(
            inner_target_angle,
            connection.1,
            controller.1,
        );

        let angle = {
            let x = triangle::a_from_lengths(
                connection_to_controller,
                self.controller_pivot_rod_length,
                self.connection_rod_length,
            );

            let y = triangle::a_from_lengths(connection_to_controller, controller.1, connection.1);

            x + y - controller.0
        };

        angle.to_degrees()
    }
}

impl Motion for DirectDriveOffset {
    fn get_pivot_angle(&self, target: f64) -> f64 {
        target + self.offset
    }
}

impl Motion for GearDrive {
    fn get_pivot_angle(&self, target: f64) -> f64 {
        target * self.gear_ratio
    }
}

impl Debug for Joint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Joint")
            .field("angle", &self.angle)
            .field("min", &self.min)
            .field("max", &self.max)
            .field("servo_angle", &self.motion.get_pivot_angle(self.angle))
            .finish()
    }
}

impl Debug for MotionField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotionField")
            .field("motion", &self.get_pivot_angle(0.))
            .finish()
    }
}

impl Default for Joint {
    fn default() -> Self {
        Self {
            angle: 0.,
            min: 0.,
            max: 180.,
            motion: Box::new(DirectDrive::new()),
        }
    }
}

#[cfg(test)]
mod test {
    use crate::kinematics::joints::{DoubleLinkage, Motion};

    #[test]
    fn hoint() {
        let linkage = DoubleLinkage {
            connection_radial_offset: 10.,
            connection_linear_offset: 2.,
            controll_pivot_horizontal_offset: 10.,
            controll_pivot_vertical_offset: 1.,
            controller_pivot_rod_length: 7.,
            connection_rod_length: 10.,
        };

        let controller = linkage.controller_offset();
        assert_eq!(controller.0.to_degrees().round(), 6., "controller angle");
        assert_eq!(controller.1.round(), 10., "controller distance");

        let connection = linkage.connection_offset();
        assert_eq!(connection.0.to_degrees().round(), 11., "connection angle");
        assert_eq!(connection.1.round(), 10., "connection distance");

        let servo_angle = linkage.get_pivot_angle(60.);
        assert_eq!(servo_angle.round(), 58.);
    }
}
