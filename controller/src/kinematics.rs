use std::f64::consts::PI;

use crate::robot::*;

use self::geometry::triangle::a_from_lengths;

/// Defines a position in 3d space
#[derive(Debug, Copy, Clone)]
pub struct Position {
    /// Side to side
    pub x: f64,

    /// Up and down
    pub y: f64,

    /// Forward and backward
    pub z: f64,
}

#[derive(Debug, Copy, Clone)]
pub struct SpherePos {
    /// Azmut angle
    pub azmut: f64,

    /// Polar angle
    pub polar: f64,

    /// 3d distance from origin
    pub dst: f64,

    /// Distance from origin on flat ground
    pub f_dst: f64,
}

impl Position {
    /// Calculates the angles for the arm to reach a position
    ///
    /// # Arguments
    /// * `upper_arm` - The length of the upper Arm
    /// * `lower_arm` - The length of the lower Arm
    ///
    /// # Returns
    /// * `Arm` - The angles for the arm to reach the position
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::Position;
    ///
    /// let mut position = Position::new(0., 0., 0.);
    ///
    /// let arm = robot.inverse_kinematics(10,10);
    /// ```
    pub fn inverse_kinematics(&mut self, upper_arm: f64, lower_arm: f64) -> Arm {
        let spos = &self.to_sphere();

        let beta = a_from_lengths(upper_arm, lower_arm, spos.dst);

        let alpha = {
            // arctan(f_dst / y)
            let a = (spos.f_dst / self.y).atan();
            let b = a_from_lengths(spos.dst, lower_arm, upper_arm);

            if a + b > PI / 2. {
                PI - a - b
            } else {
                a + b
            }
        };

        Arm {
            base: Angle(spos.azmut.to_degrees() + 90.),
            shoulder: Angle(alpha.to_degrees()),
            elbow: Angle(beta.to_degrees()),
            claw: Angle(0.),
        }
    }

    pub fn to_sphere(&self) -> SpherePos {
        // sqrt(X^2 + Z^2)
        let f_dst = (self.x * self.x + self.z * self.z).sqrt();

        // sqrt(X^2 + Y^2 + Z^2)
        let dst = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();

        // arctan(x / z)
        let azmut = (self.x / self.z).atan();

        // arctan(f_dst / y)
        let polar = (f_dst / self.y).atan();

        SpherePos {
            azmut,
            polar,
            dst,
            f_dst,
        }
    }



    /// Creates a new Position
    /// # Arguments
    /// * `x` - Side to side position
    /// * `y` - Up and down position
    /// * `z` - Forward and backward position
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
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

#[cfg(test)]
mod test {
    use std::f64::consts::SQRT_2;

    use crate::{
        kinematics::Position,
        robot::{Angle, Arm},
    };

    #[test]
    fn angle_test() {
        let mut position = Position::new(0., SQRT_2, 0.);

        let actual = position.inverse_kinematics(1., 1.);
        let expected = Arm {
            base: Angle(0.),
            shoulder: Angle(45.),
            elbow: Angle(90.),
            claw: Angle(0.),
        };

        assert_eq!(actual.base.inner(), expected.base.inner());
        assert_eq!(actual.shoulder.inner(), expected.shoulder.inner());
        assert_eq!(actual.elbow.inner(), expected.elbow.inner());
    }
}

mod geometry {
    pub mod triangle {
        /// The angles for the corner between a and b in radians
        ///
        /// x = -c^2 + a^2 + b^2
        /// y = 2ab
        /// arccos(x/y)
        pub fn a_from_lengths(a: f64, b: f64, c: f64) -> f64 {
            let x = -(c * c) + a * a + b * b;
            let y = 2. * a * b;
            (x / y).acos()
        }

        #[cfg(test)]
        mod test {
            use crate::kinematics::geometry::triangle;

            #[test]
            fn a_from_lengths() {
                assert_eq!(triangle::a_from_lengths(3., 4., 5.).to_degrees(), 90.00);
                assert_eq!(
                    triangle::a_from_lengths(2., 2., 2.).to_degrees().round(),
                    60.00
                );
            }
        }
    }
}
