use std::{cmp::max, f64::consts::PI};

use crate::robot::*;

use self::geometry::triangle::a_from_lengths;

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

impl Robot {
    pub fn inverse_kinematics(&mut self) {
        let pos = &self.position;
        let spos = &self.position.to_sphere();

        let beta = a_from_lengths(self.lower_arm, self.upper_arm, spos.dst);

        let alpha = {
            let x = (spos.f_dst / pos.y).atan();
            let y = a_from_lengths(spos.dst, self.lower_arm, self.upper_arm);

            if x + y > PI / 2. { 
                PI - x - y
            } else {
                x + y
            }
        };

        self.angles.base = Angle(spos.azmut.to_degrees() + 90.);
        self.angles.shoulder = Angle(alpha.to_degrees());
        self.angles.elbow = Angle(beta.to_degrees());
    }
}

#[cfg(test)]
mod test {
    use std::f64::consts::SQRT_2;

    use crate::kinematics::Robot;

    #[test]
    fn angle_test() {
        let mut robot = Robot::new(1., 1.);
        robot.position.x = 0.;
        robot.position.z = SQRT_2;
        robot.position.y = 0.;

        robot.inverse_kinematics();

        assert_eq!(robot.angles.base.inner().round(), 0.);
        assert_eq!(robot.angles.shoulder.inner().round(), 45.);
        assert_eq!(robot.angles.elbow.inner().round(), 90.);
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

impl Position {
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
}
