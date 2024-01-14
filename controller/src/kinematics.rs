use std::{cmp::max, f64::consts::PI, ops::{Sub, Add, AddAssign}};

use crate::robot::*;

use self::geometry::triangle::a_from_lengths;

#[derive(Debug, Clone, Copy)]
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

impl Position{
    pub fn inverse_kinematics(&mut self, upper_arm: f64, lower_arm: f64) -> Arm {
        let pos = &self;
        let spos = &self.to_sphere();

        let beta = a_from_lengths(upper_arm, lower_arm, spos.dst);

        let alpha = {
            let x = (spos.f_dst / pos.y).atan();
            let y = a_from_lengths(spos.dst, lower_arm, upper_arm);

            if x + y > PI / 2. { 
                PI - x - y
            } else {
                x + y
            }
        };

        Arm {
            base: Angle(spos.azmut.to_degrees() + 90.),
            shoulder: Angle(alpha.to_degrees()),
            elbow: Angle(beta.to_degrees()),
            claw: Angle(0.)
        }
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

        robot.update_ik();

        assert_eq!(robot.angles.base.inner().round(), 90.);
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

pub struct NoNan(f64);

impl NoNan {
    pub fn inner(&self) -> &f64 {
        if self.0.is_nan() {
            &0.
        } else {
            &self.0
        }
    }
}

impl Position {
    pub fn f_dst(&self) -> f64 {
        (self.x * self.x + self.z * self.z).sqrt()
    }

    pub fn dst(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn azmut(&self) -> f64 {
        (self.x / self.z).atan()
    }

    pub fn polar(&self) -> f64 {
        (self.f_dst() / self.y).atan()
    }

    pub fn to_sphere(&self) -> SpherePos {
        // sqrt(X^2 + Z^2)
        let f_dst = self.f_dst();

        // sqrt(X^2 + Y^2 + Z^2)
        let dst = self.dst();

        // arctan(x / z)
        let azmut = self.azmut();

        // arctan(f_dst / y)
        let polar = self.polar();

        SpherePos {
            azmut: *NoNan(azmut).inner(),
            polar: *NoNan(polar).inner(),
            dst: *NoNan(dst).inner(),
            f_dst: *NoNan(f_dst).inner(),
        }
    }
}

impl Sub for Position {
    type Output = Position;

    fn sub(self, rhs: Position) -> Self::Output {
        Position {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Add for Position {
    type Output = Position;

    fn add(self, rhs: Position) -> Self::Output {
        Position {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Position {
    fn add_assign(&mut self, rhs: Position) {
        *self = Position {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        } 
    }
}

impl SpherePos {
    pub fn to_position(&self) -> Position {
        Position {
            x: self.dst * self.azmut.cos(),
            y: self.dst * self.polar.sin(),
            z: self.dst * self.azmut.sin(),
        }
    }
}
