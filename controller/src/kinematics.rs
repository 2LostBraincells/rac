use std::{
    f64::consts::PI,
    ops::{Add, AddAssign, Sub, SubAssign, Mul},
};

use crate::robot::*;

use self::triangle::a_from_lengths;

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
    /// let mut position = Position::new(1., 1., 1.);
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

    /// Clamp all the values in the position to a range
    ///
    /// # Arguments
    /// * `min` - The minimum value for the position
    /// * `max` - The maximum value for the position
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::Position;
    /// let mut position = Position::new(1., 1., 1.);
    /// position.clamp(0., 0.);
    ///
    /// assert_eq!(position, Position::new(0., 0., 0.));
    /// ```
    pub fn clamp(&mut self, min: f64, max: f64) {
        self.x = self.x.clamp(min, max);
        self.y = self.y.clamp(min, max);
        self.z = self.z.clamp(min, max);
    }

    /// Creates a new Position
    /// # Arguments
    /// * `x` - Side to side position
    /// * `y` - Up and down position
    /// * `z` - Forward and backward position
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl Default for Position {
    #[allow(dead_code)]
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }
}

impl Eq for Position {}
impl PartialEq for Position {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }

    fn ne(&self, other: &Self) -> bool {
        self.x != other.x || self.y != other.y || self.z != other.z
    }
}

impl Sub for Position {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl SubAssign for Position {
    fn sub_assign(&mut self, rhs: Self) {
        *self = Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Add for Position {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Position {
    fn add_assign(&mut self, rhs: Self) {
        *self = Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Mul<Position> for Position {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

impl Mul<f64> for Position {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl SpherePos {
    pub fn to_position(&self) -> Position {
        Position::new(
            self.f_dst * self.azmut.cos(),
            self.dst * self.polar.sin(),
            self.f_dst * self.azmut.sin(),
        )
    }
}

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
        use crate::kinematics::triangle;

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

#[cfg(test)]
mod position {

    use std::f64::consts::SQRT_2;

    use crate::{
        kinematics::Position,
        robot::{Angle, Arm},
    };

    #[test]
    fn inverse_kinematics() {
        let mut position = Position::new(SQRT_2, 0., 0.);

        let actual = position.inverse_kinematics(1., 1.);
        let expected = Arm {
            base: Angle(180.),
            shoulder: Angle(45.),
            elbow: Angle(90.),
            claw: Angle(0.),
        };

        assert_eq!(actual, expected);
    }

    #[test]
    fn addition() {
        let a = Position::new(1., 2., 3.);
        let mut b = Position::new(2., 2., 2.);
        let c = Position::new(3., 2., 1.);

        assert_eq!(a + b, Position::new(3., 4., 5.));
        assert_eq!(a + c, Position::new(4., 4., 4.));

        b += c;
        assert_eq!(b, Position::new(5., 4., 3.));
    }

    #[test]
    fn subtraction() {
        let a = Position::new(1., 2., 3.);
        let mut b = Position::new(2., 2., 2.);
        let c = Position::new(3., 2., 1.);

        assert_eq!(a - b, Position::new(-1., 0., 1.));
        assert_eq!(a - c, Position::new(-2., 0., 2.));

        b -= c;

        assert_eq!(b, Position::new(-1., 0., 1.));
    }
}

#[cfg(test)]
mod sphere_pos {
    use crate::kinematics::{Position, SpherePos};
    use std::f64::consts::{PI, SQRT_2};

    #[test]
    fn to_position() {
        let pos = SpherePos {
            azmut: 1.,
            polar: 1.,
            f_dst: 0.,
            dst: 0.,
        };

        let actual = pos.to_position();
        let expected = Position::new(0., 0., 0.);

        assert_eq!(actual, expected);

        let pos = SpherePos {
            azmut: PI / 4.,
            polar: 0.,
            f_dst: SQRT_2,
            dst: SQRT_2,
        };

        let actual = pos.to_position();
        let expected = Position::new(1., 0., 1.);

        assert_eq!(actual, expected);
    }
}
