use crate::kinematics::triangle::a_from_lengths;
use core::{
    f64::consts::PI,
    ops::{Mul, SubAssign, Add, AddAssign, Sub},
};

/// Defines a position in 3d space
#[derive(Debug, Copy, Clone)]
pub struct Vec3D {
    /// Side to side
    pub x: f64,

    /// Forward and backward
    pub y: f64,

    /// Up and down
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

impl Vec3D {
    /// Creates a new Position
    /// # Arguments
    /// * `x` - Side to side position
    /// * `y` - Forward and backward position
    /// * `z` - Up and down position
    #[allow(unused)]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
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
    pub fn cube_clamp(&mut self, min: f64, max: f64) {
        self.x = self.x.clamp(min, max);
        self.y = self.y.clamp(min, max);
        self.z = self.z.clamp(min, max);
    }

    /// Calculates the angles for the arm to reach a position
    ///
    /// # Arguments
    /// * `upper_arm` - The length of the upper Arm
    /// * `lower_arm` - The length of the lower Arm
    ///
    /// # Returns
    /// Ok(Arm) - The angles for the arm to reach the position
    /// Err(()) - No valid solution was found
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::Position;
    ///
    /// let mut position = Position::new(1., 1., 1.);
    ///
    /// let arm = robot.inverse_kinematics(10,10);
    /// ```
    pub fn inverse_kinematics(
        &mut self,
        upper_arm: f64,
        lower_arm: f64,
    ) -> Result<(f64, f64, f64), ()> {
        let spos = &self.to_sphere();

        let base = spos.azmut.to_degrees() + 90.;

        if base.is_nan() {
            return Err(());
        }

        let elbow = a_from_lengths(upper_arm, lower_arm, spos.dst).to_degrees();

        if elbow.is_nan() {
            return Err(());
        }

        let shoulder = {
            // arctan(f_dst / y)
            let a = (spos.f_dst / self.z).atan();
            let b = a_from_lengths(spos.dst, lower_arm, upper_arm);

            if a + b > PI / 2. {
                PI - a - b
            } else {
                a + b
            }
        }
        .to_degrees();

        if shoulder.is_nan() {
            return Err(());
        }

        Ok((base, shoulder, elbow))
    }
}

impl Default for Vec3D {
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }
}

impl Vec3D {
    /// sqrt(X^2 + Z^2)
    pub fn f_dst(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    /// sqrt(X^2 + Y^2 + Z^2)
    pub fn dst(&self) -> f64 {
        (self.x.powi(2) + self.z.powi(2) + self.z.powi(2)).sqrt()
    }

    /// arctan(x / z)
    pub fn azmut(&self) -> f64 {
        match self.z.signum() as i8 {
            1 => (self.y / self.x).atan(),
            -1 => (self.y / self.x).atan() + PI,
            _ => 0.,
        }
    }

    /// arctan(f_dst / y)
    pub fn polar(&self) -> f64 {
        match self.y.signum() as i8 {
            1 => (self.z / self.f_dst()).atan(),
            -1 => (self.z / self.f_dst()).atan(),
            _ => 0.,
        }
    }

    pub fn to_sphere(&self) -> SpherePos {
        SpherePos {
            azmut: self.azmut(),
            polar: self.polar(),
            dst: self.dst(),
            f_dst: self.f_dst(),
        }
    }
}

impl SpherePos {
    pub fn update_dst(&mut self, dst: f64) {
        self.dst = dst;
        self.f_dst = dst * self.polar.cos();
    }
    pub fn to_position(&self) -> Vec3D {
        Vec3D {
            x: self.f_dst * self.azmut.cos(),
            y: self.dst * self.polar.sin(),
            z: self.f_dst * self.azmut.sin(),
        }
    }
}

impl Sub for Vec3D {
    type Output = Vec3D;

    fn sub(self, rhs: Vec3D) -> Self::Output {
        Vec3D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Add for Vec3D {
    type Output = Vec3D;

    fn add(self, rhs: Vec3D) -> Self::Output {
        Vec3D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Vec3D {
    fn add_assign(&mut self, rhs: Vec3D) {
        *self = Vec3D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl SubAssign for Vec3D {
    fn sub_assign(&mut self, rhs: Self) {
        *self = Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Eq for Vec3D {}
impl PartialEq for Vec3D {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }

    fn ne(&self, other: &Self) -> bool {
        self.x != other.x || self.y != other.y || self.z != other.z
    }
}

impl Mul<f64> for Vec3D {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Mul<Vec3D> for Vec3D {
    type Output = Self;

    fn mul(self, rhs: Vec3D) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

#[cfg(test)]
mod position {

    use std::f64::consts::SQRT_2;

    use crate::kinematics::position::Vec3D;

    #[test]
    fn to_sphere() {
        let expected = Vec3D::new(-1., -1., -1.);
        let actual = expected.to_sphere().to_position();

        assert_eq!(expected.x, actual.x.round());
        assert_eq!(expected.y, actual.y.round());
        assert_eq!(expected.z, actual.z.round());
    }

    #[test]
    fn inverse_kinematics() {
        let mut position = Vec3D::new(SQRT_2, 0., 0.);

        let actual = position.inverse_kinematics(1., 1.).unwrap();

        assert_eq!((actual.0 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 90.);
        assert_eq!((actual.1 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 45.);
        assert_eq!((actual.2 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 90.);

        let mut position = Vec3D::new(0., 0., 0.);

        let actual = position.inverse_kinematics(0., 0.);

        assert!(actual.is_err());
    }

    #[test]
    fn addition() {
        let a = Vec3D::new(1., 2., 3.);
        let mut b = Vec3D::new(2., 2., 2.);
        let c = Vec3D::new(3., 2., 1.);

        assert_eq!(a + b, Vec3D::new(3., 4., 5.));
        assert_eq!(a + c, Vec3D::new(4., 4., 4.));

        b += c;
        assert_eq!(b, Vec3D::new(5., 4., 3.));
    }

    #[test]
    fn subtraction() {
        let a = Vec3D::new(1., 2., 3.);
        let mut b = Vec3D::new(2., 2., 2.);
        let c = Vec3D::new(3., 2., 1.);

        assert_eq!(a - b, Vec3D::new(-1., 0., 1.));
        assert_eq!(a - c, Vec3D::new(-2., 0., 2.));

        b -= c;

        assert_eq!(b, Vec3D::new(-1., 0., 1.));
    }
}

#[cfg(test)]
mod sphere_pos {
    use crate::kinematics::position::{SpherePos, Vec3D};
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
        let expected = Vec3D::new(0., 0., 0.);

        assert_eq!(actual, expected);

        let pos = SpherePos {
            azmut: PI / 4.,
            polar: 0.,
            f_dst: SQRT_2,
            dst: SQRT_2,
        };

        let actual = pos.to_position();

        assert_eq!((actual.x * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 1.);
        assert_eq!((actual.y * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 0.);
        assert_eq!((actual.z * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 1.);
    }
}
