use crate::kinematics::triangle::a_from_lengths;
use core::{
    f64::consts::PI,
    ops::{Add, AddAssign, Mul, Sub, SubAssign},
};

/// Defines a 3d position using x, y and z coordinates
#[derive(Debug, Copy, Clone)]
pub struct CordinateVec {
    /// Side to side
    pub x: f64,

    /// Forward and backward
    pub y: f64,

    /// Up and down
    pub z: f64,
}

/// Defines a 3d position using y, z and azimuth coordinates
#[derive(Debug, Copy, Clone)]
pub struct MixedVec {
    pub y: f64,
    pub z: f64,
    pub azimuth: f64,
}

/// Defines a position using spherical coordinates
#[derive(Debug, Copy, Clone)]
pub struct SphereVec {
    /// Horizontal angle from origin to position from the x axis
    pub azimuth: f64,

    /// Vertical angle from origin to position from the z axis
    pub polar: f64,

    /// Distance from origin
    pub distance: f64,

    /// Distance from origin on flat ground
    pub flat_distance: f64,
}

impl CordinateVec {
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
    /// let mut position = Position::new(12., 9., -50.);
    /// position.clamp(-5., 10.);
    ///
    /// assert_eq!(position, Position::new(12., 9., -5.));
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
    ///
    /// Err(()) - No valid solution was found
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::Position;
    ///
    /// let mut position = Position::new(1., 1., 1.);
    ///
    /// let arm = position.inverse_kinematics(10,10);
    /// ```
    pub fn inverse_kinematics(
        &self,
        upper_arm: f64,
        lower_arm: f64,
    ) -> Result<(f64, f64, f64), ()> {
        // spherical representation of the position
        let spos = &self.to_sphere();

        // base angle
        let base = spos.azimuth.to_degrees() + 90.;

        // elbow angle
        let elbow = a_from_lengths(upper_arm, lower_arm, spos.distance).to_degrees();

        // shoulder angle
        let shoulder = {
            // arctan(f_dst / y)
            let a = (spos.flat_distance / self.z).atan();
            let b = a_from_lengths(spos.distance, lower_arm, upper_arm);

            if a + b > PI / 2. {
                PI - a - b
            } else {
                a + b
            }
        }
        .to_degrees();

        // make sure all the angles are valid
        if shoulder.is_nan() || base.is_nan() || elbow.is_nan() {
            return Err(());
        }

        Ok((base, shoulder, elbow))
    }

    /// Calculates the distance from origin on flat ground
    ///
    /// since this value is only on the x,z plane the z axis is irrelevant
    ///
    /// sqrt(X^2 + Z^2)
    pub fn f_dst(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    /// Calculates the distance from origin
    ///
    /// sqrt(X^2 + Y^2 + Z^2)
    pub fn dst(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    /// Calculates the horizontal angle from origin to position from the x axis
    ///
    /// arctan(x / z)
    pub fn azimuth(&self) -> f64 {
        match self.x.signum() as i8 {
            1 => (self.y / self.x).atan(),
            -1 => (self.y / self.x).atan() + PI,
            _ => 0.,
        }
    }

    /// Calculates the vertical angle from origin to position from the z axis
    ///
    /// arctan(f_dst / z)
    pub fn polar(&self) -> f64 {
        match self.z.signum() as i8 {
            1 => (self.f_dst() / self.z).atan(),
            -1 => (self.f_dst() / self.z).atan() + PI,
            _ => 0.,
        }
    }

    /// Converts a 3d position to spherical coordinates
    ///
    /// Due to floating point errors the position might
    /// not be exactly right but it is usualy close enough
    ///
    /// # Examples
    ///
    /// ```rust
    /// use std::f64::consts::{PI, SQRT_2};
    /// use robot::kinematics::Position;
    ///
    /// let position = Position::new(1., 1., 0.);
    ///
    /// let sphere = position.to_sphere();
    ///
    /// assert_eq!(sphere.azimuth, PI/2);
    /// assert_eq!(sphere.polar, 0.);
    /// assert_eq!(sphere.dst, SQRT_2);
    /// assert_eq!(sphere.f_dst, SQRT_2);
    /// ```
    pub fn to_sphere(&self) -> SphereVec {
        SphereVec {
            azimuth: self.azimuth(),
            polar: self.polar(),
            distance: self.dst(),
            flat_distance: self.f_dst(),
        }
    }
}

impl SphereVec {
    /// Creates a new position
    ///
    /// # Arguments
    /// * `azimuth` - Horizontal angle from origin to position from the x axis
    /// * `polar` - Vertical angle from origin to position from the z axis
    /// * `dst` - Distance from origin
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::SphereVec;
    /// let pos = SphereVec::new(0., 0., 0.);
    /// ```
    #[allow(unused)]
    pub fn new(azimuth: f64, polar: f64, dst: f64) -> Self {
        Self {
            azimuth,
            polar,
            distance: dst,
            flat_distance: dst * polar.sin(),
        }
    }

    /// updates the distance from origin
    ///
    /// # Arguments
    /// * `dst` - The new distance from origin
    ///
    /// # Examples
    /// ```rust
    /// use robot::kinematics::SphereVec;
    /// let mut pos = SphereVec::new(0., 0., 0.);
    ///
    /// pos.update_dst(10.);
    ///
    /// assert_eq!(pos.dst, 10.);
    /// ```
    pub fn update_dst(&mut self, dst: f64) {
        self.distance = dst;
        self.flat_distance = dst * self.polar.sin();
    }

    /// Converts spherical coordinates to a 3d position
    ///
    /// due to floating point errors the position might
    /// not be exactly the same but it is usualy close enough
    ///
    /// # Examples
    /// ```rust
    /// use std::f64::consts::{PI, SQRT_2};
    /// use robot::kinematics::SphereVec;
    /// let pos = SphereVec::new(PI/2, 0., SQRT_2);
    ///
    /// let position = pos.to_position();
    ///
    /// assert_eq!(position.x.round(), 1.);
    /// assert_eq!(position.y.round(), 1.);
    /// assert_eq!(position.z.round(), 0.);
    /// ```
    pub fn to_position(&self) -> CordinateVec {
        CordinateVec {
            x: self.flat_distance * self.azimuth.cos(),
            y: self.flat_distance * self.azimuth.sin(),
            z: self.distance * self.polar.cos(),
        }
    }
}

impl MixedVec {
    pub fn to_position(&self) -> CordinateVec {
        CordinateVec {
            x: self.azimuth.cos() * self.y,
            y: self.azimuth.sin() * self.y,
            z: self.z,
        }
    }

    pub fn to_sphere(&self) -> SphereVec {
        SphereVec {
            azimuth: self.azimuth,
            polar: (self.z/self.y).atan(),
            distance: (self.y.powi(2) + self.z.powi(2)).sqrt(),
            flat_distance: self.y,
        }
    }
}

impl Into<CordinateVec> for SphereVec {
    /// Same as [`SphereVec::to_position`]
    fn into(self) -> CordinateVec {
        self.to_position()
    }
}

impl Into<SphereVec> for CordinateVec {
    /// Same as [`CordinateVec::to_sphere`]
    fn into(self) -> SphereVec {
        self.to_sphere()
    }
}

impl Default for CordinateVec {
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }
}

impl Sub for CordinateVec {
    type Output = CordinateVec;

    fn sub(self, rhs: CordinateVec) -> Self::Output {
        CordinateVec {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Add for CordinateVec {
    type Output = CordinateVec;

    fn add(self, rhs: CordinateVec) -> Self::Output {
        CordinateVec {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for CordinateVec {
    fn add_assign(&mut self, rhs: CordinateVec) {
        *self = CordinateVec {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl SubAssign for CordinateVec {
    fn sub_assign(&mut self, rhs: Self) {
        *self = Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Eq for CordinateVec {}
impl PartialEq for CordinateVec {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }

    fn ne(&self, other: &Self) -> bool {
        self.x != other.x || self.y != other.y || self.z != other.z
    }
}

impl Mul<f64> for CordinateVec {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Mul<CordinateVec> for CordinateVec {
    type Output = Self;

    fn mul(self, rhs: CordinateVec) -> Self::Output {
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

    use crate::kinematics::position::CordinateVec;


    #[test]
    fn properties() {
        let position = CordinateVec::new(3., 4., 5.);
        let square = (3*3 + 4*4 + 5*5) as f64;

        assert_eq!(position.x, 3.);
        assert_eq!(position.y, 4.);
        assert_eq!(position.z, 5.);
        assert_eq!(position.f_dst(), 5.);
        assert_eq!(position.dst(), square.sqrt());
        assert_eq!(position.polar(), 45f64.to_radians());
        assert_eq!(position.azimuth().to_degrees().round(), 53.);

        let position = CordinateVec::new(-3., 4., -5.);

        assert_eq!(position.x, -3.);
        assert_eq!(position.y, 4.);
        assert_eq!(position.z, -5.);
        assert_eq!(position.f_dst(), 5.);
        assert_eq!(position.dst(), square.sqrt());
        assert_eq!(position.polar(), 135f64.to_radians());
        assert_eq!(position.azimuth().to_degrees().round(), 180.-53.);
    }

    #[test]
    fn to_sphere() {

        let expected = CordinateVec::new(-1., -1., -1.);
        let actual = expected.to_sphere().to_position();

        assert_eq!(expected.x, actual.x.round());
        assert_eq!(expected.y, actual.y.round());
        assert_eq!(expected.z, actual.z.round());

        let expected = CordinateVec::new(-1., 1., 1.);
        let actual = expected.to_sphere().to_position();

        assert_eq!(expected.x, actual.x.round());
        assert_eq!(expected.y, actual.y.round());
        assert_eq!(expected.z, actual.z.round());
    }

    #[test]
    fn inverse_kinematics() {
        let position = CordinateVec::new(SQRT_2, 0., 0.);

        let actual = position.inverse_kinematics(1., 1.).unwrap();

        assert_eq!((actual.0 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 90.);
        assert_eq!((actual.1 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 45.);
        assert_eq!((actual.2 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 90.);

        let position = CordinateVec::new(0., 0., 0.);

        let actual = position.inverse_kinematics(0., 0.);

        assert!(actual.is_err());
    }

    #[test]
    fn addition() {
        let a = CordinateVec::new(1., 2., 3.);
        let mut b = CordinateVec::new(2., 2., 2.);
        let c = CordinateVec::new(3., 2., 1.);

        assert_eq!(a + b, CordinateVec::new(3., 4., 5.));
        assert_eq!(a + c, CordinateVec::new(4., 4., 4.));

        b += c;
        assert_eq!(b, CordinateVec::new(5., 4., 3.));
    }

    #[test]
    fn subtraction() {
        let a = CordinateVec::new(1., 2., 3.);
        let mut b = CordinateVec::new(2., 2., 2.);
        let c = CordinateVec::new(3., 2., 1.);

        assert_eq!(a - b, CordinateVec::new(-1., 0., 1.));
        assert_eq!(a - c, CordinateVec::new(-2., 0., 2.));

        b -= c;

        assert_eq!(b, CordinateVec::new(-1., 0., 1.));
    }
}

#[cfg(test)]
mod sphere_pos {
    use crate::kinematics::position::{CordinateVec, SphereVec};
    use std::f64::consts::PI;

    #[test]
    fn to_position() {
        let pos = SphereVec {
            azimuth: 1.,
            polar: 1.,
            flat_distance: 0.,
            distance: 0.,
        };

        let actual = pos.to_position();
        let expected = CordinateVec::new(0., 0., 0.);

        assert_eq!(actual, expected);

        let pos = SphereVec::new(PI / 4., PI / 2., 2f64.sqrt());
        dbg!(pos);
        let actual = pos.to_position();

        assert_eq!(actual.x.round(), 1.);
        assert_eq!(actual.y.round(), 1.);
        assert_eq!(actual.z.round(), 0.);
    }
}
