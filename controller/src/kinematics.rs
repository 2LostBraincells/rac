use std::{
    f64::consts::PI,
    ops::{Add, AddAssign, Mul, Sub, SubAssign},
};

use std::fmt::Debug;

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

/// A arm joint with limits and functions for calculating pivot angle
#[derive(Debug)]
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

impl Position {
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
            let a = (spos.f_dst / self.y).atan();
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

    /// Converts the 3d cordinates into spherical cordinates
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
    pub fn cube_clamp(&mut self, min: f64, max: f64) {
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
        let angle = (self.connection_radial_offset / self.connection_linear_offset).atan();
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
            (self.controll_pivot_horizontal_offset / self.controll_pivot_vertical_offset).atan();
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

        let inner_target_angle = PI - target - connection.0;

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

            x + y
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

pub mod triangle {
    /// The angle for the corner between a and b in radians
    ///
    /// x = -c^2 + a^2 + b^2
    /// y = 2ab
    ///
    /// arccos(x/y)
    pub fn a_from_lengths(a: f64, b: f64, c: f64) -> f64 {
        let x = -(c * c) + a * a + b * b;
        let y = 2. * a * b;
        (x / y).acos()
    }

    /// The length of the side opposite of the angle
    ///
    /// x = a - cos(angle) * a
    /// y = sin(angle) * b
    ///
    /// sqrt(x^2 + y^2)
    pub fn length_from_two_lengths_and_angle(angle: f64, a: f64, b: f64) -> f64 {
        let x = a - angle.cos() * a;
        let y = angle.sin() * b;

        (x * x + y * y).sqrt()
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

        #[test]
        fn length_from_two_lengths_and_angle() {
            assert_eq!(
                triangle::length_from_two_lengths_and_angle(90f64.to_radians(), 3., 4.).round(),
                5.
            );
            assert_eq!(
                triangle::length_from_two_lengths_and_angle(60f64.to_radians(), 2., 2.).round(),
                2.
            );
        }
    }
}

impl Debug for MotionField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotionField")
            .field("moption", &self.get_pivot_angle(0.))
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

#[cfg(test)]
mod position {

    use std::f64::consts::SQRT_2;

    use crate::kinematics::Position;

    #[test]
    fn inverse_kinematics() {
        let mut position = Position::new(SQRT_2, 0., 0.);

        let actual = position.inverse_kinematics(1., 1.).unwrap();

        assert_eq!((actual.0 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 180.);
        assert_eq!((actual.1 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 45.);
        assert_eq!((actual.2 * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 90.);

        let mut position = Position::new(0., 0., 0.);

        let actual = position.inverse_kinematics(0., 0.);

        assert!(actual.is_err());
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

        assert_eq!((actual.x * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 1.);
        assert_eq!((actual.y * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 0.);
        assert_eq!((actual.z * 10.0f64.powi(4)).round() / 10.0f64.powi(4), 1.);
    }
}
