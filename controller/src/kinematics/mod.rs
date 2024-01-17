pub mod position;
pub mod joints;

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
