
use crate::robot::*;
use gilrs::{Axis, Button, Event, Gamepad, Gilrs};

const MAX_SPEED: f32 = 0.25;
const DEAD_ZONE: f32 = 0.1;

use self::geometry::triangle::a_from_lengths;

pub struct Position {
    /// Width
    x: f32,

    /// Height
    y: f32,

    /// Depth
    z: f32,
}

pub struct SpherePos {
    /// Azmut angle
    pub azmut: f32,

    /// Polar angle
    pub polar: f32,

    /// 3d distance from origin
    pub dst: f32,

    /// Distance from origin on flat ground
    pub f_dst: f32,
}

pub struct Robot {
    pub position: Position,
    pub angles: Arm,
    pub upper_arm: f32,
    pub lower_arm: f32,
    pub square_sum: f32,
    pub claw_open: bool,
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

impl Robot {
    pub fn new(lower_arm: f32, upper_arm: f32) -> Robot {
        Robot {
            position: Position::default(),
            angles: Arm::default(),
            upper_arm,
            lower_arm,
            square_sum: upper_arm * upper_arm + lower_arm * lower_arm,
            claw_open: false,
        }
    }

    pub fn inverse_kinematics(&mut self) {
        let pos = &self.position;
        let spos = &self.position.to_sphere();

        let beta = a_from_lengths(self.lower_arm, self.upper_arm, spos.dst);

        let alpha = {
            let x = (spos.f_dst / pos.y).atan();
            let y = a_from_lengths(spos.dst, self.lower_arm, self.upper_arm);

            x + y
        };

        self.angles.base = Angle(spos.azmut.to_degrees());
        self.angles.shoulder = Angle(alpha.to_degrees());
        self.angles.elbow = Angle(beta.to_degrees());
    }

    pub fn update_position(&mut self, gamepad: Gamepad) {
        let right_stick_axis_x = gamepad.value(Axis::RightStickX);
        let right_stick_axis_y = gamepad.value(Axis::RightStickY);
        let left_stick_axis_x = gamepad.value(Axis::LeftStickX);
        let left_stick_axis_y = gamepad.value(Axis::LeftStickY);

        if right_stick_axis_x.abs() > DEAD_ZONE {
            self.position.z += MAX_SPEED * right_stick_axis_x;
        }
        if right_stick_axis_y.abs() > DEAD_ZONE {
            self.position.x += MAX_SPEED * right_stick_axis_y;
        }
        if left_stick_axis_y.abs() > DEAD_ZONE {
            self.position.y += MAX_SPEED * left_stick_axis_x;
        }
        if gamepad.is_pressed(Button::LeftTrigger2) {
            self.claw_open = !self.claw_open;
        }
    }
}

#[cfg(test)]
mod test {
    use std::f32::consts::SQRT_2;

    use crate::kinematics::Robot;

    #[test]
    fn angle_test() {
        let mut robot = Robot::new(1., 1.);
        robot.position.x = 0.;
        robot.position.z = SQRT_2;
        robot.position.y = 0.;

        robot.inverse_kinematics();

        assert_eq!(robot.angles.base.inner(), &0.);
        assert_eq!(robot.angles.shoulder.inner(), &135.);
        assert_eq!(robot.angles.elbow.inner(), &90.);
    }
}

mod geometry {
    pub mod triangle {
        /// The angles for the corner between a and b in radians
        ///
        /// x = -c^2 + a^2 + b^2
        /// y = 2ab
        /// arccos(x/y)
        pub fn a_from_lengths(a: f32, b: f32, c: f32) -> f32 {
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
