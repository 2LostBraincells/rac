use std::{thread::sleep, time::Duration};

use crate::communication::Connection;
use crate::robot::*;

mod communication;
mod kinematics;
mod robot;


fn main() {
    // open serial connection

    let mut robot = Robot {
        angles: Arm {
            base: Angle(30.),
            shoulder: Angle(90.),
            elbow: Angle(110.),
            claw: Angle(100.),
        },
        ..Robot::new(10., 10.,)
    };

    robot.connection.connect().expect("Could not connect");

    sleep(Duration::from_secs(2));

    robot.update().expect("Failed to update servos");
}
