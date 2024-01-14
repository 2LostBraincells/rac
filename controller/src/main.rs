use std::{thread::sleep, time::Duration};

use crate::communication::Connection;

mod communication;
mod kinematics;
mod robot;


fn main() {
    // open serial connection

    let mut robot = robot::Arm {
        base: robot::Angle(0.),
        shoulder: robot::Angle(80.),
        elbow: robot::Angle(100.),
        claw: robot::Angle(110.),
        connection: Connection::default(),
    };

    robot.connection.connect().expect("Could not connect");

    sleep(Duration::from_secs(2));

    robot.update().expect("Failed to update servos");
}
