use std::{thread::sleep, time::Duration};

use gilrs::Gilrs;

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
        ..Robot::new(100., 100.,)
    };

    let mut gilrs = Gilrs::new().expect("Could not setup gilrs");

    robot.connection.connect().expect("Could not connect");

    sleep(Duration::from_secs(2));

    loop {
        if let Some(event) = gilrs.next_event() {
            let gamepad = gilrs.gamepad(event.id);

            robot.update(&gamepad).expect("Failed to update servos");
        }

        dbg!(robot.position);

        sleep(Duration::from_millis(500));
    }

}
