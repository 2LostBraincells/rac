use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use gilrs::Gilrs;
use kinematics::Joint;

use crate::robot::*;

mod communication;
mod kinematics;
mod logging;
mod robot;

fn main() {
    // open serial connection

    let mut robot = Robot {
        angles: Arm {
            base: Joint::default(),
            shoulder: Joint::default(),
            elbow: Joint::default(),
            claw: Joint::default(),
        },
        ..Robot::new(100., 100.)
    };

    let mut gilrs = Gilrs::new().expect("Could not setup gilrs");
    let mut gamepad = None;
    let mut prev = Instant::now();

    robot.connection.connect().expect("Could not connect");

    sleep(Duration::from_secs(2));

    loop {
        if let Some(event) = gilrs.next_event() {
            gamepad = Some(event.id);
        }

        if let Some(gamepad_id) = gamepad {
            let gamepad = gilrs.gamepad(gamepad_id);
            let delta = dbg!(Instant::now() - prev);
            prev = Instant::now();
            match robot.update(&gamepad, delta.as_secs_f64()) {
                Ok(it) => it,
                Err(err) => {
                    dbg!(err);
                }
            };
        }

        clearscreen::clear().unwrap();
        println!("{:?}", robot.position);
    }
}
