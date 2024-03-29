use crate::{
    arm::Arm,
    kinematics::{
        joints::{DirectDrive, DirectDriveOffset, DoubleLinkage, Joint},
        position::CordinateVec,
    },
};
use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use gilrs::Gilrs;

use crate::robot::*;

mod communication;
mod kinematics;
mod logging;
mod robot;

fn main() {
    let mut robot = Robot {
        acceleration: 100.,
        max_velocity: CordinateVec::new(10., 10., 10.),
        upper_arm: 100.,
        lower_arm: 100.,
        arm: Arm {
            base: Joint::new(0., 180., Box::new(DirectDriveOffset { offset: 90. })),
            claw: Joint::new(0., 180., Box::new(DirectDrive::new())),
            shoulder: Joint::new(
                0.,
                180.,
                Box::new(DoubleLinkage::new(1., 10., 10., 1., 10., 20.)),
            ),
            elbow: Joint::new(
                0.,
                180.,
                Box::new(DoubleLinkage::new(1., 10., 10., 1., 10., 20.)),
            ),
        },
        position: CordinateVec::new(0., 0., 0.),
        velocity: CordinateVec::new(0., 0., 0.),
        target_position: Some(CordinateVec::new(50., 50., 50.)),
        target_velocity: CordinateVec::new(0., 0., 0.),
        claw_open: false,
        connection: communication::Connection::new("/dev/ttyACM0", 115_200),
    };

    let mut gilrs = Gilrs::new().expect("Could not setup gilrs");
    // open serial connection
    robot.connection.connect().expect("Could not connect");

    sleep(Duration::from_secs(2));

    let mut prev = Instant::now();

    loop {
        let delta: Duration = dbg!(Instant::now() - prev);
        prev = Instant::now();

        clearscreen::clear().unwrap();

        if let Some(event) = gilrs.next_event() {
            let gamepad = gilrs.gamepad(event.id);
            robot.update_gamepad(&gamepad);
        }

        let _ = robot.update(delta.as_secs_f64());
        println!("pos: {:?}", robot.position);
        println!("trg: {:?}", robot.target_position);
        println!("vel: {:?}", robot.velocity);
        println!("tve: {:?}", robot.target_velocity);
        println!("ang: {:#?}", robot.arm);
    }
}
