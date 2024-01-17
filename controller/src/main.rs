use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use gilrs::Gilrs;
use kinematics::{DirectDrive, DirectDriveOffset, DoubleLinkage, Joint, Vec3D};

use crate::robot::*;

mod communication;
mod kinematics;
mod logging;
mod robot;

fn main() {

    let mut robot = Robot {
        acceleration: 10.,
        max_velocity: Vec3D::new(10., 10., 10.),
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
        position: Vec3D::new(0.,0.,0.),
        velocity: Vec3D::new(0.,0.,0.),
        target_position: Some(Vec3D::new(50.,50.,50.)),
        target_velocity: Vec3D::new(0.,0.,0.),
        claw_open: false,
        connection: communication::Connection::new("/dev/ttyACM0", 115_200),
    };

    let mut gilrs = Gilrs::new().expect("Could not setup gilrs");
    let mut gamepad = None;
    let mut prev = Instant::now();

    // open serial connection
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
        println!("pos: {:?}", robot.position);
        println!("trg: {:?}", robot.target_position);
        println!("vel: {:?}", robot.velocity);
        println!("tve: {:?}", robot.target_velocity);
        println!("ang: {:#?}", robot.arm);

    }
}
