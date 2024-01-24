use crate::{
    arm::Arm,
    kinematics::joints::{DirectDrive, DoubleLinkage, Joint},
};
use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use gilrs::Gilrs;
use kinematics::position::CordinateVec;
use robot::movement::full::Full;

use crate::robot::*;

mod communication;
mod kinematics;
mod logging;
mod robot;

fn main() {
    let mut robot = Robot {
        upper_arm: 275.,
        lower_arm: 279.,
        arm: Arm {
            base: Joint::new(0., 180., Box::new(DirectDrive::new())),
            claw: Joint::new(0., 180., Box::new(DirectDrive::new())),
            shoulder: Joint::new(
                0.,
                180.,
                Box::new({
                    DoubleLinkage {
                        connection_radial_offset: 73.,
                        connection_linear_offset: 24.,
                        controll_pivot_horizontal_offset: 63.,
                        controll_pivot_vertical_offset: 2.5,
                        controller_pivot_rod_length: 50.,
                        connection_rod_length: 108.,
                    }
                }),
            ),
            elbow: Joint::new(
                0.,
                180.,
                Box::new(DoubleLinkage::new(1., 10., 10., 1., 10., 20.)),
            ),
        },
        claw_open: false,
        connection: communication::Connection::new("/dev/cu.usbmodem1101", 115_200),
        movement: movement::Movement {
            mode: robot::movement::Mode::Full(Full {
                position: CordinateVec::new(0., 300., 300.),
                velocity: CordinateVec::new(0., 0., 0.),
                target_velocity: CordinateVec::new(0., 0., 0.),
                target_position: None,
                max_velocity: CordinateVec::new(100., 100., 100.),
            }),
            acceleration: 10.,
        },
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
        println!("{}", robot.movement.display());
        println!("ang: {:#?}", robot.arm);
    }
}

#[cfg(test)]
mod test {
    use crate::{
        communication,
        kinematics::{
            joints::{DirectDrive, DoubleLinkage, Joint},
            position::CordinateVec,
        },
        robot::{
            self,
            arm::Arm,
            movement::{self, full::Full},
            Robot,
        },
    };

    #[test]
    fn full_test() {
        let mut robot = Robot {
            upper_arm: 275.,
            lower_arm: 279.,
            arm: Arm {
                base: Joint::new(0., 180., Box::new(DirectDrive::new())),
                claw: Joint::new(0., 180., Box::new(DirectDrive::new())),
                shoulder: Joint::new(
                    0.,
                    180.,
                    Box::new({
                        DoubleLinkage {
                            connection_radial_offset: 73.,
                            connection_linear_offset: 24.,
                            controll_pivot_horizontal_offset: 63.,
                            controll_pivot_vertical_offset: 2.5,
                            controller_pivot_rod_length: 50.,
                            connection_rod_length: 108.,
                        }
                    }),
                ),
                elbow: Joint::new(
                    0.,
                    180.,
                    Box::new(DoubleLinkage::new(1., 10., 10., 1., 10., 20.)),
                ),
            },
            claw_open: false,
            connection: communication::Connection::new("/dev/cu.usbmodem1101", 115_200),
            movement: movement::Movement {
                mode: robot::movement::Mode::Full(Full {
                    position: CordinateVec::new(0., 300., 300.),
                    velocity: CordinateVec::new(0., 0., 0.),
                    target_velocity: CordinateVec::new(0., 0., 0.),
                    target_position: None,
                    max_velocity: CordinateVec::new(100., 100., 100.),
                }),
                acceleration: 10.,
            },
        };

        let _ = robot.update(0.1);
        assert_eq!(robot.arm.shoulder.angle.round(), 85.);
        assert_eq!(robot.arm.shoulder.motion.get_pivot_angle(robot.arm.shoulder.angle).round(), 160.);
    }
}
