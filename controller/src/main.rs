use std::{thread::sleep, time::Duration};

use crate::communication::Connection;

mod communication;
mod kinematics;
mod robot;

// cummunication
const PREFIX: &u8 = &b'\r';
const SUFFIX: &u8 = &b'\n';

fn handler(buffer: &Vec<u8>) {
    print!("< ");
    for byte in buffer.iter() {
        print!("{}", *byte as char);
    }
    println!();
}

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


    // connection.write(&message).expect("Failed to write");

    // // read and message buffers
    // let mut buf: Vec<u8> = Vec::new();
    // let mut read_buf: Vec<u8> = Vec::new();

    // let mut _count = 0;

    // // ping timer

    // loop {
    //     // fill read buffer with serial buffer
    //     read_buf.clear();
    //     _count += connection.read_to_end(&mut read_buf).unwrap_or(0);

    //     // parse all read bytes
    //     for byte in &read_buf {
    //         match byte {
    //             PREFIX => buf.clear(), // begining of message
    //             SUFFIX => {
    //                 handler(&buf);
    //             } // end of message
    //             chr => buf.push(*chr), // in message
    //         };
    //     }
    // }
}
