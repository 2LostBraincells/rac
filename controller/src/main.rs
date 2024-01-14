use std::{io::Read, thread::sleep, time::Duration};

mod kinematics;
mod robot;

use crate::robot::*;

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
    let mut connection = serialport::new("/dev/ttyACM0", 38400)
        .open()
        .expect("Failed to open port");

    let robot = Arm {
        base: Angle(0.),
        shoulder: Angle(80.),
        elbow: Angle(100.),
        claw: Angle(110.),
    };

    sleep(Duration::from_secs(2));

    let message = {
        let mut messages = robot.to_servos().to_message();
        messages.push(b'\n');
        let mut message = vec![b'\r'];

        message.append(&mut messages);
        message
    };

    dbg!(&message);

    connection.write(&message).expect("Failed to write");

    // read and message buffers
    let mut buf: Vec<u8> = Vec::new();
    let mut read_buf: Vec<u8> = Vec::new();

    let mut _count = 0;

    // ping timer

    loop {
        // fill read buffer with serial buffer
        read_buf.clear();
        _count += connection.read_to_end(&mut read_buf).unwrap_or(0);

        // parse all read bytes
        for byte in &read_buf {
            match byte {
                PREFIX => buf.clear(), // begining of message
                SUFFIX => {
                    handler(&buf);
                } // end of message
                chr => buf.push(*chr), // in message
            };
        }
    }
}
