use std::{io::Read, thread::sleep, time::Duration};

// servo angles
const MAX_ANGLE: f32 = 180.0;
const MIN_ANGLE: f32 = 0.0;

// microseconds for arduino
const MAX_SERVO: u16 = 2400;
const MIN_SERVO: u16 = 250;

// cummunication
const PREFIX: &u8 = &b'\r';
const SUFFIX: &u8 = &b'\n';

mod robot {
    use crate::{MAX_ANGLE, MAX_SERVO, MIN_ANGLE, MIN_SERVO};

    /// Defines a servo angle, but with more functions on it
    #[derive(Debug, Copy, Clone)]
    pub struct Angle(pub f32);

    /// Very specific names for servos
    #[derive(Debug, Copy, Clone)]
    pub struct Arm {
        pub base: Angle,
        pub shoulder: Angle,
        pub elbow: Angle,
        pub claw: Angle,
    }

    /// angle to servo fucky microseccond
    impl Into<u16> for Angle {
        fn into(self) -> u16 {
            let factor = (self.0 - MIN_ANGLE) / MAX_ANGLE;
            ((MAX_SERVO - MIN_SERVO) as f32 * factor + MIN_SERVO as f32) as u16
        }
    }

    /// he's average alright
    impl Default for Arm {
        fn default() -> Self {
            Self {
                base: Angle(0.),
                shoulder: Angle(0.),
                elbow: Angle(0.),
                claw: Angle(0.),
            }
        }
    }

    /// Arm functions
    impl Arm {
        pub fn to_servos(&self) -> Servos {
            Servos {
                base: self.base.into(),
                shoulder: self.shoulder.into(),
                elbow: self.elbow.into(),
                claw: self.claw.into(),
            }
        }
    }

    /// quirky arm
    #[derive(Debug, Copy, Clone)]
    pub struct Servos {
        pub base: u16,
        pub shoulder: u16,
        pub elbow: u16,
        pub claw: u16,
    }

    impl Servos {
        pub fn to_message(&self) -> Vec<u8> {
            unsafe { std::mem::transmute::<Box<Servos>, &[u8; 8]>(Box::new(*self)) }.to_vec()
        }
    }

    #[cfg(test)]
    mod test {
        use super::*;

        #[test]
        pub fn servos_to_message() {
            let servos = Servos {
                base: 100,
                shoulder: 200,
                elbow: 50,
                claw: 1,
            };

            let actual = servos.to_message();
            let expected: Vec<u8> = vec![100, 0, 200, 0, 50, 0, 1, 0];

            assert_eq!(actual, expected);
        }
    }
}

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

    let robot = robot::Arm {
        base: robot::Angle(0.),
        shoulder: robot::Angle(80.),
        elbow: robot::Angle(100.),
        claw: robot::Angle(110.),
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
