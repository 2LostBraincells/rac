use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

use crate::logging::*;
use serialport::{Error, SerialPort};

/// Indicates a new message
const PREFIX: u8 = b'\r';

#[derive(Debug)]
pub struct Connection {
    pub port: &'static str,
    pub baud: u32,

    /// Serial connection to arduino
    pub con: Option<Box<dyn SerialPort>>,

    /// Instant of last write
    pub last_write: Instant,

    /// buffer for reading messages into
    pub read_buf: Vec<u8>,

    /// Bufer of messages that haven't been handled yet
    pub msg_buf: VecDeque<Message>,

    /// If this value is true any operation that will require the arduino to be
    /// connected will be ignored. Usefull for debugging and testing
    pub no_connect: bool,
}

#[derive(Debug)]
pub enum ComError {
    NotConnected,
    Error(std::io::Error),
}

pub type Message = Vec<u8>;

impl Default for Connection {
    fn default() -> Self {
        Self {
            port: "",
            baud: 0,
            con: None,
            last_write: Instant::now(),
            read_buf: Vec::new(),
            msg_buf: VecDeque::new(),
            no_connect: false,
        }
    }
}

impl Connection {
    pub fn new(port: &'static str, baud: u32) -> Self {
        Self {
            port,
            baud,
            con: None,
            last_write: Instant::now(),
            read_buf: Vec::new(),
            msg_buf: VecDeque::new(),
            no_connect: true,
        }
    }

    /// Connect to arduino
    ///
    /// Attempts to open a serial connection to the arduino
    ///
    /// # Returns
    /// `Ok` if a connection gets established `Err` otherwise
    pub fn connect(&mut self) -> Result<(), Error> {
        // do nothing if no_connect is true
        if self.no_connect {
            debug("Not connecting due to no_connect flag");
            return Ok(());
        }

        // connect arduino
        self.con = Some(
            serialport::new(self.port, self.baud)
                .timeout(Duration::from_millis(100))
                .open()?,
        );
        Ok(())
    }

    /// Write raw bytes with no preprocessing
    ///
    /// For the communication to work properly it is required to add a `\r` before
    /// every emssage and `\n` after
    ///
    /// # Arguments
    /// * `data` - Data to write
    ///
    /// # Returns
    /// A `Ok` Result if the write was successfull otherwise a `ComError`
    pub fn write_raw(&mut self, data: &[u8]) -> Result<(), ComError> {
        if LOG_LEVEL >= 5 {
            print!("> ");
            for byte in data {
                print!("{} ", *byte);
            }
            println!();
        }

        // do nothing if no_connect is true
        if self.no_connect {
            debug("Not writing due to no_connect flag");
            return Ok(());
        }

        // Make sure arduino is connected
        let port = match &mut self.con {
            None => return Err(ComError::NotConnected),
            Some(port) => port,
        };

        match port.write(data) {
            Ok(_) => Ok(()),
            Err(err) => Err(ComError::Error(err)),
        }
    }

    /// Writes the given data to the ardunio
    ///
    /// # Arguments
    /// * `data` - data to write
    ///
    /// # Returns
    /// `Ok` if the data was transmitted successfully `Err` otherwise
    pub fn write(&mut self, data: &[u8], allow_drooped: bool) -> Result<(), ComError> {
        let mut message: Vec<u8> = Vec::with_capacity(data.len() + 2);

        message.push(b'\r');
        for byte in data.into_iter() {
            message.push(*byte);
        }

        if !allow_drooped {
            unreachable!("im to lazy to make it work otherwise");
        }

        // if (Instant::now() - self.last_write) > Duration::from_millis(10) {
        //     self.last_write = Instant::now();
        // } else {
        //     println!("Ratelimiting ({}s left)", (Instant::now() - self.last_write).as_secs_f32());
        //     Err(ComError::Ratelimit)
        // }
        self.write_raw(message.as_slice())
    }

    /// Read from serial buffer and return if a valid message was recived
    ///
    /// A valid message is defined as a `\r` with 8 bytes after it
    ///
    /// # Returns
    /// `Ok` If no error occured while reading
    /// `Ok(None)` If no message was recived
    /// `Ok(Some(Message))` where the `Message` contains the data
    #[allow(dead_code)]
    pub fn read(&mut self) -> Result<Option<Message>, ComError> {
        // do nothing if no_connect is true
        if self.no_connect {
            debug("Not reading due to no_connect flag");
            return Ok(None);
        }

        let port: &mut Box<dyn SerialPort> = match &mut self.con {
            None => return Err(ComError::NotConnected),
            Some(port) => port,
        };

        let mut buf: Vec<u8> = Vec::new();
        match port.read_to_end(&mut buf) {
            Ok(_) => {}
            Err(err) => return Err(ComError::Error(err)),
        }

        for byte in buf {
            match byte {
                PREFIX => self.read_buf.clear(),
                byte => {
                    if self.read_buf.len() == 8 {
                        self.msg_buf.push_back(self.read_buf.clone());
                        self.read_buf.clear()
                    }
                    self.read_buf.push(byte)
                }
            }
        }

        Ok(self.msg_buf.pop_front())
    }
}
