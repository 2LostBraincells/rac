use std::{collections::VecDeque, time::Duration};

use serialport::{Error, SerialPort};

/// port for arduino communication
const ARDUINO_PORT: &str = "/dev/ttyACM0";
const ARDUINO_BAUD: u32 = 38400;

/// Logging level all levels include the ones before
/// 0 = no logs
/// 1 = errors
/// 2 = warnings
/// 3 = info
/// 4 = debug
/// 5 = verbose
const LOG_LEVEL: u8 = 5;

#[derive(Debug)]
pub struct Connection {
    pub port: Option<Box<dyn SerialPort>>,

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
            port: None,
            read_buf: Vec::new(),
            msg_buf: VecDeque::new(),
            no_connect: false,
        }
    }
}

impl Connection {
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
        self.port = Some(
            serialport::new(ARDUINO_PORT, ARDUINO_BAUD)
                .timeout(Duration::from_secs(1))
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
        let port = match &mut self.port {
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
    pub fn write(&mut self, data: &[u8]) -> Result<(), ComError> {
        let mut message: Vec<u8> = Vec::with_capacity(data.len() + 2);

        message.push(b'\r');
        for byte in data.into_iter() {
            message.push(*byte);
        }

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
    pub fn read(&mut self) -> Result<Option<Message>, ComError> {
        // do nothing if no_connect is true
        if self.no_connect {
            debug("Not reading due to no_connect flag");
            return Ok(None);
        }

        let port: &mut Box<dyn SerialPort> = match &mut self.port {
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
                b'\r' => self.read_buf.clear(),
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

fn info(message: &'static str) {
    if LOG_LEVEL < 3 {
        return;
    }

    println!("INFO: {}", message);
}

fn debug(message: &'static str) {
    if LOG_LEVEL < 4 {
        return;
    }

    println!("DEBG: {}", message);
}

fn verbose(message: &'static str) {
    if LOG_LEVEL < 5 {
        return;
    }

    println!("VERB: {}", message);
}
