use serial2::*;
use robust_arduino_serial::*;
use std::{time::Duration, alloc::System, process::{exit, self}};
use gilrs::{Gilrs, Button, Event};

fn main() {
    let mut gilrs = Gilrs::new().unwrap();   
    let serial_port = "/dev/cu.usbmodem1101";
    
    let mut serial = SerialPort::open(serial_port, 115200);
        
    loop {
        
    }
}
