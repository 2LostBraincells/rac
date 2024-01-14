use gilrs::{Gilrs, Button, Event};
use crate::robot::*;

pub struct controller {
    pub x_axis: f32,
    pub y_axis: f32,
    pub a_button: bool,
    pub b_button: bool,
    pub x_button: bool,
    pub y_button: bool,
    pub left_trigger: f32,
    pub right_trigger: f32,
    pub left_bumper: bool,
    pub right_bumper: bool,
    pub d_pad_down: bool,
    pub d_pad_up: bool,
    pub d_pad_left: bool,
    pub d_pad_right: bool,
    pub controll: bool,
}

impl Arm {
    
}
