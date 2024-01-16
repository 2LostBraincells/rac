#![allow(dead_code)]

/// Logging level all levels include the ones before
/// 0 = no logs
/// 1 = errors
/// 2 = warnings
/// 3 = info
/// 4 = debug
/// 5 = verbose
pub const LOG_LEVEL: u8 = 5;

pub fn error(message: &'static str) {
    if LOG_LEVEL < 1 {
        return;
    }

    println!("ERRO: {}", message);
}

pub fn warn(message: &'static str) {
    if LOG_LEVEL < 2 {
        return;
    }

    println!("WARN: {}", message);
}

pub fn info(message: &'static str) {
    if LOG_LEVEL < 3 {
        return;
    }

    println!("INFO: {}", message);
}

pub fn debug(message: &'static str) {
    if LOG_LEVEL < 4 {
        return;
    }

    println!("DEBG: {}", message);
}

pub fn verbose(message: &'static str) {
    if LOG_LEVEL < 5 {
        return;
    }

    println!("VERB: {}", message);
}
