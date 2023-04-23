#![no_std]
#![warn(missing_docs)]

extern crate drs_0x01;

/// A module that implements a communication interface when creating new servo.
pub mod motors;

/// A module that implements a USART communication for the servos communications.
pub mod communication;

pub mod motor;