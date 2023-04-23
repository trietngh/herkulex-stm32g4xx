use stm32g4xx_hal as hal;
use hal::serial::usart::{Rx, Tx};
use hal::prelude::*;
use hal::stm32;
use hal::interrupt;
use hal::gpio::Alternate;
use hal::gpio::gpioa::{PA2, PA3};
use hal::serial::{FifoThreshold, NoDMA, Parity, Serial, StopBits, usart, WordLength};
use hal::stm32::{Interrupt};

use drs_0x01::builder::HerkulexMessage;
use nb::block;
use cortex_m;

const BUFF_SIZE: usize = 20;
static mut BUFF_INDEX: usize = 0;
static mut BUFFER: &mut [u8; BUFF_SIZE] = &mut [0; BUFF_SIZE];
type USART_Tx = Tx<stm32::USART2, PA2<Alternate<7>>, NoDMA>;
type USART_Rx = Rx<stm32::USART2, PA3<Alternate<7>>, NoDMA>;

static mut Rx: Option<USART_Rx> = None;

/// A communication for the USART
pub struct Communication<'a> {
    tx: &'a mut USART_Tx,
}

/// A trait that implements the communication with a servo.
pub trait HerkulexCommunication {
    /// Send a message to the Herkulex servo
    fn send_message(&mut self, msg: HerkulexMessage);

    /// Receive a message from the Herkulex servo
    fn read_message(&self) -> [u8; BUFF_SIZE];
}

impl<'a> Communication<'a> {
    /// Create a new communication with Tx and Rx
    pub fn new(tx: &'a mut USART_Tx, mut rx: USART_Rx) -> Communication<'a> {
        let comm = Communication { tx };
        rx.listen();
        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }

        cortex_m::interrupt::free(|_| unsafe {
            Rx.replace(rx);
        });
        comm
    }
}

impl<'a> HerkulexCommunication for Communication<'a> {
    /// Send a message to the Herkulex servo
    fn send_message(&mut self, msg: HerkulexMessage) {
        for b in &msg {
            hal::block!(self.tx.write(*b)).unwrap();
        }
    }
    /// Receive a message from the Herkulex servo
    ///
    /// Can be stuck in the interruption
    fn read_message(&self) -> [u8; BUFF_SIZE] {
        cortex_m::asm::wfi(); // Can stay here forever


        let mut received_message: [u8; BUFF_SIZE] = [0; BUFF_SIZE];
        for i in 0..BUFF_SIZE {
            unsafe {
                received_message[i] = BUFFER[i];
            }
        }
        received_message
    }
}

#[interrupt]
unsafe fn USART2() {
    // When a packet is received, there is at least 3 bytes : header, header, packet size
    let mut packet_size = 3;
    unsafe {
        cortex_m::interrupt::free(|_| {
            if let Some(rx) = Rx.as_mut() {
                // If it received a packet and we have not read it entirely yet
                while rx.is_rxne() || BUFF_INDEX < packet_size {
                    // Read the byte
                    if let Ok(w) = rx.read() {
                        BUFFER[BUFF_INDEX] = w; // Fill the buffer

                        // If we read the packet size in the received packet
                        // it updates our packet_size to read all bytes from the received packet
                        if BUFF_INDEX == 2 {
                            packet_size = w as usize;
                        }

                        // If the buffer is full, it rewrites at the beginning
                        if BUFF_INDEX >= BUFF_SIZE - 1 {
                            BUFF_INDEX = 0;
                        }

                        BUFF_INDEX += 1;
                    }
                }
                BUFF_INDEX = 0;
            };
        })
    }
}
