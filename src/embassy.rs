//! Feature `embassy-rp-0-10`: the traits of [`RcPort::run`](crate::RcPort::run) for the unsplit
//! `embassy_rp::uart::BufferedUart` of embassy-rp 0.10.

use crate::port::UartError;
use crate::run::{SetBaud, TxBusy};
use embassy_rp::uart::{BufferedUart, Error};

/// `BufferedUart::set_baudrate` (`uart/buffered.rs:175-178`): it disables the UART, waits 15 bit
/// periods, re-enables it; the receive buffer is kept.
impl SetBaud for BufferedUart {
    fn set_baud(&mut self, baud: u32) {
        self.set_baudrate(baud);
    }
}

/// `BufferedUart::busy`: the PL011 BUSY flag, set while the transmit FIFO holds data or a
/// character is being shifted out.
impl TxBusy for BufferedUart {
    fn tx_busy(&self) -> bool {
        self.busy()
    }
}

impl From<Error> for UartError {
    fn from(e: Error) -> Self {
        match e {
            Error::Framing => UartError::Framing,
            Error::Parity => UartError::Parity,
            Error::Break => UartError::Break,
            Error::Overrun => UartError::Overrun,
            _ => UartError::Other,
        }
    }
}
