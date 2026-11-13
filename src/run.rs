//! The async front end: the port on one UART that reads, writes, changes baud and tells whether its
//! transmitter is busy.

use crate::port::{Action, Hooks, RcPort, Stats, UartError};
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, Write};
use mavlink_device::{ParamStore, Telemetry};

/// Bytes read from the UART at a time.
const READ_BUF: usize = 64;
/// Longest sleep of `run`: the node's text hook is asked at least this often (as in `Node::run`).
const MAX_WAIT_US: u64 = 100_000;
/// Spacing of the busy checks while waiting for the transmitter to go idle: about one byte at
/// 420000 baud (23.8 us).
const TX_IDLE_POLL_US: u32 = 20;

/// Changes the baud rate of a running UART.
pub trait SetBaud {
    /// Sets `baud` on the UART, receiver and transmitter. [`RcPort::run`] calls it only after the
    /// transmitter is idle.
    fn set_baud(&mut self, baud: u32);
}

/// Tells whether the transmitter is still sending.
pub trait TxBusy {
    /// True from the first byte handed to the hardware until its last stop bit has left the pin.
    fn tx_busy(&self) -> bool;
}

/// Why [`RcPort::run`] returned. Read errors never end it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunError<E> {
    /// `write` or `flush` failed.
    Write(E),
    /// `write` took 0 bytes of a non-empty buffer.
    WriteZero,
    /// `read` returned 0 bytes: the stream ended.
    EndOfStream,
}

enum Woke<E> {
    Read(Result<usize, E>),
    Timer,
}

impl RcPort<'_> {
    /// Drives the port on `uart` forever: carries out every [`Action`] (a write with `write`,
    /// acknowledged byte-exactly; a baud change as `flush`, then a [`TxBusy::tx_busy`] check every
    /// 20 us through `delay` until it is false, with no timeout, each busy answer counted in
    /// [`Stats::tx_busy_polls`], then [`SetBaud::set_baud`]), then waits for the
    /// first of bytes from `uart` and [`next_deadline_us`](RcPort::next_deadline_us) (at most
    /// 100 ms). A read error is converted to [`UartError`] and counted, and the loop reads again at
    /// once. [`Hooks::stats`] gets the counters once per wake, after the actions, when they
    /// changed, and after each busy check of the wait for the transmitter (so that a transmitter
    /// that stays busy shows as a rising [`Stats::tx_busy_polls`]).
    ///
    /// It returns only on a write or flush error, a write of 0 bytes or a read of 0 bytes; the
    /// port keeps its state and `run` can be called again.
    ///
    /// Cancel safety: the port changes only in synchronous calls between awaits, so dropping the
    /// future at any await loses nothing the port holds; an unfinished action is offered again by
    /// the next `run`. `uart.read` must itself be cancel-safe (the read is dropped whenever the
    /// timer wins; embassy-rp 0.10's `BufferedUart::read` is, by inspection). `now_us` must be
    /// non-decreasing and on the clock of `delay`. While `run` is awaited the port cannot be
    /// touched: reports, CRSF telemetry and counters go through `hooks`, MAVLink through `params`
    /// and `tel`.
    pub async fn run<U, D>(
        &mut self,
        uart: &mut U,
        delay: &mut D,
        mut now_us: impl FnMut() -> u64,
        params: &mut impl ParamStore,
        tel: &mut impl Telemetry,
        hooks: &mut impl Hooks,
    ) -> RunError<U::Error>
    where
        U: Read + Write + SetBaud + TxBusy,
        U::Error: Into<UartError>,
        D: DelayNs,
    {
        let mut buf = [0u8; READ_BUF];
        let mut reported: Option<Stats> = None;
        loop {
            while let Some(action) = self.poll_action(now_us(), &*params, tel, hooks) {
                match action {
                    Action::Write(bytes) => match uart.write(bytes).await {
                        Ok(0) => return RunError::WriteZero,
                        Ok(n) => self.written(n),
                        Err(e) => return RunError::Write(e),
                    },
                    Action::Flush => {
                        if let Err(e) = uart.flush().await {
                            return RunError::Write(e);
                        }
                        self.done(now_us());
                    }
                    Action::WaitTxIdle => {
                        while uart.tx_busy() {
                            self.count_tx_busy();
                            let stats = self.stats();
                            hooks.stats(&stats);
                            reported = Some(stats);
                            delay.delay_us(TX_IDLE_POLL_US).await;
                        }
                        self.done(now_us());
                    }
                    Action::SetBaud(baud) => {
                        uart.set_baud(baud);
                        self.done(now_us());
                    }
                }
            }
            let stats = self.stats();
            if reported != Some(stats) {
                hooks.stats(&stats);
                reported = Some(stats);
            }
            let wait_us = self
                .next_deadline_us()
                .saturating_sub(now_us())
                .clamp(1, MAX_WAIT_US);
            let wait_us = u32::try_from(wait_us).unwrap_or(u32::MAX);
            let woke = {
                let mut read = pin!(uart.read(&mut buf));
                let mut timer = pin!(delay.delay_us(wait_us));
                poll_fn(|cx| {
                    if let Poll::Ready(r) = read.as_mut().poll(cx) {
                        Poll::Ready(Woke::Read(r))
                    } else if timer.as_mut().poll(cx).is_ready() {
                        Poll::Ready(Woke::Timer)
                    } else {
                        Poll::Pending
                    }
                })
                .await
            };
            match woke {
                Woke::Timer => {}
                Woke::Read(Ok(0)) => return RunError::EndOfStream,
                Woke::Read(Ok(n)) => {
                    if let Some(bytes) = buf.get(..n) {
                        self.receive(bytes, now_us(), params, hooks);
                    }
                }
                Woke::Read(Err(e)) => self.uart_error(e.into(), now_us()),
            }
        }
    }
}
