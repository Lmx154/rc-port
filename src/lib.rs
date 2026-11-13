//! The UART of an ExpressLRS receiver on a flight controller. The receiver changes by itself
//! between CRSF at 420000 baud and MAVLink 2 at 460800 baud when the transmitter changes model;
//! this crate follows it and REPORTS what arrives. It decides nothing: no loss flag, no failsafe,
//! no stick source, no arming rule. Those are the firmware's.
//!
//! [`RcPort`] is a sans-IO core over a `crsf_codec::Deframer` and a `mavlink_device::Node`:
//! bytes, UART errors and `now_us` in; [`Action`]s (writes, and baud changes as flush, wait for
//! the transmitter, set baud) and [`Report`]s out. [`RcPort::run`] drives it on any UART that
//! implements `embedded_io_async::{Read, Write}`, [`SetBaud`] and [`TxBusy`], with an
//! `embedded_hal_async` `DelayNs`. Feature `embassy-rp-0-10` implements the two traits for the
//! unsplit embassy-rp 0.10 `BufferedUart`.
//!
//! States: Searching (460800 for [`Config::d_mav_us`], then the CRSF baud for
//! [`Config::d_crsf_us`], and so on), Crsf, Mavlink. [`LOCK_FRAMES`] valid frames at one baud lock
//! it; frames 1 and 2 are counted, not reported. A lock is left after [`T_LEAVE_CRSF_US`] or
//! [`Config::t_leave_mav_us`] without a valid frame; the search then starts at the other baud.
//! Nothing is written while searching. See README.md.
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

#[cfg(test)]
extern crate std;

mod config;
#[cfg(feature = "embassy-rp-0-10")]
mod embassy;
mod port;
mod report;
mod run;

#[cfg(test)]
mod run_tests;
#[cfg(test)]
mod sim;
#[cfg(test)]
mod tests;

/// The CRSF codec the port uses (re-exported so that the application uses the same version).
pub use crsf_codec;
/// The MAVLink node crate the port uses (re-exported so that the application uses the same
/// version).
pub use mavlink_device;

pub use config::{
    CRSF_BAUD, CRSF_TX_INTERVAL_US, Config, D_CRSF_US, D_MAV_US, LOCK_FRAMES, MAVLINK_BAUD,
    T_LEAVE_CRSF_US, T_LEAVE_MAV_US,
};
pub use port::{Action, Hooks, Mode, NewError, RcPort, Stats, UartError};
pub use report::{CrsfLinkReport, RadioStatus, RadioStatusReport, Report, Source, StickReport};
pub use run::{RunError, SetBaud, TxBusy};

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
