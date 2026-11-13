//! What the port reports: one stick report type for both protocols and two raw link reports.

use crsf_codec::LinkStatistics;

/// Where a stick report came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// A CRSF RC channels frame (type 0x16).
    Crsf,
    /// A MAVLink RC_CHANNELS_OVERRIDE (id 70), with its sender. Which senders to accept is the
    /// application's.
    Mavlink {
        /// The sender's MAVLink system id.
        system_id: u8,
        /// The sender's MAVLink component id.
        component_id: u8,
    },
}

/// The sticks of one frame that passed its CRC while the port was locked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StickReport {
    /// Channels 1 to 16 in microseconds. CRSF: `crsf_codec::raw_to_us` of each raw value. MAVLink:
    /// `chan1_raw` to `chan16_raw` exactly as received, 0 and 65535 ("ignore") included; channels 9
    /// to 16 of a payload cut before them read 0; channels 17 and 18 are dropped.
    pub channels_us: [u16; 16],
    /// The protocol and, for MAVLink, the sender.
    pub source: Source,
    /// The `now_us` of the call that received the frame.
    pub at_us: u64,
    /// The number of stick reports so far, this one included (the first is 1); wraps at `u32::MAX`.
    pub seq: u32,
}

/// The ten fields of a CRSF link statistics frame (type 0x14), raw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrsfLinkReport {
    /// The fields as received.
    pub stats: LinkStatistics,
    /// The `now_us` of the call that received the frame.
    pub at_us: u64,
    /// The number of CRSF link reports so far, this one included; wraps at `u32::MAX`.
    pub seq: u32,
}

/// The fields of a MAVLink RADIO_STATUS (id 109), raw as received (a payload cut before a field
/// reads 0 there).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioStatus {
    /// `rssi`.
    pub rssi: u8,
    /// `remrssi`.
    pub remrssi: u8,
    /// `txbuf`, percent of free transmit buffer.
    pub txbuf: u8,
    /// `noise`.
    pub noise: u8,
    /// `remnoise`.
    pub remnoise: u8,
    /// `rxerrors`.
    pub rxerrors: u16,
    /// `fixed`.
    pub fixed: u16,
}

/// One RADIO_STATUS and its sender.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioStatusReport {
    /// The fields as received.
    pub status: RadioStatus,
    /// The sender's MAVLink system id.
    pub system_id: u8,
    /// The sender's MAVLink component id.
    pub component_id: u8,
    /// The `now_us` of the call that received the frame.
    pub at_us: u64,
    /// The number of RADIO_STATUS reports so far, this one included; wraps at `u32::MAX`.
    pub seq: u32,
}

/// One report, handed to [`Hooks::report`](crate::Hooks::report) from inside
/// [`RcPort::receive`](crate::RcPort::receive), once per frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Report {
    /// Sticks, from either protocol.
    Sticks(StickReport),
    /// CRSF link statistics.
    CrsfLink(CrsfLinkReport),
    /// MAVLink RADIO_STATUS.
    RadioStatus(RadioStatusReport),
}
