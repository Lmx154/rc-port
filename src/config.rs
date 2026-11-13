//! The numbers of the protocol follower and the [`Config`] that holds those the bench will set.
//!
//! Sources: `_plan/decision-rc-link-20261113.md` section 4 (the architect's table) and
//! `_plan/facts-rc-link.md`; "obs. N" is observation N of the facts file's section 6, made on the
//! bench (R2a). Every duration is in microseconds.

/// CRSF baud rate, the default of [`Config::crsf_baud`]: ExpressLRS `options.cpp:198` default; it
/// can be changed when the receiver is flashed. To be set from the bench (obs. 1).
pub const CRSF_BAUD: u32 = 420_000;

/// MAVLink baud rate: ExpressLRS `rx_main.cpp:1287-1290`, fixed in the receiver's source.
pub const MAVLINK_BAUD: u32 = 460_800;

/// Valid frames needed at the current baud to lock; frames 1 and 2 are counted, not reported, the
/// 3rd and later are reported. ArduPilot's `requires_3_frames` (`AP_RCProtocol.h:146-175`).
pub const LOCK_FRAMES: u8 = 3;

/// The CRSF lock is left after this long without a valid CRSF frame (1000 ms). ArduPilot's
/// 1000 ms dwell (`AP_RCProtocol.cpp:359-418`): a fade shorter than 1 s never touches the baud.
pub const T_LEAVE_CRSF_US: u32 = 1_000_000;

/// The MAVLink lock is left after this long without a valid MAVLink frame (200 ms), the default of
/// [`Config::t_leave_mav_us`]: 20 RADIO_STATUS periods; it must exceed any same-mode re-open gap.
/// To be set from the bench (obs. 2 and 6).
pub const T_LEAVE_MAV_US: u32 = 200_000;

/// While searching, time spent listening at 460800 baud (50 ms), the default of
/// [`Config::d_mav_us`]: at least 5 RADIO_STATUS periods. To be set from the bench (obs. 2 and 6).
pub const D_MAV_US: u32 = 50_000;

/// While searching, time spent listening at the CRSF baud (200 ms), the default of
/// [`Config::d_crsf_us`]: at least 5 RC frame periods at the slowest packet rate. To be set from
/// the bench (obs. 5).
pub const D_CRSF_US: u32 = 200_000;

/// Shortest spacing of the CRSF frames written to the receiver (100 ms), the default of
/// [`Config::crsf_tx_interval_us`]. The facts file establishes no pacing rule (the receiver's
/// telemetry buffer limits were not read, `facts-rc-link.md` section 3), so this is a conservative
/// fixed interval: one frame per 100 ms. To be set from the bench (R3).
pub const CRSF_TX_INTERVAL_US: u32 = 100_000;

/// The numbers the bench will set (R2a, R3, R5), so that they change without touching the logic.
///
/// A duration of 0 behaves as 1 us.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Config {
    /// CRSF baud rate; [`CRSF_BAUD`] by default.
    pub crsf_baud: u32,
    /// Time without a valid MAVLink frame after which the MAVLink lock is left; [`T_LEAVE_MAV_US`]
    /// by default.
    pub t_leave_mav_us: u32,
    /// Listening time at 460800 baud while searching; [`D_MAV_US`] by default.
    pub d_mav_us: u32,
    /// Listening time at the CRSF baud while searching; [`D_CRSF_US`] by default.
    pub d_crsf_us: u32,
    /// Shortest spacing of the CRSF frames written; [`CRSF_TX_INTERVAL_US`] by default.
    pub crsf_tx_interval_us: u32,
}

impl Config {
    /// The defaults: the constants of this module.
    pub const fn new() -> Self {
        Self {
            crsf_baud: CRSF_BAUD,
            t_leave_mav_us: T_LEAVE_MAV_US,
            d_mav_us: D_MAV_US,
            d_crsf_us: D_CRSF_US,
            crsf_tx_interval_us: CRSF_TX_INTERVAL_US,
        }
    }

    /// Sets the CRSF baud rate.
    pub const fn with_crsf_baud(mut self, v: u32) -> Self {
        self.crsf_baud = v;
        self
    }

    /// Sets the time without a valid MAVLink frame after which the MAVLink lock is left.
    pub const fn with_t_leave_mav_us(mut self, v: u32) -> Self {
        self.t_leave_mav_us = v;
        self
    }

    /// Sets the listening time at 460800 baud while searching.
    pub const fn with_d_mav_us(mut self, v: u32) -> Self {
        self.d_mav_us = v;
        self
    }

    /// Sets the listening time at the CRSF baud while searching.
    pub const fn with_d_crsf_us(mut self, v: u32) -> Self {
        self.d_crsf_us = v;
        self
    }

    /// Sets the shortest spacing of the CRSF frames written.
    pub const fn with_crsf_tx_interval_us(mut self, v: u32) -> Self {
        self.crsf_tx_interval_us = v;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}
