//! Test-only models, no hardware: the wire from the receiver to the flight controller, a UART
//! receiver that resamples it bit by bit, a transmitter with its drain time, frame builders,
//! application hooks that record, and a driver for the sans-IO core on a made-up clock.
//!
//! Assumptions of the UART model (time in picoseconds):
//! - 8N1, idle high, least significant bit first. The sender sends each burst back to back at one
//!   baud rate with exact bit times; bursts do not overlap.
//! - The receiver hunts for the first instant the line is low (a PL011 finds it within 1/16 bit,
//!   so its sample points are up to 1/16 bit later than here), checks that the line is still low
//!   at the middle of the start bit (otherwise a false start: hunting resumes there), samples the 8
//!   data bits and the stop bit at their centres at its own baud rate, and delivers the character
//!   at the centre of the stop bit.
//! - A low stop bit is a framing error, or a break when the 8 data bits were 0 too; after a break
//!   the receiver waits for the line to go high, after a framing error it hunts again from the
//!   centre of the stop bit (a line still low there starts the next character).
//! - A character with an error is not delivered as a byte, only as the error (embassy-rp drops the
//!   byte and reports the error).
//! - A baud change abandons the character in progress, keeps the characters already received
//!   (embassy-rp does not clear its ring buffer) and leaves the receiver off for 15 bit periods
//!   of the new rate (the wait inside embassy-rp's `set_baudrate`).
//! - No parity, no noise, no clock error, no overrun (the receive buffer is unbounded).

use crate::{Action, Config, Hooks, Mode, RcPort, Report, Stats, UartError};
use crsf_codec::{Battery, MAX_FRAME_LEN, encode_battery};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, MessageData, ParserError};
use mavlink_device::dialect::{ATTITUDE_DATA, RADIO_STATUS_DATA, RC_CHANNELS_OVERRIDE_DATA};
use mavlink_device::{
    Incoming, Outgoing, ParamError, ParamName, ParamStore, ParamValue, Stream, StreamSlot,
    Telemetry,
};
use std::collections::VecDeque;
use std::vec::Vec;

/// Picoseconds per second and per microsecond.
pub const PS: u64 = 1_000_000_000_000;
pub const US: u64 = 1_000_000;
/// Transmit FIFO depth of the RP2350 UART (PL011).
pub const TX_FIFO: u64 = 32;
/// The receiver's MAVLink identity as ExpressLRS sets it by default: system 255, component 68
/// (MAV_COMP_ID_TELEMETRY_RADIO).
pub const ELRS_SYS: u8 = 255;
pub const ELRS_COMP: u8 = 68;
/// The flight controller's MAVLink system id in the tests.
pub const OWN_SYS: u8 = 1;

struct Burst {
    start: u64,
    baud: u32,
    bytes: Vec<u8>,
}

impl Burst {
    /// Start of bit `bit` (10 per byte), rounded up so that `bit_at(edge(k)) == k`.
    fn edge(&self, bit: u64) -> u64 {
        self.start + (u128::from(bit) * u128::from(PS)).div_ceil(u128::from(self.baud)) as u64
    }
    fn bits(&self) -> u64 {
        self.bytes.len() as u64 * 10
    }
    fn end(&self) -> u64 {
        self.edge(self.bits())
    }
    fn bit_at(&self, t: u64) -> u64 {
        (u128::from(t - self.start) * u128::from(self.baud) / u128::from(PS)) as u64
    }
    fn level(&self, bit: u64) -> bool {
        let byte = self.bytes[(bit / 10) as usize];
        match bit % 10 {
            0 => false,
            9 => true,
            k => (byte >> (k - 1)) & 1 == 1,
        }
    }
}

/// The line from the receiver's TX to the flight controller's RX.
#[derive(Default)]
pub struct Wire {
    bursts: Vec<Burst>,
}

impl Wire {
    /// Sends `bytes` back to back at `baud` from `start` (or from the end of the previous burst,
    /// if later); returns the end of the last stop bit.
    pub fn send(&mut self, start: u64, baud: u32, bytes: &[u8]) -> u64 {
        let burst = Burst {
            start: start.max(self.end()),
            baud,
            bytes: bytes.to_vec(),
        };
        let end = burst.end();
        if !bytes.is_empty() {
            self.bursts.push(burst);
        }
        end
    }

    pub fn end(&self) -> u64 {
        self.bursts.last().map_or(0, Burst::end)
    }

    fn burst_at(&self, t: u64) -> Option<&Burst> {
        let i = self
            .bursts
            .partition_point(|b| b.start <= t)
            .checked_sub(1)?;
        self.bursts.get(i)
    }

    fn level(&self, t: u64) -> bool {
        self.burst_at(t).is_none_or(|b| {
            let bit = b.bit_at(t);
            bit >= b.bits() || b.level(bit)
        })
    }

    /// The first instant at or after `t` at which the line is low.
    fn next_low(&self, t: u64) -> Option<u64> {
        if !self.level(t) {
            return Some(t);
        }
        let first = self
            .bursts
            .partition_point(|b| b.start <= t)
            .saturating_sub(1);
        for b in &self.bursts[first..] {
            if b.end() <= t {
                continue;
            }
            let from = if t >= b.start { b.bit_at(t) + 1 } else { 0 };
            if let Some(bit) = (from..b.bits()).find(|&k| !b.level(k)) {
                return Some(b.edge(bit));
            }
        }
        None
    }

    /// The first instant at or after `t` at which the line is high.
    fn next_high(&self, t: u64) -> u64 {
        match self.burst_at(t) {
            Some(b) if !self.level(t) => {
                let bit = (b.bit_at(t) + 1..b.bits())
                    .find(|&k| b.level(k))
                    .unwrap_or(b.bits());
                b.edge(bit)
            }
            _ => t,
        }
    }
}

/// What the receiving UART hands over: a byte or an error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rx {
    Byte(u8),
    Err(UartError),
}

/// The flight controller's UART receiver: resamples the wire at its own baud rate.
pub struct Receiver {
    pub baud: u32,
    hunt: u64,
    out: VecDeque<(u64, Rx)>,
}

impl Receiver {
    pub fn new(baud: u32) -> Self {
        Self {
            baud,
            hunt: 0,
            out: VecDeque::new(),
        }
    }

    /// Centre of bit `k` (0 start, 1 to 8 data, 9 stop) of a character starting at `t0`.
    fn sample(&self, t0: u64, k: u64) -> u64 {
        t0 + (u128::from(2 * k + 1) * u128::from(PS) / (2 * u128::from(self.baud))) as u64
    }

    /// Decodes every character whose stop bit is sampled at or before `t`.
    pub fn advance(&mut self, wire: &Wire, t: u64) {
        while let Some(t0) = wire.next_low(self.hunt) {
            let start = self.sample(t0, 0);
            if start > t {
                return;
            }
            if wire.level(start) {
                self.hunt = start;
                continue;
            }
            let stop = self.sample(t0, 9);
            if stop > t {
                return;
            }
            let mut byte = 0u8;
            for k in 1..=8 {
                if wire.level(self.sample(t0, k)) {
                    byte |= 1 << (k - 1);
                }
            }
            if wire.level(stop) {
                self.out.push_back((stop, Rx::Byte(byte)));
                self.hunt = stop;
            } else if byte == 0 {
                self.out.push_back((stop, Rx::Err(UartError::Break)));
                self.hunt = wire.next_high(stop);
            } else {
                self.out.push_back((stop, Rx::Err(UartError::Framing)));
                self.hunt = stop;
            }
        }
    }

    /// A baud change at `t`.
    pub fn set_baud(&mut self, wire: &Wire, t: u64, baud: u32) {
        self.advance(wire, t);
        self.baud = baud;
        self.hunt = t + 15 * PS / u64::from(baud);
    }

    /// The characters received up to `t`, oldest first, with their times.
    pub fn take(&mut self, t: u64) -> Vec<(u64, Rx)> {
        let mut v = Vec::new();
        while self.out.front().is_some_and(|(at, _)| *at <= t) {
            v.extend(self.out.pop_front());
        }
        v
    }
}

/// Everything received on `wire` by a UART at `baud` listening from `from` to `to`.
pub fn resample(wire: &Wire, baud: u32, from: u64, to: u64) -> Vec<(u64, Rx)> {
    let mut rx = Receiver::new(baud);
    rx.hunt = from;
    rx.advance(wire, to);
    rx.take(to)
}

/// The flight controller's UART transmitter: a software buffer, a 32-byte FIFO and the shift
/// register, draining at the baud rate.
pub struct Transmitter {
    pub baud: u32,
    pub idle_at: u64,
}

impl Transmitter {
    fn byte_ps(&self) -> u64 {
        10 * PS / u64::from(self.baud)
    }
    pub fn write(&mut self, t: u64, n: usize) {
        self.idle_at = self.idle_at.max(t) + n as u64 * self.byte_ps();
    }
    /// When the software buffer is empty (what `flush` waits for).
    pub fn flushed_at(&self, t: u64) -> u64 {
        t.max(self.idle_at.saturating_sub(TX_FIFO * self.byte_ps()))
    }
    pub fn idle(&self, t: u64) -> bool {
        t >= self.idle_at
    }
}

/// CRC-8, polynomial 0xD5, over type and payload (CRSF).
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub fn crsf_frame(frame_type: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = std::vec![0xC8, (payload.len() + 2) as u8, frame_type];
    f.extend_from_slice(payload);
    f.push(crc8(&f[2..]));
    f
}

/// 16 values of 11 bits, least significant bit first.
pub fn pack(raw: &[u16; 16]) -> [u8; 22] {
    let mut out = [0u8; 22];
    for (channel, value) in raw.iter().enumerate() {
        for bit in 0..11 {
            if (value >> bit) & 1 == 1 {
                let position = channel * 11 + bit;
                out[position / 8] |= 1 << (position % 8);
            }
        }
    }
    out
}

pub fn crsf_rc(raw: &[u16; 16]) -> Vec<u8> {
    crsf_frame(0x16, &pack(raw))
}

pub fn crsf_link(fields: [u8; 10]) -> Vec<u8> {
    crsf_frame(0x14, &fields)
}

/// A MAVLink 2 frame (trailing zeros of the payload cut, as every MAVLink 2 sender does).
pub fn mav<M: MessageData>(sys: u8, comp: u8, seq: u8, m: &M) -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message_data(
        MavHeader {
            system_id: u32::from(sys),
            component_id: comp,
            sequence: seq,
        },
        m,
    );
    raw.raw_bytes().to_vec()
}

pub fn overrides(target_system: u8, ch: &[u16; 18]) -> RC_CHANNELS_OVERRIDE_DATA {
    RC_CHANNELS_OVERRIDE_DATA {
        chan1_raw: ch[0],
        chan2_raw: ch[1],
        chan3_raw: ch[2],
        chan4_raw: ch[3],
        chan5_raw: ch[4],
        chan6_raw: ch[5],
        chan7_raw: ch[6],
        chan8_raw: ch[7],
        target_system,
        target_component: 0,
        chan9_raw: ch[8],
        chan10_raw: ch[9],
        chan11_raw: ch[10],
        chan12_raw: ch[11],
        chan13_raw: ch[12],
        chan14_raw: ch[13],
        chan15_raw: ch[14],
        chan16_raw: ch[15],
        chan17_raw: ch[16],
        chan18_raw: ch[17],
    }
}

pub fn radio_status(rssi: u8, txbuf: u8) -> RADIO_STATUS_DATA {
    RADIO_STATUS_DATA {
        rxerrors: 3,
        fixed: 1,
        rssi,
        remrssi: 77,
        txbuf,
        noise: 250,
        remnoise: 0,
    }
}

/// The id of the application's own MAVLink message in the tests (not in the common dialect).
pub const CUSTOM_ID: u32 = 42_420;
/// An id in no accept list (not in the common dialect).
pub const UNLISTED_ID: u32 = 42_421;
/// The CRC_EXTRA of both.
pub const CUSTOM_EXTRA: u8 = 0x5A;

/// A made-up message `ID` whose payload is a 4-byte marker (little-endian, trailing zeros cut).
pub struct Custom<const ID: u32>(pub u32);

impl<const ID: u32> MessageData for Custom<ID> {
    type Message = mavlink_device::dialect::MavMessage;
    const ID: u32 = ID;
    const NAME: &'static str = "MARV_TEST";
    const EXTRA_CRC: u8 = CUSTOM_EXTRA;
    const ENCODED_LEN: usize = 4;

    fn ser(&self, _: MavlinkVersion, payload: &mut [u8]) -> usize {
        let b = self.0.to_le_bytes();
        payload[..4].copy_from_slice(&b);
        4 - b.iter().rev().take_while(|x| **x == 0).count().min(3)
    }

    fn deser(_: MavlinkVersion, payload: &[u8]) -> Result<Self, ParserError> {
        let mut b = [0u8; 4];
        let n = payload.len().min(4);
        b[..n].copy_from_slice(&payload[..n]);
        Ok(Self(u32::from_le_bytes(b)))
    }
}

/// The application's config with [`CUSTOM_ID`] on its accept list.
pub fn custom_listed() -> mavlink_device::Config {
    mavlink_device::Config::new(OWN_SYS, 1)
        .with_accept_list(&[(CUSTOM_ID, CUSTOM_EXTRA)])
        .unwrap()
}

/// No parameters.
pub struct NoParams;

impl ParamStore for NoParams {
    fn count(&self) -> u16 {
        0
    }
    fn name(&self, _: u16) -> Option<ParamName> {
        None
    }
    fn get(&self, _: u16) -> Option<ParamValue> {
        None
    }
    fn find(&self, _: &ParamName) -> Option<u16> {
        None
    }
    fn set(&mut self, _: u16, _: ParamValue) -> Result<ParamValue, ParamError> {
        Err(ParamError::NotFound)
    }
}

/// One stream, ATTITUDE (id 30) every 20 ms, so that MAVLink mode writes often.
pub struct Tel;

impl Telemetry for Tel {
    fn sample(&mut self, id: u32, _now_us: u64, out: &mut Outgoing<'_>) -> bool {
        id == ATTITUDE_DATA::ID && out.push(&ATTITUDE_DATA::default()).is_ok()
    }
}

pub fn slots() -> [StreamSlot; 1] {
    [StreamSlot::new(Stream {
        message_id: ATTITUDE_DATA::ID,
        default_interval_us: 20_000,
        min_interval_us: 10_000,
    })]
}

/// The battery frame the recording hooks hand to the port.
pub const BATTERY: Battery = Battery {
    voltage_dv: 168,
    current_da: 12,
    capacity_used_mah: 450,
    remaining_percent: 80,
};

/// Hooks that record every report and the last counters, and offer a battery frame each time
/// they are asked.
#[derive(Default)]
pub struct Rec {
    pub reports: Vec<Report>,
    pub last_stats: Option<Stats>,
    pub crsf_asked: Vec<u64>,
    /// `Hooks::mavlink` calls: (message id, marker, `now_us`).
    pub mavlink: Vec<(u32, u32, u64)>,
    /// `Stats::tx_busy_polls` at each `Hooks::stats` call.
    pub tx_busy_seen: Vec<u32>,
    /// `Stats::stream_slowdown_ms` at each `Hooks::stats` call.
    pub slowdown_seen: Vec<u32>,
}

impl Hooks for Rec {
    fn report(&mut self, report: Report) {
        self.reports.push(report);
    }
    fn crsf_frame(&mut self, now_us: u64, out: &mut [u8; MAX_FRAME_LEN]) -> Option<usize> {
        self.crsf_asked.push(now_us);
        encode_battery(&BATTERY, out).ok()
    }
    fn stats(&mut self, stats: &Stats) {
        self.last_stats = Some(*stats);
        self.tx_busy_seen.push(stats.tx_busy_polls);
        self.slowdown_seen.push(stats.stream_slowdown_ms);
    }
    fn mavlink(&mut self, msg: Incoming<'_>) {
        let marker = msg.decode::<Custom<CUSTOM_ID>>().map_or(u32::MAX, |c| c.0);
        let marker = msg.decode::<Custom<UNLISTED_ID>>().map_or(marker, |c| c.0);
        self.mavlink.push((msg.message_id, marker, msg.now_us));
    }
}

/// What the fake UART saw, with the time (us).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ev {
    Write {
        t_us: u64,
        bytes: Vec<u8>,
        mode: Mode,
        baud: u32,
    },
    /// `t_us` when the action was taken up, `done_us` when the wait ended.
    Flush {
        t_us: u64,
        done_us: u64,
    },
    TxIdle {
        t_us: u64,
        done_us: u64,
    },
    SetBaud {
        t_us: u64,
        baud: u32,
        flushed: bool,
        tx_idle: bool,
    },
}

/// A tiny deterministic generator for the split points of the chunking test.
pub struct XorShift(pub u64);

impl XorShift {
    pub fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 % n as u64) as usize
    }
}

/// How the driver cuts each run of received bytes into `receive` calls.
pub enum Split {
    Whole,
    Random(XorShift),
    OneByte,
}

/// The sans-IO core on the model: wakes every `step_us` or at the port's deadline, hands it the
/// characters received since the last wake (bytes in one call per run between errors), then
/// carries out its actions, waiting on the transmitter model for `Flush` and `WaitTxIdle`.
pub struct Sim<'a> {
    pub port: RcPort<'a>,
    pub wire: Wire,
    pub rx: Receiver,
    pub tx: Transmitter,
    /// The clock, in picoseconds.
    pub t: u64,
    pub step_us: u64,
    /// Bytes the fake UART takes per write call.
    pub write_chunk: usize,
    /// How each run of bytes is cut into `receive` calls (all at the same time).
    pub split: Split,
    pub hooks: Rec,
    pub params: NoParams,
    pub tel: Tel,
    pub trace: Vec<Ev>,
    /// Every mode the port went through, with `Stats::mode_since_us`.
    pub modes: Vec<(u64, Mode)>,
    /// The fake UART's baud rate (its own record, not the port's).
    pub baud: u32,
}

impl<'a> Sim<'a> {
    pub fn new(config: Config, slots: &'a mut [StreamSlot]) -> Self {
        Self::with_mav(config, mavlink_device::Config::new(OWN_SYS, 1), slots)
    }

    pub fn with_mav(
        config: Config,
        mav: mavlink_device::Config,
        slots: &'a mut [StreamSlot],
    ) -> Self {
        let start_baud = 115_200;
        Self {
            port: RcPort::new(config, mav, slots).unwrap(),
            wire: Wire::default(),
            rx: Receiver::new(start_baud),
            tx: Transmitter {
                baud: start_baud,
                idle_at: 0,
            },
            t: 0,
            step_us: 1_000,
            write_chunk: usize::MAX,
            split: Split::Whole,
            hooks: Rec::default(),
            params: NoParams,
            tel: Tel,
            trace: Vec::new(),
            modes: std::vec![(0, Mode::Searching)],
            baud: start_baud,
        }
    }

    pub fn now_us(&self) -> u64 {
        self.t / US
    }

    /// Runs the model up to `end_us` (inclusive).
    pub fn run_until(&mut self, end_us: u64) {
        loop {
            self.wake();
            let now = self.now_us();
            if now >= end_us {
                return;
            }
            let next = self
                .port
                .next_deadline_us()
                .max(now + 1)
                .min(now + self.step_us)
                .min(end_us);
            self.t = self.t.max(next * US);
        }
    }

    fn wake(&mut self) {
        let now = self.now_us();
        self.rx.advance(&self.wire, self.t);
        let mut bytes = Vec::new();
        for (_, item) in self.rx.take(self.t) {
            match item {
                Rx::Byte(b) => bytes.push(b),
                Rx::Err(e) => {
                    self.feed(&bytes, now);
                    bytes.clear();
                    self.port.uart_error(e, now);
                }
            }
        }
        self.feed(&bytes, now);
        self.drain();
        let s = self.port.stats();
        if self.modes.last().map(|m| m.1) != Some(s.mode) {
            self.modes.push((s.mode_since_us, s.mode));
        }
    }

    fn feed(&mut self, bytes: &[u8], now: u64) {
        let mut rest = bytes;
        while !rest.is_empty() {
            let n = match &mut self.split {
                Split::Whole => rest.len(),
                Split::Random(rng) => 1 + rng.below(rest.len()),
                Split::OneByte => 1,
            };
            self.port
                .receive(&rest[..n], now, &mut self.params, &mut self.hooks);
            rest = &rest[n..];
        }
    }

    fn drain(&mut self) {
        loop {
            let now = self.now_us();
            let Some(action) =
                self.port
                    .poll_action(now, &self.params, &mut self.tel, &mut self.hooks)
            else {
                return;
            };
            match action {
                Action::Write(bytes) => {
                    let n = bytes.len().min(self.write_chunk);
                    let data = bytes[..n].to_vec();
                    self.tx.write(self.t, n);
                    self.port.written(n);
                    self.trace.push(Ev::Write {
                        t_us: now,
                        bytes: data,
                        mode: self.port.stats().mode,
                        baud: self.baud,
                    });
                }
                Action::Flush => {
                    self.t = self.tx.flushed_at(self.t);
                    self.trace.push(Ev::Flush {
                        t_us: now,
                        done_us: self.now_us(),
                    });
                    self.port.done(self.now_us());
                }
                Action::WaitTxIdle => {
                    self.t = self.t.max(self.tx.idle_at);
                    self.trace.push(Ev::TxIdle {
                        t_us: now,
                        done_us: self.now_us(),
                    });
                    self.port.done(self.now_us());
                }
                Action::SetBaud(baud) => {
                    self.trace.push(Ev::SetBaud {
                        t_us: now,
                        baud,
                        flushed: self.tx.flushed_at(self.t) <= self.t,
                        tx_idle: self.tx.idle(self.t),
                    });
                    self.rx.set_baud(&self.wire, self.t, baud);
                    self.tx.baud = baud;
                    self.baud = baud;
                    self.t += 15 * PS / u64::from(baud);
                    self.port.done(self.now_us());
                }
            }
        }
    }

    /// The `SetBaud` events: (time, baud).
    pub fn bauds(&self) -> Vec<(u64, u32)> {
        self.trace
            .iter()
            .filter_map(|e| match e {
                Ev::SetBaud { t_us, baud, .. } => Some((*t_us, *baud)),
                _ => None,
            })
            .collect()
    }

    /// The stick reports.
    pub fn sticks(&self) -> Vec<crate::StickReport> {
        self.hooks
            .reports
            .iter()
            .filter_map(|r| match r {
                Report::Sticks(s) => Some(*s),
                _ => None,
            })
            .collect()
    }
}
