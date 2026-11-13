//! The sans-IO core: bytes, UART errors and time in; actions and reports out.

use crate::config::{Config, LOCK_FRAMES, MAVLINK_BAUD, T_LEAVE_CRSF_US};
use crate::report::{CrsfLinkReport, RadioStatus, RadioStatusReport, Report, Source, StickReport};
use core::cell::Cell;
use crsf_codec::{Deframer, Frame, MAX_FRAME_LEN, raw_to_us};
use marv_dialect::MessageData;
use mavlink_device::dialect::{RADIO_STATUS_DATA, RC_CHANNELS_OVERRIDE_DATA};
use mavlink_device::{
    FRAME_BUF, Incoming, MAX_ACCEPT, Node, ParamError, ParamName, ParamStore, ParamValue,
    StreamSlot, Telemetry,
};

/// The port's own entries of its node's accept list: the two messages it reports.
const ACCEPT: [(u32, u8); 2] = [
    (
        RC_CHANNELS_OVERRIDE_DATA::ID,
        RC_CHANNELS_OVERRIDE_DATA::EXTRA_CRC,
    ),
    (RADIO_STATUS_DATA::ID, RADIO_STATUS_DATA::EXTRA_CRC),
];

/// What the port is doing, as reported in [`Stats::mode`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Mode {
    /// Not locked: listening at one baud rate for a while, then the other, or changing the baud.
    #[default]
    Searching,
    /// Locked on CRSF at [`Config::crsf_baud`].
    Crsf,
    /// Locked on MAVLink at [`MAVLINK_BAUD`].
    Mavlink,
}

/// A receive error the UART reported. The byte it concerns is lost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum UartError {
    /// The stop bit was low.
    Framing,
    /// The parity bit was wrong (not used at 8N1, counted if a UART reports it).
    Parity,
    /// The line was low for a whole character or longer.
    Break,
    /// A byte arrived with the receive FIFO full.
    Overrun,
    /// Any other read error.
    Other,
}

/// Why [`RcPort::new`] failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum NewError {
    /// The application's accept list plus the port's RC_CHANNELS_OVERRIDE and RADIO_STATUS (those
    /// it does not list already) is more than `mavlink_device::MAX_ACCEPT` (16) entries.
    AcceptListFull,
}

/// The next thing the caller must do on the UART, from [`RcPort::poll_action`].
///
/// A baud change is always the sequence [`Flush`](Action::Flush), [`WaitTxIdle`](Action::WaitTxIdle),
/// [`SetBaud`](Action::SetBaud), each acknowledged with [`RcPort::done`]; nothing is written
/// between them and nothing until the next lock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action<'a> {
    /// Write these bytes, then call [`RcPort::written`] with how many the UART took. The same
    /// action (its rest) comes back until all are taken.
    Write(&'a [u8]),
    /// Wait until every byte written so far has left the software buffer (`embedded_io_async`
    /// `flush`), then call [`RcPort::done`].
    Flush,
    /// Wait until the transmitter has sent its last stop bit (the UART is no longer busy), then
    /// call [`RcPort::done`].
    WaitTxIdle,
    /// Set the UART to this baud rate, then call [`RcPort::done`].
    SetBaud(u32),
}

/// The application's side of the port: where reports go and where CRSF telemetry comes from.
pub trait Hooks {
    /// One report, called from inside [`RcPort::receive`], once per frame, in stream order. It
    /// must not block.
    fn report(&mut self, report: Report);

    /// The next CRSF frame to write: write one whole frame (address, length, type, payload, CRC;
    /// for example with `crsf_codec::encode_battery`) to the start of `out` and return its length,
    /// or `None`. Called from [`RcPort::poll_action`] only while locked on CRSF, at most once per
    /// [`Config::crsf_tx_interval_us`] (the first time at the first poll after the lock). The port
    /// writes the bytes as given; a length of 0 or above 64 is ignored.
    fn crsf_frame(&mut self, now_us: u64, out: &mut [u8; MAX_FRAME_LEN]) -> Option<usize> {
        let _ = (now_us, out);
        None
    }

    /// A message of the application's accept list (`mavlink_device::Config::accept`) other than
    /// RC_CHANNELS_OVERRIDE and RADIO_STATUS, as the node handed it over (checks as for those
    /// two; not target-filtered or interpreted). Called from inside [`RcPort::receive`] only while
    /// locked on MAVLink, from the 3rd valid frame of the lock on, once per frame, in stream
    /// order, exactly like a stick report. It must not block; `msg` borrows the receive buffer.
    fn mavlink(&mut self, msg: Incoming<'_>) {
        let _ = msg;
    }

    /// The counters, called by [`RcPort::run`] once per wake when they changed, and after each
    /// busy check while it waits for the transmitter to go idle (the sans-IO core does not call
    /// it: use [`RcPort::stats`]).
    fn stats(&mut self, stats: &Stats) {
        let _ = stats;
    }
}

/// Counters and timestamps; nothing else. No loss flag: ages are the application's to compute.
///
/// The `u32` counters saturate, except the three report counters, which are the `seq` of the last
/// report of their kind and wrap.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct Stats {
    /// The mode now.
    pub mode: Mode,
    /// When the mode began: the `now_us` of the lock, or the deadline at which a lock was left
    /// (the last valid frame's time plus the leave time), or 0 at power-up.
    pub mode_since_us: u64,
    /// Locks whose protocol differs from the protocol of the previous lock (CRSF, searching, CRSF
    /// counts 0; the first lock after power-up counts 0).
    pub mode_changes: u32,
    /// Stick reports produced (the `seq` of the last one).
    pub sticks: u32,
    /// `at_us` of the last stick report.
    pub last_sticks_us: Option<u64>,
    /// CRSF link statistics reports produced.
    pub crsf_links: u32,
    /// `at_us` of the last CRSF link statistics report.
    pub last_crsf_link_us: Option<u64>,
    /// RADIO_STATUS reports produced.
    pub radio_status: u32,
    /// `at_us` of the last RADIO_STATUS report.
    pub last_radio_status_us: Option<u64>,
    /// Valid frames counted but not reported because they came before the lock (frames 1 and 2 of
    /// each lock attempt).
    pub lock_discarded: u32,
    /// RC_CHANNELS_OVERRIDE frames received while locked whose target system is neither the
    /// node's nor 0: not reported.
    pub overrides_not_for_us: u32,
    /// CRSF frames with a wrong CRC plus MAVLink frames with a wrong checksum (handled and listed
    /// messages; `mavlink_device::Stats::dropped_checksum` and `dropped_listed_checksum`).
    pub crc_errors: u32,
    /// Bytes thrown away outside any frame by either decoder, plus bytes received during a baud
    /// change (they were sampled at a baud the port no longer knows), plus the bytes the CRSF
    /// deframer held when a baud change ended (`Deframer::clear`). The bytes the node's MAVLink
    /// receive buffer held then are dropped by `Node::on_link_down` without being counted.
    pub garbage_bytes: u32,
    /// [`UartError::Framing`] events.
    pub uart_framing: u32,
    /// [`UartError::Parity`] events.
    pub uart_parity: u32,
    /// [`UartError::Break`] events.
    pub uart_break: u32,
    /// [`UartError::Overrun`] events.
    pub uart_overrun: u32,
    /// [`UartError::Other`] events.
    pub uart_other: u32,
    /// Checks of [`TxBusy::tx_busy`](crate::TxBusy::tx_busy) by [`RcPort::run`], while it waits
    /// for the transmitter to go idle before a baud change, that found it still busy (each is
    /// followed by a 20 us delay). It only counts: the wait has no timeout. Always 0 with the
    /// sans-IO core alone.
    pub tx_busy_polls: u32,
}

/// The application's parameters as the node sees them: reads as they are, writes refused with
/// [`ParamError::Rejected`] unless `open` (locked on MAVLink).
struct Gate<'p, P> {
    params: &'p mut P,
    open: &'p Cell<bool>,
}

impl<P: ParamStore> ParamStore for Gate<'_, P> {
    fn count(&self) -> u16 {
        self.params.count()
    }
    fn name(&self, i: u16) -> Option<ParamName> {
        self.params.name(i)
    }
    fn get(&self, i: u16) -> Option<ParamValue> {
        self.params.get(i)
    }
    fn find(&self, n: &ParamName) -> Option<u16> {
        self.params.find(n)
    }
    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        if self.open.get() {
            self.params.set(i, v)
        } else {
            Err(ParamError::Rejected)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Proto {
    Crsf,
    Mavlink,
}

impl Proto {
    fn other(self) -> Self {
        match self {
            Proto::Crsf => Proto::Mavlink,
            Proto::Mavlink => Proto::Crsf,
        }
    }

    fn mode(self) -> Mode {
        match self {
            Proto::Crsf => Mode::Crsf,
            Proto::Mavlink => Mode::Mavlink,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    Flush,
    WaitTxIdle,
    SetBaud,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// A baud change to `to`'s baud in progress.
    Switching { to: Proto, step: Step },
    /// Searching: listening at `proto`'s baud until `until_us`, `frames` valid frames counted.
    Listening {
        proto: Proto,
        frames: u8,
        until_us: u64,
    },
    /// Locked on `proto`.
    Locked { proto: Proto, last_valid_us: u64 },
}

/// The state machine and the port's own counters, apart from the decoders so that a decoder's
/// callback can borrow it.
struct Fsm {
    config: Config,
    lock_frames: u8,
    own_system_id: u8,
    state: State,
    last_locked: Option<Proto>,
    next_crsf_tx_us: u64,
    stats: Stats,
}

impl Fsm {
    fn t_leave_us(&self, proto: Proto) -> u64 {
        let t = match proto {
            Proto::Crsf => T_LEAVE_CRSF_US,
            Proto::Mavlink => self.config.t_leave_mav_us,
        };
        u64::from(t.max(1))
    }

    fn dwell_us(&self, proto: Proto) -> u64 {
        let d = match proto {
            Proto::Crsf => self.config.d_crsf_us,
            Proto::Mavlink => self.config.d_mav_us,
        };
        u64::from(d.max(1))
    }

    fn baud(&self, proto: Proto) -> u32 {
        match proto {
            Proto::Crsf => self.config.crsf_baud,
            Proto::Mavlink => MAVLINK_BAUD,
        }
    }

    fn locked_on(&self, proto: Proto) -> bool {
        matches!(self.state, State::Locked { proto: p, .. } if p == proto)
    }

    /// Applies the deadline due at `now_us`, if any; true when a baud change starts.
    fn advance(&mut self, now_us: u64) -> bool {
        match self.state {
            State::Listening {
                proto, until_us, ..
            } if now_us >= until_us => {
                self.state = State::Switching {
                    to: proto.other(),
                    step: Step::Flush,
                };
                true
            }
            State::Locked {
                proto,
                last_valid_us,
            } => {
                let deadline = last_valid_us.saturating_add(self.t_leave_us(proto));
                if now_us < deadline {
                    return false;
                }
                self.stats.mode = Mode::Searching;
                self.stats.mode_since_us = deadline;
                self.state = State::Switching {
                    to: proto.other(),
                    step: Step::Flush,
                };
                true
            }
            _ => false,
        }
    }

    /// A frame of `proto` that passed its decoder's checks; true when it is to be reported.
    fn valid_frame(&mut self, proto: Proto, now_us: u64) -> bool {
        match &mut self.state {
            State::Locked {
                proto: p,
                last_valid_us,
            } if *p == proto => {
                *last_valid_us = now_us;
                true
            }
            State::Listening {
                proto: p, frames, ..
            } if *p == proto => {
                *frames = frames.saturating_add(1);
                if *frames < self.lock_frames {
                    self.stats.lock_discarded = self.stats.lock_discarded.saturating_add(1);
                    return false;
                }
                self.state = State::Locked {
                    proto,
                    last_valid_us: now_us,
                };
                self.stats.mode = proto.mode();
                self.stats.mode_since_us = now_us;
                if self.last_locked.is_some_and(|last| last != proto) {
                    self.stats.mode_changes = self.stats.mode_changes.saturating_add(1);
                }
                self.last_locked = Some(proto);
                self.next_crsf_tx_us = now_us;
                true
            }
            _ => false,
        }
    }

    fn on_crsf(&mut self, frame: Frame<'_>, now_us: u64, hooks: &mut impl Hooks) {
        if !self.valid_frame(Proto::Crsf, now_us) {
            return;
        }
        match frame {
            Frame::RcChannels(rc) => {
                self.sticks(rc.raw.map(raw_to_us), Source::Crsf, now_us, hooks)
            }
            Frame::LinkStatistics(stats) => {
                self.stats.crsf_links = self.stats.crsf_links.wrapping_add(1);
                self.stats.last_crsf_link_us = Some(now_us);
                hooks.report(Report::CrsfLink(CrsfLinkReport {
                    stats,
                    at_us: now_us,
                    seq: self.stats.crsf_links,
                }));
            }
            _ => {}
        }
    }

    fn on_mavlink(&mut self, m: &Incoming<'_>, hooks: &mut impl Hooks) {
        if !self.valid_frame(Proto::Mavlink, m.now_us) {
            return;
        }
        if let Some(s) = m.decode::<RADIO_STATUS_DATA>() {
            self.stats.radio_status = self.stats.radio_status.wrapping_add(1);
            self.stats.last_radio_status_us = Some(m.now_us);
            hooks.report(Report::RadioStatus(RadioStatusReport {
                status: RadioStatus {
                    rssi: s.rssi,
                    remrssi: s.remrssi,
                    txbuf: s.txbuf,
                    noise: s.noise,
                    remnoise: s.remnoise,
                    rxerrors: s.rxerrors,
                    fixed: s.fixed,
                },
                system_id: m.system_id,
                component_id: m.component_id,
                at_us: m.now_us,
                seq: self.stats.radio_status,
            }));
        } else if let Some(rc) = m.decode::<RC_CHANNELS_OVERRIDE_DATA>() {
            if rc.target_system != self.own_system_id && rc.target_system != 0 {
                self.stats.overrides_not_for_us = self.stats.overrides_not_for_us.saturating_add(1);
                return;
            }
            let channels_us = [
                rc.chan1_raw,
                rc.chan2_raw,
                rc.chan3_raw,
                rc.chan4_raw,
                rc.chan5_raw,
                rc.chan6_raw,
                rc.chan7_raw,
                rc.chan8_raw,
                rc.chan9_raw,
                rc.chan10_raw,
                rc.chan11_raw,
                rc.chan12_raw,
                rc.chan13_raw,
                rc.chan14_raw,
                rc.chan15_raw,
                rc.chan16_raw,
            ];
            let source = Source::Mavlink {
                system_id: m.system_id,
                component_id: m.component_id,
            };
            self.sticks(channels_us, source, m.now_us, hooks);
        } else if !ACCEPT.iter().any(|(id, _)| *id == m.message_id) {
            hooks.mavlink(*m);
        }
    }

    fn sticks(
        &mut self,
        channels_us: [u16; 16],
        source: Source,
        now_us: u64,
        hooks: &mut impl Hooks,
    ) {
        self.stats.sticks = self.stats.sticks.wrapping_add(1);
        self.stats.last_sticks_us = Some(now_us);
        hooks.report(Report::Sticks(StickReport {
            channels_us,
            source,
            at_us: now_us,
            seq: self.stats.sticks,
        }));
    }
}

/// The port: owns the receiver's UART protocol state, a CRSF deframer and a MAVLink [`Node`].
///
/// Drive it with four inputs and one output, all with the caller's clock (`now_us`,
/// microseconds, non-decreasing):
/// - [`receive`](Self::receive) the bytes read from the UART; reports come out through
///   [`Hooks::report`] from inside the call;
/// - [`uart_error`](Self::uart_error) each read error;
/// - [`poll_action`](Self::poll_action) until it returns `None`, carrying out each [`Action`] and
///   acknowledging it with [`written`](Self::written) or [`done`](Self::done);
/// - then sleep until [`next_deadline_us`](Self::next_deadline_us) or the next bytes.
///
/// Each call first applies the deadline due at its `now_us` (leaving a lock, ending a listening
/// period), then its input. The first actions after [`new`](Self::new) set the UART to 460800
/// baud, whatever it was built with.
pub struct RcPort<'a> {
    fsm: Fsm,
    crsf: Deframer,
    switch_garbage: u32,
    node: Node<'a>,
    tx: [u8; FRAME_BUF],
    tx_len: usize,
    tx_off: usize,
}

impl<'a> RcPort<'a> {
    /// A port at power-up: searching, 460800 baud first.
    ///
    /// The node is built from `mav` and `streams` as [`Node::new`] does, with the accept list
    /// `mav.accept` (the application's, in its order) followed by RC_CHANNELS_OVERRIDE and
    /// RADIO_STATUS, the two messages the port reports, where the application does not list them.
    /// Where it does, the entry stays in its place with the port's CRC_EXTRA for that id. The
    /// application's other messages are handed to [`Hooks::mavlink`]. More than
    /// `mavlink_device::MAX_ACCEPT` (16) entries in all: [`NewError::AcceptListFull`], nothing
    /// dropped.
    pub fn new(
        config: Config,
        mav: mavlink_device::Config,
        streams: &'a mut [StreamSlot],
    ) -> Result<Self, NewError> {
        let own_system_id = mav.system_id;
        let own = |id: u32| ACCEPT.iter().copied().find(|p| p.0 == id);
        let added = ACCEPT.iter().copied().filter(|p| !mav.accept.contains(p.0));
        let mut pairs = [(0u32, 0u8); MAX_ACCEPT];
        let mut n = 0;
        for pair in mav
            .accept
            .iter()
            .map(|p| own(p.0).unwrap_or(p))
            .chain(added)
        {
            *pairs.get_mut(n).ok_or(NewError::AcceptListFull)? = pair;
            n += 1;
        }
        let mav = mav
            .with_accept_list(pairs.get(..n).unwrap_or(&[]))
            .map_err(|_| NewError::AcceptListFull)?;
        Ok(Self {
            fsm: Fsm {
                config,
                lock_frames: LOCK_FRAMES,
                own_system_id,
                state: State::Switching {
                    to: Proto::Mavlink,
                    step: Step::Flush,
                },
                last_locked: None,
                next_crsf_tx_us: 0,
                stats: Stats::default(),
            },
            crsf: Deframer::new(),
            switch_garbage: 0,
            node: Node::new(mav, streams),
            tx: [0; FRAME_BUF],
            tx_len: 0,
            tx_off: 0,
        })
    }

    /// Takes bytes read from the UART, in any split. While locked or listening they go to the
    /// decoder of the protocol of the current baud (CRSF: [`Deframer`]; MAVLink: the node's
    /// `receive_with`, so the node also acts on HEARTBEAT, parameter and command frames); during a
    /// baud change they are counted as garbage. Each valid frame counts toward the lock or keeps
    /// it; while locked, each RC, link statistics, RC_CHANNELS_OVERRIDE (target own system or 0)
    /// and RADIO_STATUS frame becomes one [`Report`], and each other listed MAVLink message one
    /// [`Hooks::mavlink`] call.
    ///
    /// A valid CRSF frame is any frame the deframer accepts. A valid MAVLink frame is one the
    /// node hands over through `receive_with`'s hook: a message of the accept list (the port's
    /// RC_CHANNELS_OVERRIDE and RADIO_STATUS and the application's own entries), MAVLink 2,
    /// incompatibility flags 0, checksum right for the listed CRC_EXTRA, sender not the node's own
    /// system id (HEARTBEAT, parameter and command frames do not count).
    ///
    /// `params` is written only while locked on MAVLink: before the lock (and from frames that
    /// come before the locking frame in the same call) the node gets a view of `params` whose
    /// `set` returns `ParamError::Rejected`, so a PARAM_SET heard while listening at 460800
    /// changes nothing (the node queues its answer, the value unchanged; it is written only if
    /// the port locks on MAVLink before the next baud change drops it).
    pub fn receive(
        &mut self,
        bytes: &[u8],
        now_us: u64,
        params: &mut impl ParamStore,
        hooks: &mut impl Hooks,
    ) {
        self.advance(now_us);
        let proto = match self.fsm.state {
            State::Switching { .. } => {
                let n = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
                self.switch_garbage = self.switch_garbage.saturating_add(n);
                return;
            }
            State::Listening { proto, .. } | State::Locked { proto, .. } => proto,
        };
        let fsm = &mut self.fsm;
        match proto {
            Proto::Crsf => self
                .crsf
                .push(bytes, |frame| fsm.on_crsf(frame, now_us, hooks)),
            Proto::Mavlink => {
                let open = Cell::new(fsm.locked_on(Proto::Mavlink));
                let mut params = Gate {
                    params,
                    open: &open,
                };
                self.node
                    .receive_with(bytes, now_us, &mut params, &mut |m| {
                        fsm.on_mavlink(&m, hooks);
                        open.set(fsm.locked_on(Proto::Mavlink));
                    });
            }
        }
    }

    /// Counts a read error. Apart from the deadline due at `now_us`, which every call applies, it
    /// changes nothing: an error neither keeps a lock nor ends one.
    pub fn uart_error(&mut self, error: UartError, now_us: u64) {
        self.advance(now_us);
        let s = &mut self.fsm.stats;
        let counter = match error {
            UartError::Framing => &mut s.uart_framing,
            UartError::Parity => &mut s.uart_parity,
            UartError::Break => &mut s.uart_break,
            UartError::Overrun => &mut s.uart_overrun,
            UartError::Other => &mut s.uart_other,
        };
        *counter = counter.saturating_add(1);
    }

    /// The next action, or `None` when there is nothing to do until
    /// [`next_deadline_us`](Self::next_deadline_us) or new bytes.
    ///
    /// Writes come only while locked and only in the locked protocol: CRSF, one frame from
    /// [`Hooks::crsf_frame`] at most every [`Config::crsf_tx_interval_us`]; MAVLink, the node's
    /// `poll_transmit` frames. While searching nothing is written; when a lock is left, the unsent
    /// rest of a frame is dropped and the baud change begins.
    pub fn poll_action(
        &mut self,
        now_us: u64,
        params: &impl ParamStore,
        tel: &mut impl Telemetry,
        hooks: &mut impl Hooks,
    ) -> Option<Action<'_>> {
        self.advance(now_us);
        let proto = match self.fsm.state {
            State::Switching { to, step } => {
                return Some(match step {
                    Step::Flush => Action::Flush,
                    Step::WaitTxIdle => Action::WaitTxIdle,
                    Step::SetBaud => Action::SetBaud(self.fsm.baud(to)),
                });
            }
            State::Listening { .. } => return None,
            State::Locked { proto, .. } => proto,
        };
        if self.tx_off >= self.tx_len {
            self.tx_off = 0;
            self.tx_len = 0;
            match proto {
                Proto::Crsf if now_us >= self.fsm.next_crsf_tx_us => {
                    let interval = u64::from(self.fsm.config.crsf_tx_interval_us.max(1));
                    self.fsm.next_crsf_tx_us = now_us.saturating_add(interval);
                    let mut frame = [0u8; MAX_FRAME_LEN];
                    if let Some(n) = hooks.crsf_frame(now_us, &mut frame)
                        && let (Some(dst), Some(src)) = (self.tx.get_mut(..n), frame.get(..n))
                    {
                        dst.copy_from_slice(src);
                        self.tx_len = n;
                    }
                }
                Proto::Crsf => {}
                Proto::Mavlink => {
                    if let Some(n) = self.node.poll_transmit(now_us, params, tel, &mut self.tx) {
                        self.tx_len = n;
                    }
                }
            }
        }
        self.tx
            .get(self.tx_off..self.tx_len)
            .filter(|rest| !rest.is_empty())
            .map(Action::Write)
    }

    /// After an [`Action::Write`]: the UART took `n` of its bytes.
    pub fn written(&mut self, n: usize) {
        self.tx_off = self.tx_off.saturating_add(n).min(self.tx_len);
    }

    /// After [`Action::Flush`], [`Action::WaitTxIdle`] or [`Action::SetBaud`]: it is done. After
    /// the baud is set, both decoders' buffers are cleared (`Deframer::clear` for CRSF, which counts
    /// the bytes it held as garbage; `Node::on_link_down` for MAVLink) and listening begins, for
    /// [`Config::d_mav_us`] or [`Config::d_crsf_us`] from `now_us`. Ignored when no such action
    /// is pending.
    pub fn done(&mut self, now_us: u64) {
        let State::Switching { to, step } = self.fsm.state else {
            return;
        };
        self.fsm.state = match step {
            Step::Flush => State::Switching {
                to,
                step: Step::WaitTxIdle,
            },
            Step::WaitTxIdle => State::Switching {
                to,
                step: Step::SetBaud,
            },
            Step::SetBaud => {
                self.crsf.clear();
                self.node.on_link_down();
                State::Listening {
                    proto: to,
                    frames: 0,
                    until_us: now_us.saturating_add(self.fsm.dwell_us(to)),
                }
            }
        };
    }

    /// The earliest time at which [`poll_action`](Self::poll_action) could have something new
    /// without new bytes: 0 while an action is pending; else the end of the listening period, or
    /// the earliest of the lock's leave deadline and the next CRSF frame or node deadline.
    pub fn next_deadline_us(&self) -> u64 {
        if self.tx_off < self.tx_len {
            return 0;
        }
        match self.fsm.state {
            State::Switching { .. } => 0,
            State::Listening { until_us, .. } => until_us,
            State::Locked {
                proto,
                last_valid_us,
            } => {
                let leave = last_valid_us.saturating_add(self.fsm.t_leave_us(proto));
                let other = match proto {
                    Proto::Crsf => self.fsm.next_crsf_tx_us,
                    Proto::Mavlink => self.node.next_deadline_us(),
                };
                leave.min(other)
            }
        }
    }

    /// The counters.
    pub fn stats(&self) -> Stats {
        let crsf = self.crsf.stats();
        let mav = self.node.stats();
        let mut s = self.fsm.stats;
        s.crc_errors = crsf
            .crc_errors
            .saturating_add(mav.dropped_checksum)
            .saturating_add(mav.dropped_listed_checksum);
        s.garbage_bytes = crsf
            .garbage_bytes
            .saturating_add(mav.garbage_bytes)
            .saturating_add(self.switch_garbage);
        s
    }

    /// Counts one idle-wait check of `run` that found the transmitter busy.
    pub(crate) fn count_tx_busy(&mut self) {
        let n = &mut self.fsm.stats.tx_busy_polls;
        *n = n.saturating_add(1);
    }

    /// The MAVLink node, for its counters, `gcs_present` and `stream_slowdown_ms`.
    pub fn node(&self) -> &Node<'a> {
        &self.node
    }

    fn advance(&mut self, now_us: u64) {
        if self.fsm.advance(now_us) {
            self.tx_len = 0;
            self.tx_off = 0;
        }
    }

    /// Test hook: the number of valid frames that lock (the I16 negative control sets 1).
    #[cfg(test)]
    pub(crate) fn set_lock_frames(&mut self, n: u8) {
        self.fsm.lock_frames = n;
    }
}
