//! Host tests of the async `RcPort::run`: a fake UART on the models of `sim` (the wire resampled at
//! the UART's baud, a transmitter that drains at it), a fake delay, one made-up clock, and the
//! future polled by hand with a no-op waker, the clock moved 20 us whenever it is pending.

use crate::sim::*;
use crate::*;
use crsf_codec::raw_to_us;
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorKind, ErrorType, Read, Write};
use std::cell::Cell;
use std::collections::VecDeque;
use std::future::{Future, poll_fn};
use std::pin::pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};
use std::vec::Vec;

/// Picoseconds per millisecond.
const MS: u64 = 1_000_000_000;
/// How far the clock moves while the future is pending.
const STEP_PS: u64 = 20 * US;

type Clock = Rc<Cell<u64>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FakeErr(UartError);

impl core::fmt::Display for FakeErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl core::error::Error for FakeErr {}

impl embedded_io_async::Error for FakeErr {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl From<FakeErr> for UartError {
    fn from(e: FakeErr) -> Self {
        e.0
    }
}

/// What the fake UART saw, time in us.
#[derive(Clone, Debug, PartialEq, Eq)]
enum UEv {
    Write {
        t_us: u64,
        n: usize,
    },
    FlushDone {
        t_us: u64,
    },
    Busy {
        t_us: u64,
        busy: bool,
    },
    SetBaud {
        t_us: u64,
        baud: u32,
        flushed: bool,
        idle: bool,
    },
}

struct FakeUart {
    clock: Clock,
    wire: Wire,
    rx: Receiver,
    ready: VecDeque<Rx>,
    tx: Transmitter,
    trace: Vec<UEv>,
    /// Reads dropped while pending.
    reads_dropped: Rc<Cell<u32>>,
    /// The transmitter drains at the baud rate divided by this (1: the real rate).
    tx_div: u32,
}

impl FakeUart {
    fn new(clock: Clock, wire: Wire) -> Self {
        Self {
            clock,
            wire,
            rx: Receiver::new(115_200),
            ready: VecDeque::new(),
            tx: Transmitter {
                baud: 115_200,
                idle_at: 0,
            },
            trace: Vec::new(),
            reads_dropped: Rc::new(Cell::new(0)),
            tx_div: 1,
        }
    }

    fn now_us(&self) -> u64 {
        self.clock.get() / US
    }

    fn flushed(&self) -> bool {
        self.tx.flushed_at(self.clock.get()) <= self.clock.get()
    }
}

/// Counts a read dropped before it returned.
struct Guard(Rc<Cell<u32>>, bool);

impl Drop for Guard {
    fn drop(&mut self) {
        if !self.1 {
            self.0.set(self.0.get() + 1);
        }
    }
}

impl ErrorType for FakeUart {
    type Error = FakeErr;
}

impl Read for FakeUart {
    /// Like embassy-rp's `BufferedUart::read`: bytes when there are any (up to the first error),
    /// the error when it comes first; cancel-safe (nothing is taken unless it returns).
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, FakeErr> {
        let mut guard = Guard(self.reads_dropped.clone(), false);
        let out = poll_fn(|_| {
            let now = self.clock.get();
            self.rx.advance(&self.wire, now);
            self.ready
                .extend(self.rx.take(now).into_iter().map(|x| x.1));
            if let Some(Rx::Err(e)) = self.ready.front().copied() {
                self.ready.pop_front();
                return Poll::Ready(Err(FakeErr(e)));
            }
            let mut n = 0;
            while n < buf.len() {
                let Some(Rx::Byte(b)) = self.ready.front().copied() else {
                    break;
                };
                buf[n] = b;
                n += 1;
                self.ready.pop_front();
            }
            if n == 0 {
                Poll::Pending
            } else {
                Poll::Ready(Ok(n))
            }
        })
        .await;
        guard.1 = true;
        out
    }
}

impl Write for FakeUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, FakeErr> {
        self.tx.write(self.clock.get(), buf.len());
        self.trace.push(UEv::Write {
            t_us: self.now_us(),
            n: buf.len(),
        });
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), FakeErr> {
        poll_fn(|_| {
            if self.flushed() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        self.trace.push(UEv::FlushDone {
            t_us: self.now_us(),
        });
        Ok(())
    }
}

impl SetBaud for FakeUart {
    /// Like embassy-rp: the receiver restarts at the new rate and the call takes 15 bit periods.
    fn set_baud(&mut self, baud: u32) {
        let now = self.clock.get();
        self.trace.push(UEv::SetBaud {
            t_us: now / US,
            baud,
            flushed: self.flushed(),
            idle: self.tx.idle(now),
        });
        self.rx.set_baud(&self.wire, now, baud);
        self.tx.baud = baud / self.tx_div;
        self.clock.set(now + 15 * PS / u64::from(baud));
    }
}

impl TxBusy for FakeUart {
    fn tx_busy(&self) -> bool {
        !self.tx.idle(self.clock.get())
    }
}

/// A wrapper that records the busy checks (`TxBusy::tx_busy` takes `&self`).
struct Recorded {
    uart: FakeUart,
    busy_log: std::cell::RefCell<Vec<UEv>>,
}

impl ErrorType for Recorded {
    type Error = FakeErr;
}

impl Read for Recorded {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, FakeErr> {
        self.uart.read(buf).await
    }
}

impl Write for Recorded {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, FakeErr> {
        self.merge();
        self.uart.write(buf).await
    }
    async fn flush(&mut self) -> Result<(), FakeErr> {
        self.merge();
        self.uart.flush().await
    }
}

impl SetBaud for Recorded {
    fn set_baud(&mut self, baud: u32) {
        self.merge();
        self.uart.set_baud(baud);
    }
}

impl TxBusy for Recorded {
    fn tx_busy(&self) -> bool {
        let busy = self.uart.tx_busy();
        self.busy_log.borrow_mut().push(UEv::Busy {
            t_us: self.uart.now_us(),
            busy,
        });
        busy
    }
}

impl Recorded {
    /// Moves the busy checks into the trace, in order.
    fn merge(&mut self) {
        let log = core::mem::take(&mut *self.busy_log.borrow_mut());
        self.uart.trace.extend(log);
    }
}

struct FakeDelay {
    clock: Clock,
}

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
        let target = self.clock.get() + u64::from(ns) * 1_000;
        poll_fn(|_| {
            if self.clock.get() >= target {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

const CRSF_RAW: [u16; 16] = [
    172, 992, 1811, 992, 1500, 300, 992, 992, 992, 992, 992, 992, 992, 992, 1100, 1700,
];
const MAV_CH: [u16; 18] = [
    988, 1500, 2012, 1500, 1820, 1065, 1500, 1500, 1500, 1500, 1500, 1500, 0, 65535, 1501, 1999, 0,
    0,
];

/// CRSF at 250 Hz for 1.2 s, 100 ms of silence, MAVLink for 1.5 s, 100 ms of silence, CRSF for
/// 1.2 s; returns the wire and the end.
fn three_segments() -> (Wire, u64) {
    let mut wire = Wire::default();
    let seg_crsf = |wire: &mut Wire, from: u64| {
        let mut t = from;
        let mut k = 0;
        while t < from + 1_200 * MS {
            wire.send(t, CRSF_BAUD, &crsf_rc(&CRSF_RAW));
            if k % 25 == 0 {
                wire.send(
                    t + 2 * MS,
                    CRSF_BAUD,
                    &crsf_link([60, 61, 100, 5, 0, 7, 3, 70, 99, 8]),
                );
            }
            t += 4 * MS;
            k += 1;
        }
        wire.end()
    };
    let end = seg_crsf(&mut wire, 20 * MS);
    let mut t = end + 100 * MS;
    let mav_start = t;
    let mut seq = 0u8;
    while t < mav_start + 1_500 * MS {
        seq = seq.wrapping_add(1);
        wire.send(
            t,
            MAVLINK_BAUD,
            &mav(ELRS_SYS, ELRS_COMP, seq, &radio_status(200, 100)),
        );
        if ((t - mav_start) / (10 * MS)).is_multiple_of(2) {
            wire.send(
                t + 5 * MS,
                MAVLINK_BAUD,
                &mav(ELRS_SYS, ELRS_COMP, seq, &overrides(OWN_SYS, &MAV_CH)),
            );
        }
        t += 10 * MS;
    }
    let from = wire.end() + 100 * MS;
    let end = seg_crsf(&mut wire, from);
    (wire, end)
}

struct Outcome {
    reports: Vec<Report>,
    last_stats_hook: Option<Stats>,
    tx_busy_seen: Vec<u32>,
    stats: Stats,
    trace: Vec<UEv>,
    returned: bool,
    reads_dropped: u32,
    futures_dropped: u32,
}

/// Runs `run` on the three segments until `end` + 50 ms. With `drop_every`, the `run` future is
/// dropped after every few polls (1 to `drop_every`, pseudo-random) and a new one is started.
fn drive(config: Config, drop_every: Option<u64>, tx_div: u32) -> Outcome {
    let (wire, end) = three_segments();
    let clock: Clock = Rc::new(Cell::new(0));
    let mut uart = Recorded {
        uart: FakeUart::new(clock.clone(), wire),
        busy_log: std::cell::RefCell::new(Vec::new()),
    };
    uart.uart.tx_div = tx_div;
    let reads_dropped = uart.uart.reads_dropped.clone();
    let mut delay = FakeDelay {
        clock: clock.clone(),
    };
    let mut slots = slots();
    let mut port =
        RcPort::new(config, mavlink_device::Config::new(OWN_SYS, 1), &mut slots).unwrap();
    let mut hooks = Rec::default();
    let mut params = NoParams;
    let mut tel = Tel;
    let mut cx = Context::from_waker(Waker::noop());
    let mut returned = false;
    let mut futures_dropped = 0;
    let mut lcg = 12345u64;
    'outer: while clock.get() < end + 50 * MS {
        let c = clock.clone();
        let fut = port.run(
            &mut uart,
            &mut delay,
            move || c.get() / US,
            &mut params,
            &mut tel,
            &mut hooks,
        );
        let mut fut = pin!(fut);
        let mut polls_left = drop_every.map(|n| {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            1 + (lcg >> 33) % n
        });
        loop {
            if fut.as_mut().poll(&mut cx).is_ready() {
                returned = true;
                break 'outer;
            }
            clock.set(clock.get() + STEP_PS);
            if clock.get() >= end + 50 * MS {
                break 'outer;
            }
            if let Some(left) = polls_left.as_mut() {
                *left -= 1;
                if *left == 0 {
                    futures_dropped += 1;
                    break;
                }
            }
        }
    }
    uart.merge();
    Outcome {
        reports: hooks.reports.clone(),
        last_stats_hook: hooks.last_stats,
        tx_busy_seen: hooks.tx_busy_seen.clone(),
        stats: port.stats(),
        trace: uart.uart.trace.clone(),
        returned,
        reads_dropped: reads_dropped.get(),
        futures_dropped,
    }
}

fn check_mode_following(o: &Outcome) {
    assert!(!o.returned, "run returned");
    assert_eq!(o.stats.mode, Mode::Crsf);
    assert_eq!(o.stats.mode_changes, 2);
    assert_eq!(
        o.last_stats_hook,
        Some(o.stats),
        "the stats hook has the latest counters"
    );
    assert!(
        o.stats.uart_framing > 0,
        "read errors at the wrong baud were counted and run went on"
    );
    let mut sources = Vec::new();
    for r in &o.reports {
        if let Report::Sticks(s) = r {
            match s.source {
                Source::Crsf => assert_eq!(s.channels_us, CRSF_RAW.map(raw_to_us)),
                Source::Mavlink {
                    system_id,
                    component_id,
                } => {
                    assert_eq!((system_id, component_id), (ELRS_SYS, ELRS_COMP));
                    assert_eq!(s.channels_us[..], MAV_CH[..16]);
                }
            }
            if sources.last() != Some(&s.source) {
                sources.push(s.source);
            }
        }
    }
    let mav = Source::Mavlink {
        system_id: ELRS_SYS,
        component_id: ELRS_COMP,
    };
    assert_eq!(sources, [Source::Crsf, mav, Source::Crsf]);
    // Each CRSF segment is reported from its lock on (about 290 and 275 frames); the MAVLink
    // segment only from about 1 s after the CRSF frames stopped (T_LEAVE_CRSF, 100 ms of it in
    // the gap), so about 23 overrides of its 75.
    let mavlink = o
        .reports
        .iter()
        .filter(|r| matches!(r, Report::Sticks(s) if s.source == mav))
        .count();
    assert!(
        o.stats.sticks > 540 && mavlink >= 5,
        "{} sticks, {mavlink} from MAVLink",
        o.stats.sticks
    );
}

/// The async front end follows CRSF -> MAVLink -> CRSF on the fake UART: two mode changes, the
/// right stick values from each protocol, read errors counted without ending `run`, the stats
/// hook up to date.
#[test]
fn run_follows_crsf_to_mavlink_and_back() {
    let o = drive(Config::new(), None, 1);
    check_mode_following(&o);
    let writes = o
        .trace
        .iter()
        .filter(|e| matches!(e, UEv::Write { .. }))
        .count();
    assert!(writes > 20, "writes {writes}");
}

/// Before every baud change `run` flushed, then checked `tx_busy` until it was false, then set the
/// baud, with no write in between; the fake UART was flushed and idle at every `set_baud`. Here a
/// CRSF frame is queued every 1 ms and the fake transmitter drains at a quarter of the baud rate
/// (a 12-byte frame takes 1.14 ms), so a backlog builds up and leaving CRSF finds the transmitter
/// busy: the flush and the idle wait are exercised.
#[test]
fn run_flushes_then_waits_tx_idle_then_sets_baud() {
    let o = drive(Config::new().with_crsf_tx_interval_us(1_000), None, 4);
    check_mode_following(&o);
    let mut flushed = false;
    let mut idle_seen = false;
    let mut waited = 0;
    let mut sets = Vec::new();
    for e in &o.trace {
        match e {
            UEv::Write { .. } => {
                flushed = false;
                idle_seen = false;
            }
            UEv::FlushDone { .. } => flushed = true,
            UEv::Busy { busy, .. } => {
                assert!(flushed, "busy checked before the flush");
                if *busy {
                    waited += 1;
                }
                idle_seen = !busy;
            }
            UEv::SetBaud {
                flushed: f,
                idle,
                baud,
                ..
            } => {
                assert!(
                    flushed && idle_seen,
                    "set_baud without flush and an idle check: {e:?}"
                );
                assert!(*f && *idle, "the transmitter was busy at set_baud: {e:?}");
                sets.push(*baud);
                flushed = false;
                idle_seen = false;
            }
        }
    }
    // Power-up, the end of the first 460800 period, leaving CRSF, leaving MAVLink.
    assert_eq!(sets, [MAVLINK_BAUD, CRSF_BAUD, MAVLINK_BAUD, CRSF_BAUD]);
    assert!(
        waited > 0,
        "no baud change found the transmitter busy: the wait was not exercised"
    );
}

/// CONTRACT A2: dropping the `run` future at any await point (after 1 to 5 polls, thousands of
/// times) and calling `run` again loses nothing: the same reports, counters and mode following
/// as an undisturbed run, and the ordering before every baud change holds.
#[test]
fn run_dropped_at_any_await_loses_nothing() {
    let calm = drive(Config::new(), None, 1);
    let o = drive(Config::new(), Some(5), 1);
    assert!(
        o.futures_dropped > 10_000,
        "futures dropped {}",
        o.futures_dropped
    );
    assert!(
        o.reads_dropped > 10_000,
        "reads dropped {}",
        o.reads_dropped
    );
    check_mode_following(&o);
    assert_eq!(o.reports, calm.reports);
    assert_eq!(o.stats, calm.stats);
    for e in &o.trace {
        if let UEv::SetBaud { flushed, idle, .. } = e {
            assert!(*flushed && *idle);
        }
    }
}

/// CONTRACT A5: the `run` future is `Send` when the UART, the delay and the hooks are.
#[test]
fn run_future_is_send() {
    struct U;
    impl ErrorType for U {
        type Error = FakeErr;
    }
    impl Read for U {
        async fn read(&mut self, _: &mut [u8]) -> Result<usize, FakeErr> {
            Ok(0)
        }
    }
    impl Write for U {
        async fn write(&mut self, b: &[u8]) -> Result<usize, FakeErr> {
            Ok(b.len())
        }
        async fn flush(&mut self) -> Result<(), FakeErr> {
            Ok(())
        }
    }
    impl SetBaud for U {
        fn set_baud(&mut self, _: u32) {}
    }
    impl TxBusy for U {
        fn tx_busy(&self) -> bool {
            false
        }
    }
    struct D;
    impl DelayNs for D {
        async fn delay_ns(&mut self, _: u32) {}
    }
    fn assert_send<T: Send>(_: &T) {}
    let mut slots = slots();
    let mut port = RcPort::new(
        Config::new(),
        mavlink_device::Config::new(OWN_SYS, 1),
        &mut slots,
    )
    .unwrap();
    let (mut u, mut d, mut p, mut t, mut h) = (U, D, NoParams, Tel, Rec::default());
    let fut = port.run(&mut u, &mut d, || 0, &mut p, &mut t, &mut h);
    assert_send(&fut);
}

/// `Stats::tx_busy_polls` counts exactly the idle-wait checks that found the transmitter busy,
/// and the stats hook sees it rise during the wait: with the slowed fake transmitter (as above)
/// the hook saw every value from 0 to the number of busy answers in the trace (more than 0), one
/// step per busy check, and it stops there: the idle answers that ended each wait are not
/// counted.
#[test]
fn run_counts_idle_wait_polls_that_found_the_transmitter_busy() {
    let o = drive(Config::new().with_crsf_tx_interval_us(1_000), None, 4);
    let busy = |b: bool| {
        o.trace
            .iter()
            .filter(|e| matches!(e, UEv::Busy { busy, .. } if *busy == b))
            .count()
    };
    assert!(busy(true) > 0, "the wait was not exercised");
    assert_eq!(busy(false), 4, "one idle answer per baud change");
    assert_eq!(o.stats.tx_busy_polls as usize, busy(true));
    let mut seen = o.tx_busy_seen.clone();
    seen.dedup();
    assert_eq!(seen, (0..=o.stats.tx_busy_polls).collect::<Vec<u32>>());
    assert_eq!(
        o.last_stats_hook.map(|s| s.tx_busy_polls),
        Some(o.stats.tx_busy_polls)
    );
}

/// `Stats::stream_slowdown_ms` is the node's slowdown and reaches `Hooks::stats`: RADIO_STATUS
/// every 10 ms with `txbuf` 100 (the three that lock), then 10, 10, 30, 49, 50, 19 move it by
/// ArduPilot's steps (+60 below 20, +20 below 50, 50 unchanged) to 0, 60, 120, 140, 160, 220; when
/// the frames stop, the lock is left and the baud change (`Node::on_link_down`) sets it to 0.
#[test]
fn stats_carry_the_node_stream_slowdown_to_the_hook() {
    const TXBUF: [u8; 9] = [100, 100, 100, 10, 10, 30, 49, 50, 19];
    let mut wire = Wire::default();
    for (k, txbuf) in TXBUF.iter().enumerate() {
        let t = (20 + 10 * k as u64) * MS;
        let status = radio_status(200, *txbuf);
        wire.send(t, MAVLINK_BAUD, &mav(ELRS_SYS, ELRS_COMP, k as u8, &status));
    }
    let end = wire.end() + 300 * MS;
    let clock: Clock = Rc::new(Cell::new(0));
    let mut uart = FakeUart::new(clock.clone(), wire);
    let mut delay = FakeDelay {
        clock: clock.clone(),
    };
    let mut slots = slots();
    let mut port = RcPort::new(
        Config::new(),
        mavlink_device::Config::new(OWN_SYS, 1),
        &mut slots,
    )
    .unwrap();
    let (mut hooks, mut params, mut tel) = (Rec::default(), NoParams, Tel);
    {
        let c = clock.clone();
        let mut fut = pin!(port.run(
            &mut uart,
            &mut delay,
            move || c.get() / US,
            &mut params,
            &mut tel,
            &mut hooks,
        ));
        let mut cx = Context::from_waker(Waker::noop());
        while clock.get() < end {
            assert!(fut.as_mut().poll(&mut cx).is_pending(), "run returned");
            clock.set(clock.get() + STEP_PS);
        }
    }
    let mut seen = hooks.slowdown_seen.clone();
    seen.dedup();
    assert_eq!(seen, [0, 60, 120, 140, 160, 220, 0]);
    let s = port.stats();
    assert_eq!(s.radio_status, 7, "frames 3 to 9 reported");
    assert_eq!(s.mode, Mode::Searching);
    assert_eq!(s.mode_changes, 0);
    assert_eq!(s.stream_slowdown_ms, 0);
    assert_eq!(port.node().stream_slowdown_ms(), 0);
    assert_eq!(hooks.last_stats, Some(s));
}
