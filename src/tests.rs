//! Host tests of the sans-IO core on the model of `sim` (wire, resampling UART, transmitter, made-up
//! clock). The numbered ones (I13, I14, I16, I17, I18) are the invariants of the design decision;
//! the others are the scenarios of the brief.
//!
//! The lock bound proved here, in the model, for one change from protocol A to protocol B (t_last
//! = end of the last A frame on the wire, G = gap from there to the start of the first B frame):
//!
//!   lock - t_last <= max(T_LEAVE_A, G) + D_A + 5 P + 2 d + 2 eps + 3 w
//!
//! D_A is the listening time at A's baud (the dwell that is NOT B's), P the longest interval
//! between the starts of consecutive valid B frames, d the longest B frame, eps the longest baud
//! change (flush, transmitter drain, 15 bit periods) and w the longest time from the end of a
//! frame to the `receive` call that hands it over (the wake period). Why: the A lock is left at
//! the latest at t_last + w + T_LEAVE_A and the search starts at B's baud (the order is "the
//! other baud first"); a listening period at B's baud that starts while B frames flow locks
//! within 3 P + d + w (the first whole frame starts within P, the third ends 2 P + d later, and
//! it is handed over within w), which fits in D_B. If the B frames start while the search is
//! already running, they can arrive at the end of a B period with too little of it left (less
//! than 2 P + d + w) and the lock waits for one A period and two baud changes: 2 P + d + w +
//! 2 eps + D_A + 3 P + d + w. The worst of the cases is the formula. It needs 3 P + d + w <= D_B
//! (MAVLink: 30 + 1.1 + 1 <= 50 ms; CRSF at 50 Hz: 60 + 0.6 + 1 <= 200 ms).
//!
//! With the defaults, MAVLink as B (P = 10 ms, the RADIO_STATUS period; d = 1.09 ms, a 50-byte
//! RC_CHANNELS_OVERRIDE) and D_A = D_CRSF = 200 ms: max(T_LEAVE_A, G) + 252.2 ms + 2 eps + 3 w,
//! under the brief's max(T_LEAVE_old, gap) + 280 ms while 2 eps + 3 w < 27.8 ms. CRSF as B at
//! 50 Hz (P = 20 ms, d = 0.62 ms) with D_A = D_MAV = 50 ms: max(...) + 151.3 ms + 2 eps + 3 w.
//! When the B frames already flow as the A lock is left (G <= T_LEAVE_A), the first listening
//! period is at B's baud and the bound is sharper: lock - t_last <= T_LEAVE_A + eps + 3 P + d +
//! 2 w (checked too; it is what pins the search order).
//! Re-lock after a total loss in CRSF mode (frames return at t_r while searching): the same
//! argument without the leave, relock - t_r <= D_MAV + 5 P + 2 d + 2 eps + 2 w, which is the
//! decision's "50 ms plus 3 frames" plus 2 P + d + w: the frames can return at the end of a CRSF
//! period, where a lock-on that is not finished is cut by the next baud change.

use crate::sim::*;
use crate::*;
use crsf_codec::{Deframer, encode_battery, raw_to_us};
use proptest::prelude::*;
use proptest::test_runner::{Config as PtConfig, TestCaseError, TestRunner};
use std::format;
use std::vec;
use std::vec::Vec;

/// Picoseconds per millisecond.
const MS: u64 = 1_000_000_000;

/// Duration in us (rounded up) of `len` bytes at `baud`.
fn bytes_us(len: usize, baud: u32) -> u64 {
    (len as u64 * 10 * 1_000_000).div_ceil(u64::from(baud))
}

/// What a frame on the wire must give.
#[derive(Clone, Debug, PartialEq, Eq)]
enum What {
    /// This stick report.
    Sticks([u16; 16], Source),
    Link,
    Radio,
    /// Nothing ever: a corrupted frame or an override for another system.
    Never([u16; 16]),
}

#[derive(Clone, Debug)]
struct Sent {
    start: u64,
    end: u64,
    what: What,
    bytes: Vec<u8>,
}

/// A scenario: the wire and every frame put on it.
#[derive(Default)]
struct Scen {
    wire: Wire,
    sent: Vec<Sent>,
    seq: u8,
}

const LINK_FIELDS: [u8; 10] = [60, 61, 100, 5, 0, 7, 3, 70, 99, 8];

impl Scen {
    fn put(&mut self, at: u64, baud: u32, bytes: &[u8], what: What) -> u64 {
        let start = at.max(self.wire.end());
        let end = self.wire.send(start, baud, bytes);
        self.sent.push(Sent {
            start,
            end,
            what,
            bytes: bytes.to_vec(),
        });
        end
    }

    fn crsf_rc(&mut self, at: u64, raw: &[u16; 16]) -> u64 {
        let what = What::Sticks(raw.map(raw_to_us), Source::Crsf);
        self.put(at, CRSF_BAUD, &crsf_rc(raw), what)
    }

    fn crsf_link(&mut self, at: u64) -> u64 {
        self.put(at, CRSF_BAUD, &crsf_link(LINK_FIELDS), What::Link)
    }

    fn mav_override(&mut self, at: u64, target: u8, ch: &[u16; 18]) -> u64 {
        self.seq = self.seq.wrapping_add(1);
        let bytes = mav(ELRS_SYS, ELRS_COMP, self.seq, &overrides(target, ch));
        let mut first = [0u16; 16];
        first.copy_from_slice(&ch[..16]);
        let what = if target == OWN_SYS || target == 0 {
            What::Sticks(
                first,
                Source::Mavlink {
                    system_id: ELRS_SYS,
                    component_id: ELRS_COMP,
                },
            )
        } else {
            What::Never(first)
        };
        self.put(at, MAVLINK_BAUD, &bytes, what)
    }

    fn mav_radio(&mut self, at: u64) -> u64 {
        self.seq = self.seq.wrapping_add(1);
        let bytes = mav(ELRS_SYS, ELRS_COMP, self.seq, &radio_status(200, 100));
        self.put(at, MAVLINK_BAUD, &bytes, What::Radio)
    }

    fn garbage(&mut self, at: u64, baud: u32, bytes: &[u8]) -> u64 {
        self.wire.send(at, baud, bytes)
    }

    /// CRSF as ExpressLRS sends it: RC every `period_us`, link statistics every 100 ms between
    /// two RC frames. Returns the end of the last frame.
    fn crsf_stream(&mut self, from: u64, to: u64, period_us: u64, raw: &[u16; 16]) -> u64 {
        let mut t = from;
        let mut next_link = from;
        let mut end = from;
        while t < to {
            end = self.crsf_rc(t, raw);
            if t >= next_link {
                end = self.crsf_link(t + period_us * US / 2);
                next_link += 100 * MS;
            }
            t += period_us * US;
        }
        end
    }

    /// MAVLink as ExpressLRS sends it: RADIO_STATUS every 10 ms and, when `ch` is given,
    /// RC_CHANNELS_OVERRIDE every 20 ms, 5 ms after a RADIO_STATUS. Returns the end of the last
    /// frame.
    fn mav_stream(&mut self, from: u64, to: u64, ch: Option<&[u16; 18]>) -> u64 {
        let mut t = from;
        let mut k = 0u64;
        let mut end = from;
        while t < to {
            end = self.mav_radio(t);
            if let Some(ch) = ch
                && k.is_multiple_of(2)
            {
                end = self.mav_override(t + 5 * MS, OWN_SYS, ch);
            }
            t += 10 * MS;
            k += 1;
        }
        end
    }

    fn sticks(&self) -> Vec<&Sent> {
        self.sent
            .iter()
            .filter(|s| matches!(s.what, What::Sticks(..)))
            .collect()
    }
}

/// Runs `f` on a fresh model whose wire is `wire`.
fn with_sim<R>(config: Config, wire: Wire, f: impl FnOnce(&mut Sim<'_>) -> R) -> R {
    let mut slots = slots();
    let mut sim = Sim::new(config, &mut slots);
    sim.wire = wire;
    f(&mut sim)
}

/// I13's check: every stick report equals the expected report of exactly one valid frame of the
/// input, frames in order and each at most once, handed over after the frame ended.
fn check_reports_come_from_frames(
    sticks: &[StickReport],
    sent: &[Sent],
) -> Result<(), TestCaseError> {
    let mut next = 0;
    for r in sticks {
        let wanted = What::Sticks(r.channels_us, r.source);
        let Some(p) = sent[next..].iter().position(|s| s.what == wanted) else {
            return Err(TestCaseError::fail(format!(
                "report {r:?} is no later valid frame of the input"
            )));
        };
        let s = &sent[next + p];
        // The last character is handed over at its stop bit's centre (half a bit before the
        // frame's end) and now_us is floored.
        let half_bit = PS / (2 * u64::from(CRSF_BAUD));
        prop_assert!(
            r.at_us * US + US + half_bit >= s.end,
            "report at {} us before its frame ended at {} ps",
            r.at_us,
            s.end
        );
        next += p + 1;
    }
    Ok(())
}

/// The longest baud change in the trace, from the first action to the end of the receiver's
/// 15 bit periods, in us.
fn eps_us(trace: &[Ev]) -> u64 {
    let mut longest = 0;
    let mut started = None;
    for e in trace {
        match e {
            Ev::Flush { t_us, .. } if started.is_none() => started = Some(*t_us),
            Ev::SetBaud { t_us, baud, .. } => {
                let off = 15 * 1_000_000 / u64::from(*baud) + 1;
                longest = longest.max(t_us + off - started.unwrap_or(*t_us));
                started = None;
            }
            _ => {}
        }
    }
    longest
}

const CRSF_RAW: [u16; 16] = [
    172, 992, 1811, 992, 1500, 300, 992, 992, 992, 992, 992, 992, 992, 992, 1100, 1700,
];
const MAV_CH: [u16; 18] = [
    988, 1500, 2012, 1500, 1820, 1065, 1500, 1500, 1500, 1500, 1500, 1500, 0, 65535, 1501, 1999, 0,
    0,
];

fn mav_us() -> [u16; 16] {
    let mut v = [0u16; 16];
    v.copy_from_slice(&MAV_CH[..16]);
    v
}

/// The model itself, and the control that makes I16 mean something: at the sender's baud the
/// bytes arrive exactly, at the other baud they arrive as garbage with framing errors and no
/// valid frame.
#[test]
fn uart_model_same_baud_is_exact_wrong_baud_is_garbage() {
    let mut scen = Scen::default();
    let end = scen.crsf_stream(MS, 300 * MS, 4_000, &CRSF_RAW);
    let sent =
        |scen: &Scen| -> Vec<u8> { scen.sent.iter().flat_map(|s| s.bytes.clone()).collect() };
    let bytes = |items: &[(u64, Rx)]| -> Vec<u8> {
        items
            .iter()
            .filter_map(|(_, r)| match r {
                Rx::Byte(b) => Some(*b),
                Rx::Err(_) => None,
            })
            .collect()
    };
    let errors = |items: &[(u64, Rx)]| {
        items
            .iter()
            .filter(|(_, r)| matches!(r, Rx::Err(_)))
            .count()
    };
    let exact = resample(&scen.wire, CRSF_BAUD, 0, end + MS);
    assert_eq!(bytes(&exact), sent(&scen));
    assert_eq!(errors(&exact), 0);

    let wrong = resample(&scen.wire, MAVLINK_BAUD, 0, end + MS);
    assert!(errors(&wrong) > 100, "framing errors: {}", errors(&wrong));
    let mut d = Deframer::new();
    d.push(&bytes(&wrong), |_| {});
    assert_eq!(d.stats().frames, 0);

    let mut scen = Scen::default();
    let end = scen.mav_stream(MS, 300 * MS, Some(&MAV_CH));
    let wrong = resample(&scen.wire, CRSF_BAUD, 0, end + MS);
    assert!(errors(&wrong) > 50, "framing errors: {}", errors(&wrong));
    let exact = resample(&scen.wire, MAVLINK_BAUD, 0, end + MS);
    assert_eq!(errors(&exact), 0);
    assert_eq!(bytes(&exact), sent(&scen));
}

/// A random segment of the I13/I17/B4 scenarios.
#[derive(Clone, Debug)]
struct Seg {
    mav: bool,
    dur_ms: u64,
    period_us: u64,
    garbage_pct: u64,
    corrupt_pct: u64,
    gap_ms: u64,
    seed: u64,
}

fn seg() -> impl Strategy<Value = Seg> {
    (
        any::<bool>(),
        100u64..800,
        prop_oneof![Just(4_000u64), Just(6_667), Just(20_000)],
        0u64..30,
        0u64..20,
        0u64..1_500,
        any::<u64>(),
    )
        .prop_map(
            |(mav, dur_ms, period_us, garbage_pct, corrupt_pct, gap_ms, seed)| Seg {
                mav,
                dur_ms,
                period_us,
                garbage_pct,
                corrupt_pct,
                gap_ms,
                seed,
            },
        )
}

/// One to four segments after a first one of at least 500 ms with little garbage (so that every
/// case locks at least once).
fn segments() -> impl Strategy<Value = Vec<Seg>> {
    (seg(), proptest::collection::vec(seg(), 0..4)).prop_map(|(mut first, rest)| {
        first.dur_ms = first.dur_ms.max(500);
        first.garbage_pct = first.garbage_pct.min(5);
        first.corrupt_pct = first.corrupt_pct.min(5);
        let mut v = vec![first];
        v.extend(rest);
        v
    })
}

/// Raw CRSF channels 1 to 3 carrying `id` in even values (on even values `raw_to_us` is
/// one-to-one, so the report identifies the frame); the rest 992.
fn crsf_id(id: u32) -> [u16; 16] {
    let mut raw = [992u16; 16];
    for (k, r) in raw.iter_mut().take(3).enumerate() {
        *r = (((id >> (10 * k)) & 0x3FF) << 1) as u16;
    }
    raw
}

/// Builds random valid streams: every stick frame unique (its number is in channels 1 to 3 for CRSF, 1 and 2 for MAVLink),
/// MAVLink channels with 0, 65535 and cut payloads, overrides for other systems, random garbage at
/// the stream's baud between frames, and corrupted copies (one payload byte changed).
fn build(segs: &[Seg]) -> (Scen, u64) {
    let mut scen = Scen::default();
    let mut t = MS;
    let mut id = 1u32;
    for s in segs {
        let mut rng = XorShift(s.seed | 1);
        let end = t + s.dur_ms * MS;
        let baud = if s.mav { MAVLINK_BAUD } else { CRSF_BAUD };
        let period = if s.mav { 10 * MS } else { s.period_us * US };
        let mut k = 0u64;
        while t < end {
            if (rng.below(100) as u64) < s.garbage_pct {
                let n = 1 + rng.below(20);
                let g: Vec<u8> = (0..n).map(|_| rng.below(256) as u8).collect();
                scen.garbage(t, baud, &g);
            }
            if s.mav {
                scen.mav_radio(t);
                if k.is_multiple_of(2) {
                    let mut ch = [0u16; 18];
                    ch[0] = id as u16;
                    ch[1] = (id >> 16) as u16;
                    let cut = rng.below(3) == 0;
                    for (i, c) in ch.iter_mut().enumerate().skip(2) {
                        *c = if cut && i >= 8 {
                            0
                        } else {
                            match rng.below(4) {
                                0 => 0,
                                1 => 65535,
                                _ => rng.below(65536) as u16,
                            }
                        };
                    }
                    let target = match rng.below(10) {
                        0 => 7,
                        1 => 0,
                        _ => OWN_SYS,
                    };
                    scen.mav_override(t + 5 * MS, target, &ch);
                    id += 1;
                }
            } else {
                let mut raw = crsf_id(id);
                for r in raw.iter_mut().skip(3) {
                    *r = rng.below(2048) as u16;
                }
                scen.crsf_rc(t, &raw);
                id += 1;
                if k.is_multiple_of(100_000 / s.period_us.max(1)) {
                    scen.crsf_link(t + period / 2);
                }
            }
            if (rng.below(100) as u64) < s.corrupt_pct {
                let raw = crsf_id(id);
                id += 1;
                let (mut bytes, first) = if s.mav {
                    let mut ch = [1500u16; 18];
                    ch[..2].copy_from_slice(&raw[..2]);
                    let mut first = [0u16; 16];
                    first.copy_from_slice(&ch[..16]);
                    (mav(ELRS_SYS, ELRS_COMP, 0, &overrides(OWN_SYS, &ch)), first)
                } else {
                    (crsf_rc(&raw), raw.map(raw_to_us))
                };
                let header = if s.mav { 10 } else { 3 };
                let at = header + rng.below(bytes.len() - header - 1);
                bytes[at] ^= 1 << rng.below(8);
                scen.put(t + period * 3 / 4, baud, &bytes, What::Never(first));
            }
            t += period;
            k += 1;
        }
        t = scen.wire.end() + s.gap_ms * MS;
    }
    let end_us = scen.wire.end() / US + 1_500_000;
    (scen, end_us)
}

proptest! {
    #![proptest_config(PtConfig { cases: 48, failure_persistence: None, ..PtConfig::default() })]

    /// I13: every stick report equals the conversion of exactly one frame present in the input
    /// that passed its CRC (each frame's values are unique), in order, never twice: never a
    /// held, repeated, interpolated or default value. Corrupted frames and overrides for another
    /// system are never reported.
    #[test]
    fn i13_every_stick_report_is_one_valid_frame_of_the_input(
        segs in segments(),
        step_us in 200u64..2_000,
    ) {
        let (scen, end_us) = build(&segs);
        let sent = scen.sent.clone();
        let sticks = with_sim(Config::new(), scen.wire, |sim| {
            sim.step_us = step_us;
            sim.run_until(end_us);
            sim.sticks()
        });
        prop_assert!(!sticks.is_empty());
        check_reports_come_from_frames(&sticks, &sent)?;
        for (i, r) in sticks.iter().enumerate() {
            prop_assert_eq!(r.seq, i as u32 + 1);
        }
    }

    /// B4: the same input cut into different `receive` calls (whole runs, random pieces, one
    /// byte at a time) gives the same reports, counters and UART actions.
    #[test]
    fn chunking_does_not_change_reports_counters_or_actions(
        segs in segments(),
        step_us in 200u64..2_000,
        split_seed in any::<u64>(),
    ) {
        let (scen, end_us) = build(&segs);
        let run = |split: Split, wire: Wire| {
            with_sim(Config::new(), wire, |sim| {
                sim.step_us = step_us;
                sim.split = split;
                sim.run_until(end_us);
                (sim.hooks.reports.clone(), sim.port.stats(), sim.trace.clone())
            })
        };
        let rebuild = || build(&segs).0.wire;
        let whole = run(Split::Whole, scen.wire);
        let random = run(Split::Random(XorShift(split_seed | 1)), rebuild());
        let bytes = run(Split::OneByte, rebuild());
        prop_assert!(!whole.0.is_empty());
        prop_assert_eq!(&whole, &random);
        prop_assert_eq!(&whole, &bytes);
    }

    /// I17: no byte is written while searching, every write is in the locked protocol at its
    /// baud (whole CRSF battery frames at most one per `crsf_tx_interval_us`, whole MAVLink 2
    /// frames), and before every baud change the fake UART was flushed and its transmitter idle,
    /// after a Flush and a WaitTxIdle with no write in between.
    #[test]
    fn i17_nothing_written_while_searching_tx_idle_before_every_baud_change(
        segs in segments(),
        step_us in 200u64..2_000,
        write_chunk in prop_oneof![Just(1usize), Just(7), Just(300)],
    ) {
        let (scen, end_us) = build(&segs);
        let trace = with_sim(Config::new(), scen.wire, |sim| {
            sim.step_us = step_us;
            sim.write_chunk = write_chunk;
            sim.run_until(end_us);
            sim.trace.clone()
        });
        let mut battery = [0u8; 64];
        let n = encode_battery(&BATTERY, &mut battery).unwrap();
        let battery = &battery[..n];
        let mut stream: Vec<u8> = Vec::new();
        let mut period_mode = None;
        let mut last_crsf_frame_us: Option<u64> = None;
        let mut flushed_since_write = false;
        let mut idle_since_flush = false;
        let mut writes = 0;
        let check_period = |stream: &[u8], mode: Option<Mode>| -> Result<(), TestCaseError> {
            let mut rest = stream;
            while !rest.is_empty() {
                match mode {
                    Some(Mode::Crsf) => {
                        prop_assert!(rest.len() >= battery.len() && &rest[..battery.len()] == battery,
                            "not a whole battery frame: {:?}", rest);
                        rest = &rest[battery.len()..];
                    }
                    Some(Mode::Mavlink) => {
                        prop_assert!(rest.len() >= 12 && rest[0] == 0xFD, "not a MAVLink 2 frame: {:?}", rest);
                        let n = 12 + usize::from(rest[1]);
                        prop_assert!(rest.len() >= n, "a MAVLink frame is cut");
                        rest = &rest[n..];
                    }
                    _ => prop_assert!(false, "bytes written while searching"),
                }
            }
            Ok(())
        };
        for e in &trace {
            match e {
                Ev::Write { t_us, bytes, mode, baud } => {
                    writes += 1;
                    prop_assert_ne!(*mode, Mode::Searching);
                    match mode {
                        Mode::Crsf => prop_assert_eq!(*baud, CRSF_BAUD),
                        Mode::Mavlink => prop_assert_eq!(*baud, MAVLINK_BAUD),
                        Mode::Searching => {}
                    }
                    if period_mode != Some(*mode) {
                        check_period(&stream, period_mode)?;
                        stream.clear();
                        period_mode = Some(*mode);
                    }
                    if *mode == Mode::Crsf && stream.len().is_multiple_of(battery.len()) {
                        if let Some(last) = last_crsf_frame_us {
                            prop_assert!(t_us - last >= u64::from(CRSF_TX_INTERVAL_US),
                                "CRSF frames {} us apart", t_us - last);
                        }
                        last_crsf_frame_us = Some(*t_us);
                    }
                    stream.extend_from_slice(bytes);
                    flushed_since_write = false;
                    idle_since_flush = false;
                }
                Ev::Flush { .. } => flushed_since_write = true,
                Ev::TxIdle { .. } => idle_since_flush = flushed_since_write,
                Ev::SetBaud { flushed, tx_idle, .. } => {
                    prop_assert!(*flushed && *tx_idle, "baud changed with the transmitter busy");
                    prop_assert!(flushed_since_write && idle_since_flush,
                        "baud changed without Flush then WaitTxIdle after the last write");
                    check_period(&stream, period_mode)?;
                    stream.clear();
                    period_mode = None;
                    last_crsf_frame_us = None;
                    flushed_since_write = false;
                    idle_since_flush = false;
                }
            }
        }
        check_period(&stream, period_mode)?;
        prop_assert!(writes > 0);
    }
}

/// The I16 scenario: a UART listening at one baud for a long time (the dwell made 10 s, the
/// worst case: every planted frame falls in one listening period) receives the other protocol's
/// real traffic sent at the other baud, as bytes and framing errors from the resampler, with
/// valid frames planted at random byte positions: 1 or 2 of the listened protocol (below the
/// lock threshold, at least one a stick frame) and 0 to 4 of the other protocol.
#[derive(Clone, Debug)]
struct I16Case {
    listen_mav: bool,
    dur_ms: u64,
    same: Vec<(usize, bool)>,
    other: Vec<usize>,
}

fn i16_case() -> impl Strategy<Value = I16Case> {
    (
        any::<bool>(),
        50u64..150,
        proptest::collection::vec((any::<usize>(), any::<bool>()), 1..=2),
        proptest::collection::vec(any::<usize>(), 0..=4),
    )
        .prop_map(|(listen_mav, dur_ms, mut same, other)| {
            same[0].1 = true;
            I16Case {
                listen_mav,
                dur_ms,
                same,
                other,
            }
        })
}

/// Marker channel values of planted stick frames.
const PLANTED_RAW: [u16; 16] = [1234; 16];
const PLANTED_CH: [u16; 18] = [1234; 18];

/// Feeds received characters (time in ps) to the port, waking every millisecond and at its
/// deadlines, acknowledging every action at once.
fn feed_items(port: &mut RcPort<'_>, items: &[(u64, Rx)], end_us: u64, rec: &mut Rec) {
    let mut params = NoParams;
    let mut tel = Tel;
    let mut i = 0;
    let mut now = 0u64;
    loop {
        let mut bytes = Vec::new();
        while i < items.len() && items[i].0 <= now * US {
            match items[i].1 {
                Rx::Byte(b) => bytes.push(b),
                Rx::Err(e) => {
                    port.receive(&bytes, now, &mut params, rec);
                    bytes.clear();
                    port.uart_error(e, now);
                }
            }
            i += 1;
        }
        port.receive(&bytes, now, &mut params, rec);
        while let Some(a) = port.poll_action(now, &params, &mut tel, rec) {
            match a {
                Action::Write(b) => {
                    let n = b.len();
                    port.written(n);
                }
                _ => port.done(now),
            }
        }
        if now >= end_us {
            return;
        }
        now = (now + 1_000)
            .min(port.next_deadline_us().max(now + 1))
            .min(end_us);
    }
}

/// Runs an I16 case with `lock_frames`; returns every report, the counters and the
/// `Hooks::mavlink` calls. With `custom`, the application lists [`CUSTOM_ID`] and every planted
/// MAVLink frame is that message.
fn i16_run(
    lock_frames: u8,
    case: &I16Case,
    custom: bool,
) -> (Vec<Report>, Stats, Vec<(u32, u32, u64)>) {
    let mut scen = Scen::default();
    let end = if case.listen_mav {
        scen.crsf_stream(MS, (1 + case.dur_ms) * MS, 4_000, &CRSF_RAW)
    } else {
        scen.mav_stream(MS, (1 + case.dur_ms) * MS, Some(&MAV_CH))
    };
    let listen_baud = if case.listen_mav {
        MAVLINK_BAUD
    } else {
        CRSF_BAUD
    };
    let mut items = resample(&scen.wire, listen_baud, 0, end + MS);
    let crsf_planted = |stick: bool| {
        if stick {
            crsf_rc(&PLANTED_RAW)
        } else {
            crsf_link(LINK_FIELDS)
        }
    };
    let mav_planted = |stick: bool| {
        if custom {
            mav(ELRS_SYS, ELRS_COMP, 9, &Custom::<CUSTOM_ID>(0xC0FFEE))
        } else if stick {
            mav(ELRS_SYS, ELRS_COMP, 9, &overrides(OWN_SYS, &PLANTED_CH))
        } else {
            mav(ELRS_SYS, ELRS_COMP, 9, &radio_status(1, 100))
        }
    };
    let mut plant = |pos: usize, frame: Vec<u8>| {
        let at = pos % (items.len() + 1);
        let t = items.get(at).or(items.last()).map_or(MS, |x| x.0);
        for (k, b) in frame.into_iter().enumerate() {
            items.insert(at + k, (t, Rx::Byte(b)));
        }
    };
    for &(pos, stick) in &case.same {
        plant(
            pos,
            if case.listen_mav {
                mav_planted(stick)
            } else {
                crsf_planted(stick)
            },
        );
    }
    for &pos in &case.other {
        plant(
            pos,
            if case.listen_mav {
                crsf_planted(true)
            } else {
                mav_planted(true)
            },
        );
    }
    let config = if case.listen_mav {
        Config::new().with_d_mav_us(10_000_000)
    } else {
        Config::new().with_d_mav_us(1).with_d_crsf_us(10_000_000)
    };
    let mut slots = slots();
    let mav_config = if custom {
        custom_listed()
    } else {
        mavlink_device::Config::new(OWN_SYS, 1)
    };
    let mut port = RcPort::new(config, mav_config, &mut slots).unwrap();
    port.set_lock_frames(lock_frames);
    let mut rec = Rec::default();
    feed_items(&mut port, &items, end / US + 50_000, &mut rec);
    (rec.reports, port.stats(), rec.mavlink)
}

fn i16_check(lock_frames: u8, case: &I16Case) -> Result<(), TestCaseError> {
    let (reports, stats, _) = i16_run(lock_frames, case, false);
    prop_assert!(
        reports.is_empty(),
        "reports from planted frames: {:?}",
        reports
    );
    prop_assert_eq!(stats.mode, Mode::Searching);
    prop_assert!(stats.lock_discarded as usize <= case.same.len());
    prop_assert!(
        stats.uart_framing > 0,
        "the traffic was not garbage at this baud"
    );
    Ok(())
}

proptest! {
    #![proptest_config(PtConfig { cases: 128, failure_persistence: None, ..PtConfig::default() })]

    /// I16: no stick report (and no lock) from CRC-valid frames planted inside wrong-baud
    /// garbage: CRSF traffic sent at 420000 received at 460800 and MAVLink traffic sent at
    /// 460800 received at 420000, with 1 or 2 valid frames of the listened protocol and up to 4
    /// of the other planted at random positions.
    #[test]
    fn i16_no_report_from_valid_frames_planted_in_wrong_baud_garbage(case in i16_case()) {
        i16_check(LOCK_FRAMES, &case)?;
    }
}

proptest! {
    #![proptest_config(PtConfig { cases: 128, failure_persistence: None, ..PtConfig::default() })]

    /// The application's listed message ([`CUSTOM_ID`]) planted in wrong-baud garbage as I16
    /// plants frames (every planted MAVLink frame is that message): never handed to
    /// `Hooks::mavlink`, no report, no lock.
    #[test]
    fn listed_mavlink_message_never_handed_over_from_wrong_baud_garbage(case in i16_case()) {
        let (reports, stats, handed) = i16_run(LOCK_FRAMES, &case, true);
        prop_assert!(handed.is_empty(), "handed over: {:?}", handed);
        prop_assert!(reports.is_empty(), "reports: {:?}", reports);
        prop_assert_eq!(stats.mode, Mode::Searching);
        prop_assert!(stats.uart_framing > 0, "the traffic was not garbage at this baud");
    }
}

/// I16's negative control: the same check, run with LOCK_FRAMES = 1, fails.
#[test]
fn i16_negative_control_fails_with_lock_frames_1() {
    let mut runner = TestRunner::new(PtConfig {
        cases: 128,
        failure_persistence: None,
        ..PtConfig::default()
    });
    let result = runner.run(&i16_case(), |case| i16_check(1, &case));
    assert!(result.is_err(), "the I16 check passed with LOCK_FRAMES = 1");
    // And it fails for the reason it should: a planted stick frame reported.
    let case = I16Case {
        listen_mav: true,
        dur_ms: 100,
        same: vec![(400, true)],
        other: vec![],
    };
    let (reports, _, _) = i16_run(1, &case, false);
    assert!(reports.iter().any(|r| matches!(r, Report::Sticks(s)
    if s.channels_us == {
        let mut v = [0u16; 16];
        v.copy_from_slice(&PLANTED_CH[..16]);
        v
    })));
}

/// I14, CRSF: when the frames stop, the reports stop and every counter freezes; the mode goes
/// to Searching exactly T_LEAVE_CRSF after the last valid frame and searching on a silent line
/// changes no counter.
#[test]
fn i14_crsf_frames_stop_reports_stop_and_counters_freeze() {
    let mut scen = Scen::default();
    let last = scen.crsf_stream(10 * MS, 1_000 * MS, 4_000, &CRSF_RAW);
    with_sim(Config::new(), scen.wire, |sim| {
        sim.run_until(last / US + 2_000);
        let before = sim.port.stats();
        let n = sim.hooks.reports.len();
        assert!(before.sticks > 200);
        assert_eq!(before.mode, Mode::Crsf);
        let last_valid = before
            .last_sticks_us
            .unwrap()
            .max(before.last_crsf_link_us.unwrap());
        sim.run_until(last_valid + 999_000);
        assert_eq!(sim.port.stats(), before);
        sim.run_until(last_valid + 4_000_000);
        let after = sim.port.stats();
        assert_eq!(sim.hooks.reports.len(), n);
        assert_eq!(after.mode, Mode::Searching);
        assert_eq!(after.mode_since_us, last_valid + u64::from(T_LEAVE_CRSF_US));
        let mut same = after;
        same.mode = before.mode;
        same.mode_since_us = before.mode_since_us;
        assert_eq!(same, before, "a counter moved after the frames stopped");
    });
}

/// I14, MAVLink: when the overrides stop, the stick reports stop and their counter freezes,
/// while RADIO_STATUS keeps coming, is reported, and keeps the mode Mavlink.
#[test]
fn i14_mavlink_overrides_stop_radio_status_continues_mode_stays() {
    let mut scen = Scen::default();
    scen.mav_stream(10 * MS, 1_000 * MS, Some(&MAV_CH));
    let end = scen.mav_stream(1_000 * MS, 3_000 * MS, None);
    let last_override = scen.sticks().last().unwrap().end;
    with_sim(Config::new(), scen.wire, |sim| {
        sim.run_until(last_override / US + 2_000);
        let before = sim.port.stats();
        assert_eq!(before.mode, Mode::Mavlink);
        assert!(before.sticks > 40);
        sim.run_until(end / US + 2_000);
        let after = sim.port.stats();
        assert_eq!(after.mode, Mode::Mavlink);
        assert_eq!(after.mode_since_us, before.mode_since_us);
        assert_eq!(after.sticks, before.sticks);
        assert_eq!(after.last_sticks_us, before.last_sticks_us);
        assert!(after.radio_status >= before.radio_status + 199);
        assert_eq!(
            (after.crc_errors, after.garbage_bytes),
            (before.crc_errors, before.garbage_bytes)
        );
        assert!(
            sim.sticks()
                .iter()
                .all(|s| s.at_us * US <= last_override + 2 * MS)
        );
    });
}

/// MAVLink mode with the radio link lost (RADIO_STATUS only, 5 s): it stays Mavlink with no
/// baud change, the stick-report counter is frozen and the link-report counter advances at the
/// RADIO_STATUS rate.
#[test]
fn mavlink_link_lost_stays_mavlink_sticks_frozen_radio_status_advances() {
    let mut scen = Scen::default();
    scen.mav_stream(10 * MS, 600 * MS, Some(&MAV_CH));
    let lost_at = scen.wire.end();
    let end = scen.mav_stream(600 * MS, 5_600 * MS, None);
    with_sim(Config::new(), scen.wire, |sim| {
        sim.run_until(lost_at / US + 2_000);
        let at_loss = sim.port.stats();
        let bauds_at_loss = sim.bauds().len();
        assert_eq!(at_loss.mode, Mode::Mavlink);
        for k in 1..=5 {
            sim.run_until(lost_at / US + k * 1_000_000);
            let s = sim.port.stats();
            assert_eq!(s.mode, Mode::Mavlink);
            assert_eq!(s.sticks, at_loss.sticks);
            assert_eq!(s.last_sticks_us, at_loss.last_sticks_us);
            let expected = at_loss.radio_status + k as u32 * 100;
            assert!(
                s.radio_status.abs_diff(expected) <= 1,
                "{} vs {expected}",
                s.radio_status
            );
        }
        sim.run_until(end / US + 2_000);
        assert_eq!(
            sim.bauds().len(),
            bauds_at_loss,
            "a baud change in MAVLink mode"
        );
        assert_eq!(sim.port.stats().mode_changes, 0);
    });
}

#[derive(Clone, Debug)]
struct I18Case {
    mav: bool,
    gap_ms: u64,
    garbage: Option<(bool, Vec<u8>)>,
    step_us: u64,
}

fn i18_case() -> impl Strategy<Value = I18Case> {
    (
        any::<bool>(),
        0u64..1_000,
        proptest::option::of((
            any::<bool>(),
            proptest::collection::vec(any::<u8>(), 1..200),
        )),
        200u64..1_000,
    )
        .prop_map(|(mav, gap, garbage, step_us)| I18Case {
            mav,
            // A gap below T_LEAVE less a margin of 30 ms: the delivery latency (2 w) and the
            // time a false CRSF candidate in the garbage can hold back the next frame (64 bytes).
            gap_ms: if mav { gap % 170 } else { gap % 970 },
            garbage,
            step_us,
        })
}

proptest! {
    #![proptest_config(PtConfig { cases: 64, failure_persistence: None, ..PtConfig::default() })]

    /// I18: a gap in the same mode shorter than T_LEAVE (silence, or garbage at either baud, as
    /// when the receiver re-opens its port) causes no mode change and no baud change.
    #[test]
    fn i18_same_mode_gap_shorter_than_t_leave_changes_nothing(case in i18_case()) {
        let mut scen = Scen::default();
        let first_end = if case.mav {
            scen.mav_stream(10 * MS, 600 * MS, Some(&MAV_CH))
        } else {
            scen.crsf_stream(10 * MS, 600 * MS, 4_000, &CRSF_RAW)
        };
        let resume = first_end + case.gap_ms * MS;
        if let Some((at_mav_baud, bytes)) = &case.garbage {
            let baud = if *at_mav_baud { MAVLINK_BAUD } else { CRSF_BAUD };
            let len = bytes_us(bytes.len(), baud) * US;
            if first_end + MS + len < resume {
                scen.garbage(first_end + MS, baud, bytes);
            }
        }
        let end = if case.mav {
            scen.mav_stream(resume, resume + 600 * MS, Some(&MAV_CH))
        } else {
            scen.crsf_stream(resume, resume + 600 * MS, 4_000, &CRSF_RAW)
        };
        let (modes, bauds, stats, after_gap) = with_sim(Config::new(), scen.wire, |sim| {
            sim.step_us = case.step_us;
            sim.run_until(end / US + 2_000);
            let after_gap = sim.sticks().iter().filter(|s| s.at_us * US > resume).count();
            (sim.modes.clone(), sim.bauds(), sim.port.stats(), after_gap)
        });
        let locked = if case.mav { Mode::Mavlink } else { Mode::Crsf };
        prop_assert_eq!(modes.iter().map(|m| m.1).collect::<Vec<_>>(), vec![Mode::Searching, locked]);
        let lock_us = modes[1].0;
        prop_assert!(bauds.iter().all(|b| b.0 <= lock_us), "baud change after the lock: {:?}", bauds);
        prop_assert_eq!(stats.mode_changes, 0);
        prop_assert!(after_gap > 0);
    }
}

/// A run of protocol changes: segment k is CRSF or MAVLink for `seg_ms`, then `gap_ms` of
/// silence before the next.
struct Changes {
    scen: Scen,
    /// (mav, first frame start, last frame end) of each segment.
    segs: Vec<(bool, u64, u64)>,
}

fn changes(first_mav: bool, gaps_ms: &[u64], seg_ms: u64, crsf_period_us: u64) -> Changes {
    let mut scen = Scen::default();
    let mut segs = Vec::new();
    let mut t = 20 * MS;
    let mut mav = first_mav;
    for k in 0..=gaps_ms.len() {
        let start = t;
        let end = if mav {
            scen.mav_stream(start, start + seg_ms * MS, Some(&MAV_CH))
        } else {
            scen.crsf_stream(start, start + seg_ms * MS, crsf_period_us, &CRSF_RAW)
        };
        segs.push((mav, start, end));
        if let Some(gap) = gaps_ms.get(k) {
            t = end + gap * MS;
        }
        mav = !mav;
    }
    Changes { scen, segs }
}

/// The lock bound of the module comment, in us, for a change into `mav` (or the first lock).
fn lock_bound_us(
    into_mav: bool,
    t_last: Option<u64>,
    t_first: u64,
    crsf_period_us: u64,
    eps: u64,
    w: u64,
) -> u64 {
    let (p, d, d_other, t_leave_old) = if into_mav {
        (
            10_000,
            bytes_us(12 + 38, MAVLINK_BAUD),
            u64::from(D_CRSF_US),
            u64::from(T_LEAVE_CRSF_US),
        )
    } else {
        (
            crsf_period_us,
            bytes_us(26, CRSF_BAUD),
            u64::from(D_MAV_US),
            u64::from(T_LEAVE_MAV_US),
        )
    };
    // At power-up the search is already running when the first frames start.
    let base = match t_last {
        Some(t_last) => (t_last / US) + t_leave_old.max((t_first - t_last) / US + 1),
        None => t_first / US,
    };
    base + d_other + 5 * p + 2 * d + 2 * eps + 3 * w
}

/// Checks a run of changes: the locks alternate as the segments do, each within the bound, every
/// stick report has the value of its segment's protocol and falls in that segment, and every
/// stick frame that starts after its segment's lock is reported.
fn check_changes(c: &Changes, sim: &Sim<'_>, crsf_period_us: u64) -> Result<u64, TestCaseError> {
    let eps = eps_us(&sim.trace);
    let w = sim.step_us;
    let locks: Vec<(u64, Mode)> = sim
        .modes
        .iter()
        .copied()
        .filter(|m| m.1 != Mode::Searching)
        .collect();
    prop_assert_eq!(locks.len(), c.segs.len(), "locks {:?}", sim.modes);
    let mut worst_margin = u64::MAX;
    for (k, &(mav, start, end)) in c.segs.iter().enumerate() {
        let (lock_us, mode) = locks[k];
        prop_assert_eq!(mode, if mav { Mode::Mavlink } else { Mode::Crsf });
        let t_last = k.checked_sub(1).map(|j| c.segs[j].2);
        let bound = lock_bound_us(mav, t_last, start, crsf_period_us, eps, w);
        prop_assert!(
            lock_us <= bound,
            "lock {k} at {lock_us} us, bound {bound} us (eps {eps})"
        );
        worst_margin = worst_margin.min(bound - lock_us);
        // When the new frames already flow as the old lock is left (G <= T_LEAVE_old), the first
        // listening period is at the new baud ("the other baud first") and locks:
        // lock - t_last <= T_LEAVE_old + eps + 3 P + d + 2 w.
        if let Some(t_last) = t_last {
            let (p, d, t_leave_old) = if mav {
                (
                    10_000,
                    bytes_us(12 + 38, MAVLINK_BAUD),
                    u64::from(T_LEAVE_CRSF_US),
                )
            } else {
                (
                    crsf_period_us,
                    bytes_us(26, CRSF_BAUD),
                    u64::from(T_LEAVE_MAV_US),
                )
            };
            if start - t_last <= t_leave_old * US {
                let sharp = t_last / US + t_leave_old + eps + 3 * p + d + 2 * w;
                prop_assert!(
                    lock_us <= sharp,
                    "lock {k} at {lock_us} us, first-period bound {sharp} us"
                );
            }
        }
        let next_start = c.segs.get(k + 1).map_or(u64::MAX, |s| s.1);
        let expected = if mav {
            (
                mav_us(),
                Source::Mavlink {
                    system_id: ELRS_SYS,
                    component_id: ELRS_COMP,
                },
            )
        } else {
            (CRSF_RAW.map(raw_to_us), Source::Crsf)
        };
        let reports: Vec<StickReport> = sim
            .sticks()
            .into_iter()
            .filter(|r| r.at_us * US >= start && r.at_us * US < next_start)
            .collect();
        for r in &reports {
            prop_assert_eq!((r.channels_us, r.source), expected);
            prop_assert!(r.at_us * US <= end + 2 * w * US);
        }
        let frames_after_lock = c
            .scen
            .sticks()
            .iter()
            .filter(|s| s.start >= start && s.end <= end && s.start > lock_us * US)
            .count();
        prop_assert!(
            reports.len() >= frames_after_lock,
            "segment {k}: {} reports, {frames_after_lock} frames after the lock",
            reports.len()
        );
    }
    Ok(worst_margin)
}

/// Mode following end to end on the model: CRSF (250 Hz), then the transmitter changes model:
/// 100 ms of silence, MAVLink at 460800 (RADIO_STATUS every 10 ms, RC_CHANNELS_OVERRIDE every
/// 20 ms), and back; 10 changes each way in one run. The mode-change counter is 20, every lock
/// is within the bound of the module comment, and with the sticks held still every report
/// equals the value of its protocol.
#[test]
fn mode_following_10_changes_each_way() {
    let crsf_period_us = 4_000;
    let mut c = changes(false, &[100; 20], 1_500, crsf_period_us);
    let end_us = c.segs.last().unwrap().2 / US + 10_000;
    let wire = std::mem::take(&mut c.scen.wire);
    with_sim(Config::new(), wire, |sim| {
        sim.run_until(end_us);
        let stats = sim.port.stats();
        assert_eq!(stats.mode_changes, 20);
        let margin = check_changes(&c, sim, crsf_period_us).unwrap();
        assert!(margin > 0);
        assert!(stats.sticks > 20 * 50);
        assert!(stats.lock_discarded >= 2 * 21);
    });
}

proptest! {
    #![proptest_config(PtConfig { cases: 64, failure_persistence: None, ..PtConfig::default() })]

    /// The lock bound of the module comment over random gaps (0 to 1.5 s, so on both sides of
    /// T_LEAVE), random phases, both directions, CRSF at 250 or 50 Hz, random wake periods; also
    /// under the brief's max(T_LEAVE_old, gap) + 280 ms.
    #[test]
    fn lock_bound_holds_for_random_gaps_and_phases(
        first_mav in any::<bool>(),
        gaps in proptest::collection::vec(0u64..1_500, 1..=2),
        // Each segment outlasts T_LEAVE_CRSF plus the bound: a shorter one is never seen,
        // by design (the old lock holds for T_LEAVE).
        seg_ms in 1_300u64..1_800,
        crsf_period_us in prop_oneof![Just(4_000u64), Just(20_000)],
        step_us in 100u64..1_000,
    ) {
        let mut c = changes(first_mav, &gaps, seg_ms, crsf_period_us);
        let end_us = c.segs.last().unwrap().2 / US + 10_000;
        let wire = std::mem::take(&mut c.scen.wire);
        with_sim(Config::new(), wire, |sim| {
            sim.step_us = step_us;
            sim.run_until(end_us);
            check_changes(&c, sim, crsf_period_us)?;
            let locks: Vec<u64> = sim.modes.iter().filter(|m| m.1 != Mode::Searching).map(|m| m.0).collect();
            for (pair, lock) in c.segs.windows(2).zip(&locks[1..]) {
                let (t_last, t_first) = (pair[0].2 / US, pair[1].1 / US);
                let t_leave = if pair[0].0 { T_LEAVE_MAV_US } else { T_LEAVE_CRSF_US };
                prop_assert!(*lock <= t_last + u64::from(t_leave).max(t_first - t_last) + 280_000);
            }
            Ok(())
        })?;
    }
}

/// CRSF mode with the radio link lost (total silence): the port leaves CRSF exactly T_LEAVE_CRSF
/// after the last frame, keeps searching (460800 for D_MAV, the CRSF baud for D_CRSF, again and
/// again), writes nothing, and when the frames return at any phase of the search it re-locks
/// within D_MAV + 5 P + 2 d + 2 eps + 2 w of their return. Swept over return times 2 ms apart
/// across more than one search cycle, at 250 Hz and 50 Hz.
#[test]
fn crsf_link_lost_keeps_searching_writes_nothing_relocks_within_bound() {
    for period_us in [4_000u64, 20_000] {
        for k in 0..140u64 {
            let mut scen = Scen::default();
            let last = scen.crsf_stream(10 * MS, 500 * MS, period_us, &CRSF_RAW);
            let back = last + 1_000 * MS + 600 * MS + k * 2 * MS;
            let end = scen.crsf_stream(back, back + 400 * MS, period_us, &CRSF_RAW);
            with_sim(Config::new(), scen.wire, |sim| {
                sim.run_until(back / US - 1);
                let s = sim.port.stats();
                assert_eq!(s.mode, Mode::Searching);
                let left = s.mode_since_us;
                let last_valid = s.last_sticks_us.unwrap().max(s.last_crsf_link_us.unwrap());
                assert_eq!(left, last_valid + u64::from(T_LEAVE_CRSF_US));
                let bauds: Vec<(u64, u32)> =
                    sim.bauds().into_iter().filter(|b| b.0 >= left).collect();
                assert!(bauds.len() >= 6, "{bauds:?}");
                assert_eq!(bauds[0], (left, MAVLINK_BAUD));
                for pair in bauds.windows(2) {
                    let ((t0, b0), (t1, b1)) = (pair[0], pair[1]);
                    assert_ne!(b0, b1, "the bauds alternate");
                    let dwell = u64::from(if b0 == MAVLINK_BAUD {
                        D_MAV_US
                    } else {
                        D_CRSF_US
                    });
                    // The listening period starts when set_baud returns (15 bit periods, 36 us).
                    assert!(
                        t1 >= t0 + dwell && t1 <= t0 + dwell + 37,
                        "dwell {} us at {b0}",
                        t1 - t0
                    );
                }
                let writes_while_searching = sim
                    .trace
                    .iter()
                    .filter(|e| matches!(e, Ev::Write { t_us, .. } if *t_us >= left))
                    .count();
                assert_eq!(writes_while_searching, 0);
                sim.run_until(end / US + 2_000);
                let s = sim.port.stats();
                assert_eq!(s.mode, Mode::Crsf);
                assert_eq!(s.mode_changes, 0);
                let eps = eps_us(&sim.trace);
                let bound = u64::from(D_MAV_US)
                    + 5 * period_us
                    + 2 * bytes_us(26, CRSF_BAUD)
                    + 2 * eps
                    + 2 * sim.step_us;
                let relock = s.mode_since_us - back / US;
                assert!(
                    relock <= bound,
                    "period {period_us} us, return {k}: relock after {relock} us, bound {bound} us"
                );
            });
        }
    }
}

/// RC_CHANNELS_OVERRIDE target filter and values: target = own system id and target = 0 give a
/// report, another target gives none and is counted in `overrides_not_for_us` (it still counts
/// as a valid MAVLink frame). Values are exactly as received: 0 and 65535 kept, channels 9 to 16
/// of a payload cut before them read 0, channels 17 and 18 dropped.
#[test]
fn override_target_filter_and_exact_values() {
    let mut scen = Scen::default();
    scen.mav_stream(10 * MS, 100 * MS, None);
    let own: [u16; 18] = [
        0, 65535, 1000, 2000, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
    ];
    let zero: [u16; 18] = [1500; 18];
    let other: [u16; 18] = [1600; 18];
    let cut: [u16; 18] = [
        1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];
    scen.mav_override(105 * MS, OWN_SYS, &own);
    scen.mav_override(110 * MS, 0, &zero);
    scen.mav_override(115 * MS, 7, &other);
    scen.mav_override(120 * MS, OWN_SYS, &cut);
    let cut_frame = mav(ELRS_SYS, ELRS_COMP, 0, &overrides(OWN_SYS, &cut));
    assert_eq!(
        cut_frame[1], 17,
        "the payload is cut after target_component"
    );
    let end = scen.mav_stream(125 * MS, 200 * MS, None);
    with_sim(Config::new(), scen.wire, |sim| {
        sim.run_until(end / US + 2_000);
        let got: Vec<[u16; 16]> = sim.sticks().iter().map(|s| s.channels_us).collect();
        let first16 = |c: &[u16; 18]| {
            let mut v = [0u16; 16];
            v.copy_from_slice(&c[..16]);
            v
        };
        assert_eq!(got, vec![first16(&own), first16(&zero), first16(&cut)]);
        for s in sim.sticks() {
            assert_eq!(
                s.source,
                Source::Mavlink {
                    system_id: ELRS_SYS,
                    component_id: ELRS_COMP
                }
            );
        }
        let stats = sim.port.stats();
        assert_eq!(stats.overrides_not_for_us, 1);
        assert_eq!(stats.mode, Mode::Mavlink);
    });
}

/// UART errors: a burst of framing errors increments the counter, produces no report and does
/// not by itself change the mode before T_LEAVE; it does not keep the lock either.
#[test]
fn uart_error_burst_counted_no_report_no_mode_change_before_t_leave() {
    let mut slots = slots();
    let mut port = RcPort::new(
        Config::new(),
        mavlink_device::Config::new(OWN_SYS, 1),
        &mut slots,
    )
    .unwrap();
    let mut rec = Rec::default();
    let mut tel = Tel;
    let mut actions = |port: &mut RcPort<'_>, rec: &mut Rec, now: u64| {
        let mut bauds = Vec::new();
        while let Some(a) = port.poll_action(now, &NoParams, &mut tel, rec) {
            match a {
                Action::Write(b) => {
                    let n = b.len();
                    port.written(n);
                }
                Action::SetBaud(b) => {
                    bauds.push(b);
                    port.done(now);
                }
                _ => port.done(now),
            }
        }
        bauds
    };
    assert_eq!(actions(&mut port, &mut rec, 0), vec![MAVLINK_BAUD]);
    assert_eq!(actions(&mut port, &mut rec, 50_000), vec![CRSF_BAUD]);
    let frame = crsf_rc(&CRSF_RAW);
    for k in 0..10u64 {
        port.receive(&frame, 60_000 + k * 4_000, &mut NoParams, &mut rec);
    }
    assert_eq!(port.stats().mode, Mode::Crsf);
    let last_valid = 60_000 + 9 * 4_000;
    let reports = rec.reports.len();
    assert_eq!(reports, 8);
    for k in 0..1_000u64 {
        let now = last_valid + 1 + k * 990;
        port.uart_error(UartError::Framing, now);
        assert!(actions(&mut port, &mut rec, now).is_empty());
    }
    let s = port.stats();
    assert_eq!(s.uart_framing, 1_000);
    assert_eq!(s.mode, Mode::Crsf);
    assert_eq!(rec.reports.len(), reports);
    port.receive(&frame, last_valid + 999_999, &mut NoParams, &mut rec);
    assert_eq!(
        rec.reports.len(),
        reports + 1,
        "a frame before T_LEAVE is still reported"
    );
    let last_valid = last_valid + 999_999;
    for k in 0..1_000u64 {
        port.uart_error(UartError::Framing, last_valid + 1 + k * 999);
    }
    assert_eq!(port.stats().mode, Mode::Crsf);
    assert_eq!(
        actions(&mut port, &mut rec, last_valid + u64::from(T_LEAVE_CRSF_US)),
        vec![MAVLINK_BAUD]
    );
    let s = port.stats();
    assert_eq!(
        (s.mode, s.uart_framing, s.mode_since_us),
        (
            Mode::Searching,
            2_000,
            last_valid + u64::from(T_LEAVE_CRSF_US)
        )
    );
    port.uart_error(UartError::Break, last_valid + 2_000_000);
    port.uart_error(UartError::Overrun, last_valid + 2_000_000);
    port.uart_error(UartError::Parity, last_valid + 2_000_000);
    port.uart_error(UartError::Other, last_valid + 2_000_000);
    let s = port.stats();
    assert_eq!(
        (s.uart_break, s.uart_overrun, s.uart_parity, s.uart_other),
        (1, 1, 1, 1)
    );
}

/// Carries out every pending action at `now_us` (writes taken whole, waits and baud changes done
/// at once); returns the last baud set.
fn settle(port: &mut RcPort<'_>, now_us: u64, rec: &mut Rec) -> Option<u32> {
    let mut baud = None;
    while let Some(a) = port.poll_action(now_us, &NoParams, &mut Tel, rec) {
        match a {
            Action::Write(b) => {
                let n = b.len();
                port.written(n);
            }
            Action::SetBaud(b) => {
                baud = Some(b);
                port.done(now_us);
            }
            _ => port.done(now_us),
        }
    }
    baud
}

/// A port listening at the CRSF baud (`crsf`) or at 460800 from the returned time on.
fn port_listening<'a>(
    slots: &'a mut [mavlink_device::StreamSlot],
    crsf: bool,
    rec: &mut Rec,
) -> (RcPort<'a>, u64) {
    let config = if crsf {
        Config::new().with_d_mav_us(1)
    } else {
        Config::new()
    };
    let mut port = RcPort::new(config, mavlink_device::Config::new(OWN_SYS, 1), slots).unwrap();
    assert_eq!(settle(&mut port, 0, rec), Some(MAVLINK_BAUD));
    if !crsf {
        return (port, 0);
    }
    assert_eq!(settle(&mut port, 1, rec), Some(CRSF_BAUD));
    (port, 1)
}

/// A valid stick frame: CRSF RC or a MAVLink RC_CHANNELS_OVERRIDE for this system.
fn stick_frame(crsf: bool, seq: u8) -> Vec<u8> {
    if crsf {
        crsf_rc(&CRSF_RAW)
    } else {
        mav(ELRS_SYS, ELRS_COMP, seq, &overrides(OWN_SYS, &MAV_CH))
    }
}

/// `receive` applies the deadline due at its `now_us` before it looks at its bytes, so a caller
/// that hands frames over late can neither keep a lock nor finish a lock-on with them. (i) Locked:
/// a valid stick frame in a `receive` at the leave deadline (last valid frame + T_LEAVE) is not
/// reported, the lock is left at that deadline and the frame's bytes are garbage. (ii)
/// Listening: a third valid frame in a `receive` at the end of the listening period does not
/// lock. Both protocols.
#[test]
fn late_receive_applies_the_deadline_before_the_bytes() {
    for crsf in [false, true] {
        let (locked, t_leave, dwell) = if crsf {
            (Mode::Crsf, u64::from(T_LEAVE_CRSF_US), u64::from(D_CRSF_US))
        } else {
            (
                Mode::Mavlink,
                u64::from(T_LEAVE_MAV_US),
                u64::from(D_MAV_US),
            )
        };

        // (i) Locked.
        let mut sl = slots();
        let mut rec = Rec::default();
        let (mut port, t0) = port_listening(&mut sl, crsf, &mut rec);
        let mut last = t0;
        for k in 1..=4u8 {
            last = t0 + u64::from(k) * 1_000;
            port.receive(&stick_frame(crsf, k), last, &mut NoParams, &mut rec);
        }
        let before = port.stats();
        assert_eq!((before.mode, before.sticks), (locked, 2), "crsf {crsf}");
        let frame = stick_frame(crsf, 5);
        port.receive(&frame, last + t_leave, &mut NoParams, &mut rec);
        let after = port.stats();
        assert_eq!(
            after.mode,
            Mode::Searching,
            "crsf {crsf}: a late frame kept the lock"
        );
        assert_eq!(after.mode_since_us, last + t_leave, "crsf {crsf}");
        assert_eq!(
            (after.sticks, rec.reports.len()),
            (2, 2),
            "crsf {crsf}: a late frame was reported"
        );
        assert_eq!(
            after.garbage_bytes,
            before.garbage_bytes + frame.len() as u32,
            "crsf {crsf}"
        );

        // (ii) Listening.
        let mut sl = slots();
        let mut rec = Rec::default();
        let (mut port, t0) = port_listening(&mut sl, crsf, &mut rec);
        for k in 1..=2u8 {
            let t = t0 + u64::from(k) * 1_000;
            port.receive(&stick_frame(crsf, k), t, &mut NoParams, &mut rec);
        }
        let before = port.stats();
        assert_eq!(before.lock_discarded, 2, "crsf {crsf}");
        let frame = stick_frame(crsf, 3);
        port.receive(&frame, t0 + dwell, &mut NoParams, &mut rec);
        let after = port.stats();
        assert_eq!(
            (after.mode, after.sticks, after.lock_discarded),
            (Mode::Searching, 0, 2),
            "crsf {crsf}: a frame after the listening period locked"
        );
        assert!(rec.reports.is_empty(), "crsf {crsf}");
        assert_eq!(
            after.garbage_bytes,
            before.garbage_bytes + frame.len() as u32,
            "crsf {crsf}"
        );
    }
}

/// The length of the whole frame of the given protocol at the start of `bytes`, if there is one:
/// CRSF (address 0xC8, length byte 2 to 62) or MAVLink 2 (0xFD, incompatibility flags 0, not
/// signed).
fn whole_frame_len(crsf: bool, bytes: &[u8]) -> Option<usize> {
    let n = if crsf {
        let len = usize::from(*bytes.get(1)?);
        (bytes[0] == 0xC8 && (2..=62).contains(&len)).then_some(len + 2)?
    } else {
        (bytes[0] == 0xFD && *bytes.get(2)? == 0).then_some(usize::from(bytes[1]) + 12)?
    };
    (n <= bytes.len()).then_some(n)
}

/// When a lock is left with a frame written only in part, its unsent rest is dropped: the port
/// locked on protocol A has written 2 bytes of a frame (`written(2)`) when its caller stalls past
/// the leave deadline (as `run` would behind a slow write); the lock is left, and after the baud
/// change to B and the lock on B, the fake UART's trace shows nothing written before the baud
/// change, and every byte written after it, from the first write on, in whole frames of B (the
/// first write is one whole frame). A to B is CRSF to MAVLink and MAVLink to CRSF.
#[test]
fn unsent_rest_of_a_frame_dropped_when_the_lock_is_left() {
    for crsf_first in [true, false] {
        let mut scen = Scen::default();
        let (old_end, end) = if crsf_first {
            let old_end = scen.crsf_stream(MS, 400 * MS, 4_000, &CRSF_RAW);
            (
                old_end,
                scen.mav_stream(old_end + 50 * MS, 2_500 * MS, Some(&MAV_CH)),
            )
        } else {
            let old_end = scen.mav_stream(MS, 400 * MS, Some(&MAV_CH));
            (
                old_end,
                scen.crsf_stream(old_end + 50 * MS, 1_500 * MS, 4_000, &CRSF_RAW),
            )
        };
        let (old_mode, new_mode, new_baud, t_leave) = if crsf_first {
            (
                Mode::Crsf,
                Mode::Mavlink,
                MAVLINK_BAUD,
                u64::from(T_LEAVE_CRSF_US),
            )
        } else {
            (
                Mode::Mavlink,
                Mode::Crsf,
                CRSF_BAUD,
                u64::from(T_LEAVE_MAV_US),
            )
        };
        with_sim(Config::new(), scen.wire, |sim| {
            sim.run_until(old_end / US + 1);
            let s = sim.port.stats();
            assert_eq!(s.mode, old_mode);
            let last_valid = [
                s.last_sticks_us,
                s.last_crsf_link_us,
                s.last_radio_status_us,
            ]
            .into_iter()
            .flatten()
            .max()
            .unwrap();
            let leave = last_valid + t_leave;
            // The next write of A, before the lock is left; the UART takes 2 of its bytes.
            let frame = loop {
                let now = sim.now_us();
                assert!(now < leave, "no write before the leave deadline");
                if let Some(Action::Write(b)) =
                    sim.port
                        .poll_action(now, &sim.params, &mut sim.tel, &mut sim.hooks)
                {
                    break b.to_vec();
                }
                sim.t += 100 * US;
            };
            assert_eq!(whole_frame_len(crsf_first, &frame), Some(frame.len()));
            sim.tx.write(sim.t, 2);
            sim.port.written(2);
            sim.trace.push(Ev::Write {
                t_us: sim.now_us(),
                bytes: frame[..2].to_vec(),
                mode: old_mode,
                baud: sim.baud,
            });
            let partial = sim.trace.len();
            // The caller stalls past the leave deadline.
            sim.t = (leave + 1) * US;
            sim.run_until(end / US + 10_000);
            assert_eq!(sim.port.stats().mode, new_mode, "crsf_first {crsf_first}");
            let after = &sim.trace[partial..];
            let set = after
                .iter()
                .position(|e| matches!(e, Ev::SetBaud { .. }))
                .unwrap();
            assert!(matches!(after[0], Ev::Flush { .. }), "{:?}", after[0]);
            assert!(
                matches!(after[set], Ev::SetBaud { baud, .. } if baud == new_baud),
                "{:?}",
                after[set]
            );
            let writes: Vec<&Vec<u8>> = after[set..]
                .iter()
                .filter_map(|e| match e {
                    Ev::Write { bytes, baud, .. } => {
                        assert_eq!(*baud, new_baud);
                        Some(bytes)
                    }
                    _ => None,
                })
                .collect();
            assert!(!writes.is_empty(), "nothing written after the lock on B");
            assert_eq!(
                whole_frame_len(!crsf_first, writes[0]),
                Some(writes[0].len()),
                "crsf_first {crsf_first}: the first write after the baud change is not one whole frame of B: {:?} (rest of A: {:?})",
                writes[0],
                &frame[2..]
            );
            let all: Vec<u8> = writes.iter().flat_map(|w| w.iter().copied()).collect();
            let mut rest = &all[..];
            while !rest.is_empty() {
                let n = whole_frame_len(!crsf_first, rest).unwrap_or_else(|| {
                    panic!("crsf_first {crsf_first}: bytes not in whole frames of B: {rest:?}")
                });
                rest = &rest[n..];
            }
        });
    }
}

/// `Hooks::mavlink`: the application's listed message ([`CUSTOM_ID`], the only MAVLink traffic
/// with an unlisted id between its frames) is handed over once per frame, in order, while locked
/// on MAVLink and from the 3rd valid frame of the lock on: frames 1 and 2 are lock-on frames. It
/// alone makes and keeps the lock: the lock is left T_LEAVE_MAV after the last one. The unlisted
/// id never is handed over, nor the listed message sent at the CRSF baud while locked on CRSF.
#[test]
fn listed_mavlink_message_handed_over_only_while_locked() {
    let mut scen = Scen::default();
    let mut ends = Vec::new();
    let mut t = MS;
    for k in 1..=30u32 {
        let listed = mav(ELRS_SYS, ELRS_COMP, k as u8, &Custom::<CUSTOM_ID>(k));
        ends.push(scen.garbage(t, MAVLINK_BAUD, &listed));
        let unlisted = mav(
            ELRS_SYS,
            ELRS_COMP,
            k as u8,
            &Custom::<UNLISTED_ID>(1_000 + k),
        );
        scen.garbage(t + 5 * MS, MAVLINK_BAUD, &unlisted);
        t += 10 * MS;
    }
    let crsf_from = ends[29] + 300 * MS;
    let mut t = crsf_from;
    let mut k = 2_000u32;
    while t < crsf_from + 600 * MS {
        scen.crsf_rc(t, &CRSF_RAW);
        let listed = mav(ELRS_SYS, ELRS_COMP, 0, &Custom::<CUSTOM_ID>(k));
        scen.garbage(t + 2 * MS, CRSF_BAUD, &listed);
        k += 1;
        t += 4 * MS;
    }
    let mut sl = slots();
    let mut sim = Sim::with_mav(Config::new(), custom_listed(), &mut sl);
    sim.wire = scen.wire;
    sim.run_until(t / US + 10_000);

    let handed = &sim.hooks.mavlink;
    let markers: Vec<u32> = handed.iter().map(|h| h.1).collect();
    assert_eq!(markers, (3..=30).collect::<Vec<u32>>(), "{handed:?}");
    for &(id, marker, at_us) in handed {
        assert_eq!(id, CUSTOM_ID);
        assert!(
            at_us * US + US >= ends[marker as usize - 1],
            "handed over before its end"
        );
    }
    let last_us = handed.last().unwrap().2;
    let lock_us = handed[0].2;
    let crsf_modes: Vec<Mode> = sim.modes.iter().map(|m| m.1).collect();
    assert_eq!(
        crsf_modes,
        [Mode::Searching, Mode::Mavlink, Mode::Searching, Mode::Crsf]
    );
    assert_eq!(sim.modes[1].0, lock_us, "locked by the 3rd listed message");
    assert_eq!(
        sim.modes[2].0,
        last_us + u64::from(T_LEAVE_MAV_US),
        "the lock was kept by the listed messages"
    );
    let s = sim.port.stats();
    assert_eq!((s.lock_discarded, s.radio_status), (4, 0));
    assert!(s.sticks > 100, "the CRSF lock reported its sticks");
    assert!(sim.port.node().stats().dropped_unknown_message >= 30);

    // The garbage check above can fail: with LOCK_FRAMES = 1 a planted listed message is handed
    // over.
    let case = I16Case {
        listen_mav: true,
        dur_ms: 100,
        same: vec![(400, false)],
        other: vec![],
    };
    assert!(!i16_run(1, &case, true).2.is_empty());
}

/// The merged accept list holds the application's entries and the port's two where not listed,
/// 16 at most: more is `NewError::AcceptListFull`, nothing dropped.
#[test]
fn merged_accept_list_over_16_is_an_error() {
    let pairs: Vec<(u32, u8)> = (0..16).map(|i| (50_000 + i, i as u8)).collect();
    let new = |list: &[(u32, u8)]| {
        let mut sl = slots();
        let mav = mavlink_device::Config::new(OWN_SYS, 1)
            .with_accept_list(list)
            .unwrap();
        RcPort::new(Config::new(), mav, &mut sl).err()
    };
    assert_eq!(new(&pairs[..14]), None);
    assert_eq!(new(&pairs[..15]), Some(NewError::AcceptListFull));
    assert_eq!(new(&pairs[..16]), Some(NewError::AcceptListFull));
    let mut both = pairs[..14].to_vec();
    both.extend([(70, 1), (109, 2)]);
    assert_eq!(
        new(&both),
        None,
        "the port's ids listed by the application count once"
    );
    let mut one = pairs[..15].to_vec();
    one.push((109, 2));
    assert_eq!(new(&one), Some(NewError::AcceptListFull));
}

/// The application listing RC_CHANNELS_OVERRIDE itself (with CRC_EXTRA 0, not its 124: the
/// port's wins) changes nothing: the same reports and counters, no `Hooks::mavlink` call.
#[test]
fn application_listing_rc_channels_override_changes_nothing() {
    let run = |mav: mavlink_device::Config| {
        let mut scen = Scen::default();
        let end = scen.mav_stream(MS, 600 * MS, Some(&MAV_CH));
        let mut sl = slots();
        let mut sim = Sim::with_mav(Config::new(), mav, &mut sl);
        sim.wire = scen.wire;
        sim.run_until(end / US + 1_000);
        (sim.hooks.reports, sim.port.stats(), sim.hooks.mavlink)
    };
    let plain = run(mavlink_device::Config::new(OWN_SYS, 1));
    let listed = run(mavlink_device::Config::new(OWN_SYS, 1)
        .with_accept_list(&[(70, 0)])
        .unwrap());
    assert!(plain.1.sticks > 20, "{:?}", plain.1);
    assert_eq!(listed.0, plain.0);
    assert_eq!(listed.1, plain.1);
    assert!(listed.2.is_empty() && plain.2.is_empty());
}

use mavlink_device::dialect::{MavParamType, PARAM_SET_DATA};
use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};

/// One REAL32 parameter, RC_TEST.
struct OneParam(f32);

impl ParamStore for OneParam {
    fn count(&self) -> u16 {
        1
    }
    fn name(&self, i: u16) -> Option<ParamName> {
        (i == 0).then(|| ParamName::new("RC_TEST").unwrap())
    }
    fn get(&self, i: u16) -> Option<ParamValue> {
        (i == 0).then_some(ParamValue::F32(self.0))
    }
    fn find(&self, n: &ParamName) -> Option<u16> {
        (n.as_str() == "RC_TEST").then_some(0)
    }
    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        match (i, v) {
            (0, ParamValue::F32(x)) => {
                self.0 = x;
                Ok(v)
            }
            _ => Err(ParamError::NotFound),
        }
    }
}

/// A PARAM_SET of RC_TEST to `value`, for this system and component.
fn param_set(seq: u8, value: f32) -> Vec<u8> {
    let mut id = [0u8; 16];
    id[..7].copy_from_slice(b"RC_TEST");
    let m = PARAM_SET_DATA {
        param_value: value,
        target_system: OWN_SYS,
        target_component: 1,
        param_id: id.into(),
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
    };
    mav(ELRS_SYS, ELRS_COMP, seq, &m)
}

/// No parameter write before the lock: a valid PARAM_SET heard while listening at 460800, among
/// the first two lock-on frames, does not change the store; the same PARAM_SET after the lock
/// does.
#[test]
fn param_set_applied_only_once_locked_on_mavlink() {
    let mut sl = slots();
    let mut rec = Rec::default();
    let (mut port, _) = port_listening(&mut sl, false, &mut rec);
    let mut store = OneParam(1.0);
    let radio = |seq| mav(ELRS_SYS, ELRS_COMP, seq, &radio_status(200, 100));
    port.receive(&param_set(1, 2.0), 500, &mut store, &mut rec);
    port.receive(&radio(2), 1_000, &mut store, &mut rec);
    port.receive(&param_set(3, 2.0), 1_500, &mut store, &mut rec);
    port.receive(&radio(4), 2_000, &mut store, &mut rec);
    assert_eq!(port.stats().lock_discarded, 2);
    assert_eq!(store.0, 1.0, "a PARAM_SET before the lock was applied");
    port.receive(&radio(5), 3_000, &mut store, &mut rec);
    assert_eq!(port.stats().mode, Mode::Mavlink);
    assert_eq!(store.0, 1.0);
    port.receive(&param_set(6, 2.0), 4_000, &mut store, &mut rec);
    assert_eq!(store.0, 2.0, "the PARAM_SET after the lock was not applied");
}
