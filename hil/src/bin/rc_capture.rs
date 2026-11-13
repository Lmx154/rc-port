//! rc_capture: a LISTEN-ONLY capture of the ExpressLRS receiver's line on UART0 RX (GPIO17, pad R0),
//! the first board step of the radio link (decision R2a). It answers facts-file section 6, items 1, 2,
//! 3 and 5, in a log that is judged afterwards: nothing is decided on the board, and there is no
//! PASS/FAIL line. It runs until the probe's timeout ends it.
//!
//! Listen only. Nothing is transmitted and no pin is driven:
//! - GPIO14 (UART0 TX, pad T0) is never configured: listener A is the RX-only `BufferedUartRx`.
//! - Listener B's state machine only reads its pin (`pio_uart::UartRx` sets no pin direction or level).
//! - The MAVLink node is only fed: `poll_transmit` is never called, so nothing it queues is sent.
//! - PIO0, PIO2 and GPIO36-39 (the four ESCs, DShot) are never touched; only PIO1 and GPIO17 are used.
//!
//! Two listeners on the same pin at once, so both protocols are seen without knowing which is on:
//! - A: UART0 RX on GPIO17, `BufferedUartRx` (interrupt-driven, no DMA), at `CRSF_BAUD` 8N1 (420000,
//!   or the build-time env var `RC_CAPTURE_CRSF_BAUD`), into `crsf_codec::Deframer`. The HAL's error
//!   kinds (framing, parity, break, overrun) are counted apart.
//! - B: `pio_uart::UartRx` on PIO1 state machine 0, GPIO17, at 460800 8N1, into a
//!   `mavlink_device::Node` whose accept list is RC_CHANNELS_OVERRIDE (70) and RADIO_STATUS (109).
//!   Its reader runs on an interrupt executor (SWI_IRQ_0) above thread mode: its FIFO holds 8 bytes,
//!   174 us at 460800.
//!
//! Pin sharing (the technique of dshot's `dshot_smoke`, README rule B9 there): a PIO block reads a
//! GPIO's input whatever the pad's function select is. GPIO17 is given to PIO1 first, with
//! `make_pio_pin` on a second handle of the pin (`PIN_17::steal`), and `UartRx::new` (which turns the
//! pad pull-up on); THEN `BufferedUartRx::new` takes the real handle, sets the function to UART0 RX
//! and writes the whole pad control word (embassy-rp 0.10: input enabled, isolation off, every other
//! field 0, rp-pac's default; so neither pull is on, by reading the code, as rc-port's own UART will
//! have it; the banner prints the pad as it is). The UART owns the pad from then on; PIO1 keeps
//! reading the input. Neither is ever dropped (PIO1's last drop would reset the function of every pin
//! it was given).
//!
//! Output, all through defmt, every line beginning with `t=` the board's time in ms:
//! - `[EVT]` when one of four streams (CRSF RC 0x16, CRSF link statistics 0x14, MAVLink
//!   RC_CHANNELS_OVERRIDE, MAVLink RADIO_STATUS) starts (first valid frame after 100 ms or more
//!   without one, with the gap) or stops (none for 100 ms, with the time of the last frame).
//! - `[SUM]` (counts per second) and `[VAL]` (latest values) once a second.
//! - `[WIN]` every 10 s: the four streams' totals over the last 10 s (the issue-3860 verdict is
//!   read from these: 0 RC_CHANNELS_OVERRIDE with more than 500 RADIO_STATUS = reproduced).
//!
//! The readers only count and decode; they publish their counters after each read, and the
//! formatting is done by the thread-mode logger.
#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use crsf_codec::{Deframer, Frame, raw_to_us};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::peripherals::{PIN_17, PIO1, UART0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUartRx};
use embassy_rp::{bind_interrupts, clocks, interrupt, pac};
use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::Read;
use marv_dialect::MessageData;
use mavlink_device::dialect::{RADIO_STATUS_DATA, RC_CHANNELS_OVERRIDE_DATA};
use mavlink_device::{Incoming, Node, ParamError, ParamName, ParamStore, ParamValue};
use panic_probe as _;
use pio_uart::UartRx;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

/// Listener A's baud: 420000 unless the build sets `RC_CAPTURE_CRSF_BAUD` (decimal).
const CRSF_BAUD: u32 = match option_env!("RC_CAPTURE_CRSF_BAUD") {
    Some(s) => parse_baud(s),
    None => 420_000,
};
/// Listener B's baud: the receiver's MAVLink rate.
const MAVLINK_BAUD: u32 = 460_800;
/// The GPIO both listeners hear.
const RX_GPIO: usize = 17;
/// UART receive ring: 8192 bytes are 195 ms of line at 420000 baud, so the thread-mode reader may be
/// held up by the logger for far longer than it ever is before the UART's 32-byte FIFO overruns.
const UART_RING_LEN: usize = 8192;
/// Bytes taken from the ring per read: short, so that one read's decoding is short.
const CRSF_READ_LEN: usize = 64;
/// Bytes taken from the PIO FIFO per read (it holds 8; bytes arriving during a read are taken too).
const MAV_READ_LEN: usize = 32;
/// A stream has stopped when no frame came for this long; the next frame starts it again.
const STOP_MS: u64 = 100;
/// The logger looks at the counters this often.
const TICK_MS: u64 = 10;
/// `[SUM]` and `[VAL]` every second, `[WIN]` every `WIN_SUMS` seconds.
const SUM_MS: u64 = 1000;
const WIN_SUMS: u32 = 10;
/// Listener B's interrupt executor: below every hardware interrupt (UART0 and PIO1 run at 0), above
/// thread mode, where the CRSF reader and the logger run.
const MAV_PRIORITY: u8 = 0xE0;
/// The node's own system id. 0 is no valid sender id, so the node's own-system filter drops no real
/// frame; the node never sends, so the id is used for nothing else.
const MAV_OWN_SYSTEM: u8 = 0;
/// `Stream::gap_ms` of the first start since power-up.
const FIRST: u64 = u64::MAX;

const ACCEPT: [(u32, u8); 2] = [
    (
        RC_CHANNELS_OVERRIDE_DATA::ID,
        RC_CHANNELS_OVERRIDE_DATA::EXTRA_CRC,
    ),
    (RADIO_STATUS_DATA::ID, RADIO_STATUS_DATA::EXTRA_CRC),
];

const MAV_CONFIG: mavlink_device::Config =
    match mavlink_device::Config::new(MAV_OWN_SYSTEM, 0).with_accept_list(&ACCEPT) {
        Ok(c) => c,
        Err(_) => panic!("the accept list of two entries was refused"),
    };

const fn parse_baud(s: &str) -> u32 {
    let b = s.as_bytes();
    assert!(!b.is_empty(), "RC_CAPTURE_CRSF_BAUD is empty");
    let mut v: u32 = 0;
    let mut i = 0;
    while i < b.len() {
        let d = b[i];
        assert!(
            d.is_ascii_digit(),
            "RC_CAPTURE_CRSF_BAUD must be decimal digits"
        );
        v = match v.checked_mul(10) {
            Some(x) => x,
            None => panic!("RC_CAPTURE_CRSF_BAUD is too large"),
        };
        v = match v.checked_add((d - b'0') as u32) {
            Some(x) => x,
            None => panic!("RC_CAPTURE_CRSF_BAUD is too large"),
        };
        i += 1;
    }
    assert!(v > 0, "RC_CAPTURE_CRSF_BAUD is 0");
    v
}

fn bump(c: &mut u32) {
    *c = c.wrapping_add(1);
}

/// One stream of frames, as its reader saw it.
#[derive(Clone, Copy)]
struct Stream {
    total: u32,
    /// Starts so far: first frames after `STOP_MS` or more without one, or the first ever.
    starts: u32,
    seen: bool,
    last_ms: u64,
    start_ms: u64,
    /// The gap before the last start (`FIRST` for the first since power-up).
    gap_ms: u64,
}

impl Stream {
    const NEW: Self = Self {
        total: 0,
        starts: 0,
        seen: false,
        last_ms: 0,
        start_ms: 0,
        gap_ms: FIRST,
    };

    fn frame(&mut self, now_ms: u64) {
        let gap = now_ms.saturating_sub(self.last_ms);
        if !self.seen || gap >= STOP_MS {
            bump(&mut self.starts);
            self.start_ms = now_ms;
            self.gap_ms = if self.seen { gap } else { FIRST };
        }
        self.seen = true;
        self.last_ms = now_ms;
        bump(&mut self.total);
    }
}

/// Listener A's counters (totals since power-up) and latest values.
#[derive(Clone, Copy)]
struct CrsfSide {
    bytes: u32,
    rc: Stream,
    ls: Stream,
    other: u32,
    crc: u32,
    badlen: u32,
    garbage: u32,
    framing: u32,
    parity: u32,
    brk: u32,
    overrun: u32,
    uart_other: u32,
    /// Channels 1-5, 15 and 16 in us.
    rc_us: [u16; 7],
    lq: u8,
    rssi1: u8,
    rssi2: u8,
    snr: i8,
    rf_mode: u8,
}

impl CrsfSide {
    const NEW: Self = Self {
        bytes: 0,
        rc: Stream::NEW,
        ls: Stream::NEW,
        other: 0,
        crc: 0,
        badlen: 0,
        garbage: 0,
        framing: 0,
        parity: 0,
        brk: 0,
        overrun: 0,
        uart_other: 0,
        rc_us: [0; 7],
        lq: 0,
        rssi1: 0,
        rssi2: 0,
        snr: 0,
        rf_mode: 0,
    };

    fn on_frame(&mut self, f: &Frame<'_>, now_ms: u64) {
        match f {
            Frame::RcChannels(rc) => {
                self.rc.frame(now_ms);
                let r = &rc.raw;
                self.rc_us = [
                    raw_to_us(r[0]),
                    raw_to_us(r[1]),
                    raw_to_us(r[2]),
                    raw_to_us(r[3]),
                    raw_to_us(r[4]),
                    raw_to_us(r[14]),
                    raw_to_us(r[15]),
                ];
            }
            Frame::LinkStatistics(ls) => {
                self.ls.frame(now_ms);
                self.lq = ls.uplink_link_quality;
                self.rssi1 = ls.uplink_rssi_1;
                self.rssi2 = ls.uplink_rssi_2;
                self.snr = ls.uplink_snr;
                self.rf_mode = ls.rf_mode;
            }
            _ => {}
        }
    }
}

/// Listener B's counters (totals since power-up) and latest values.
#[derive(Clone, Copy)]
struct MavSide {
    running: bool,
    bytes: u32,
    ovr: Stream,
    rst: Stream,
    hb: u32,
    unknown: u32,
    crc: u32,
    garbage: u32,
    mav1: u32,
    own: u32,
    pio_framing: u32,
    pio_overrun: u32,
    pio_runt: u32,
    /// Channels 1-5, 15 and 16 in us, as received.
    ovr_us: [u16; 7],
    ovr_src: (u8, u8),
    ovr_tgt: (u8, u8),
    txbuf: u8,
    rs_rssi: u8,
    rs_noise: u8,
    rs_remrssi: u8,
    rs_remnoise: u8,
    rs_src: (u8, u8),
}

impl MavSide {
    const NEW: Self = Self {
        running: false,
        bytes: 0,
        ovr: Stream::NEW,
        rst: Stream::NEW,
        hb: 0,
        unknown: 0,
        crc: 0,
        garbage: 0,
        mav1: 0,
        own: 0,
        pio_framing: 0,
        pio_overrun: 0,
        pio_runt: 0,
        ovr_us: [0; 7],
        ovr_src: (0, 0),
        ovr_tgt: (0, 0),
        txbuf: 0,
        rs_rssi: 0,
        rs_noise: 0,
        rs_remrssi: 0,
        rs_remnoise: 0,
        rs_src: (0, 0),
    };

    fn on_message(&mut self, m: &Incoming<'_>, now_ms: u64) {
        if m.message_id == RC_CHANNELS_OVERRIDE_DATA::ID {
            self.ovr.frame(now_ms);
            self.ovr_src = (m.system_id, m.component_id);
            if let Some(rc) = m.decode::<RC_CHANNELS_OVERRIDE_DATA>() {
                self.ovr_us = [
                    rc.chan1_raw,
                    rc.chan2_raw,
                    rc.chan3_raw,
                    rc.chan4_raw,
                    rc.chan5_raw,
                    rc.chan15_raw,
                    rc.chan16_raw,
                ];
                self.ovr_tgt = (rc.target_system, rc.target_component);
            }
        } else if m.message_id == RADIO_STATUS_DATA::ID {
            self.rst.frame(now_ms);
            self.rs_src = (m.system_id, m.component_id);
            if let Some(s) = m.decode::<RADIO_STATUS_DATA>() {
                self.txbuf = s.txbuf;
                self.rs_rssi = s.rssi;
                self.rs_noise = s.noise;
                self.rs_remrssi = s.remrssi;
                self.rs_remnoise = s.remnoise;
            }
        }
    }
}

static CRSF: Mutex<RefCell<CrsfSide>> = Mutex::new(RefCell::new(CrsfSide::NEW));
static MAV: Mutex<RefCell<MavSide>> = Mutex::new(RefCell::new(MavSide::NEW));

fn publish<T: Copy>(cell: &Mutex<RefCell<T>>, v: &T) {
    critical_section::with(|cs| *cell.borrow_ref_mut(cs) = *v);
}

fn snapshot<T: Copy>(cell: &Mutex<RefCell<T>>) -> T {
    critical_section::with(|cs| *cell.borrow_ref(cs))
}

/// The node needs a parameter store; this one has none (requests are answered into a queue that is
/// never sent).
struct NoParams;

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

#[embassy_executor::task]
async fn crsf_reader(mut rx: BufferedUartRx) -> ! {
    let mut deframer = Deframer::new();
    let mut side = CrsfSide::NEW;
    let mut buf = [0u8; CRSF_READ_LEN];
    loop {
        let r = rx.read(&mut buf).await;
        let now_ms = Instant::now().as_millis();
        match r {
            Ok(n) => {
                side.bytes = side.bytes.wrapping_add(n as u32);
                let bytes = buf.get(..n).unwrap_or(&[]);
                deframer.push(bytes, |f| side.on_frame(&f, now_ms));
            }
            Err(uart::Error::Framing) => bump(&mut side.framing),
            Err(uart::Error::Parity) => bump(&mut side.parity),
            Err(uart::Error::Break) => bump(&mut side.brk),
            Err(uart::Error::Overrun) => bump(&mut side.overrun),
            Err(_) => bump(&mut side.uart_other),
        }
        let st = deframer.stats();
        side.other = st.other_frames;
        side.crc = st.crc_errors;
        side.badlen = st.bad_length;
        side.garbage = st.garbage_bytes;
        publish(&CRSF, &side);
    }
}

#[embassy_executor::task]
async fn mav_reader(mut rx: UartRx<'static, PIO1, 0>) -> ! {
    let mut params = NoParams;
    let mut node = Node::new(MAV_CONFIG, &mut []);
    let mut side = MavSide::NEW;
    side.running = true;
    let mut buf = [0u8; MAV_READ_LEN];
    loop {
        // An overrun comes back as an error and is counted in `rx.stats()`.
        if let Ok(n) = rx.read(&mut buf).await {
            let now = Instant::now();
            let now_ms = now.as_millis();
            side.bytes = side.bytes.wrapping_add(n as u32);
            let bytes = buf.get(..n).unwrap_or(&[]);
            node.receive_with(bytes, now.as_micros(), &mut params, &mut |m| {
                side.on_message(&m, now_ms)
            });
        }
        let ps = rx.stats();
        side.pio_framing = ps.framing_errors;
        side.pio_overrun = ps.overruns;
        side.pio_runt = ps.runt_starts;
        let ns = node.stats();
        side.hb = ns.heartbeats_received;
        side.unknown = ns.dropped_unknown_message;
        side.crc = ns.dropped_checksum.wrapping_add(ns.dropped_listed_checksum);
        side.garbage = ns.garbage_bytes;
        side.mav1 = ns.dropped_mavlink1;
        side.own = ns.dropped_own_system;
        publish(&MAV, &side);
    }
}

static UART_RING: StaticCell<[u8; UART_RING_LEN]> = StaticCell::new();
static MAV_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    // SAFETY: the executor was started on this interrupt, and this is its only handler.
    unsafe { MAV_EXECUTOR.on_interrupt() }
}

fn line_level() -> u32 {
    (pac::SIO.gpio_in(0).read() >> RX_GPIO) & 1
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let clk_sys = clocks::clk_sys_freq();
    info!(
        "[INFO] rc_capture: listen only: nothing is transmitted; DShot pins untouched (PIO2 and GPIO36-39 not used, GPIO14 not configured)"
    );
    info!(
        "[INFO] rc_capture: A = UART0 RX (BufferedUartRx, no DMA) on GPIO17 at {=u32} baud 8N1 -> CRSF deframer; ring {=usize} bytes",
        CRSF_BAUD, UART_RING_LEN
    );

    // Listener B first: PIO1 gets GPIO17 before the UART takes the pad (module doc).
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO1, Irqs);
    // SAFETY: a second handle of GPIO17, for PIO1's input only. `UartRx` never sets the pin's
    // direction or level (pio-uart rule U3), and PIO1 runs no other program, so PIO1 never drives
    // it. The UART, built below with the real handle, then owns the pad's function and pad control.
    // Neither owner is ever dropped (`common` lives in `main`, which never returns; the readers loop
    // forever), so nothing resets the pin under the other.
    let pio_pin = common.make_pio_pin(unsafe { PIN_17::steal() });
    let pio_cfg = pio_uart::Config::new()
        .with_baud(MAVLINK_BAUD)
        .with_clk_sys_hz(clk_sys);
    let mav_rx = UartRx::new(&mut common, sm0, pio_pin, pio_cfg);

    // Listener A: the RX half alone; GPIO14 stays unconfigured.
    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = CRSF_BAUD;
    let ring = UART_RING.init([0; UART_RING_LEN]);
    let crsf_rx = BufferedUartRx::new(p.UART0, Irqs, p.PIN_17, ring, uart_cfg);

    match mav_rx {
        Ok(rx) => {
            let d = rx.divider();
            info!(
                "[INFO] rc_capture: B = PIO1 SM0 pio-uart UartRx (no DMA) on GPIO17 at {=u32} baud 8N1 -> MAVLink node (accepts 70, 109; never sends); divider {=u32}+{=u8}/256, {=i32} ppm, clk_sys {=u32} Hz",
                MAVLINK_BAUD,
                d.integer(),
                d.fraction(),
                d.error_ppm,
                clk_sys
            );
            // SAFETY: sets the priority of an interrupt no one else uses, before it is enabled.
            unsafe {
                let mut nvic = cortex_m::Peripherals::steal().NVIC;
                nvic.set_priority(interrupt::SWI_IRQ_0, MAV_PRIORITY);
            }
            let mav_spawner = MAV_EXECUTOR.start(interrupt::SWI_IRQ_0);
            match mav_reader(rx) {
                Ok(t) => mav_spawner.spawn(t),
                Err(_) => info!("[INFO] rc_capture: B: the reader task could not be spawned"),
            }
        }
        Err(e) => info!(
            "[INFO] rc_capture: B REFUSED by pio-uart at {=u32} baud, clk_sys {=u32} Hz: {}; only A runs",
            MAVLINK_BAUD, clk_sys, e.reason
        ),
    }
    match crsf_reader(crsf_rx) {
        Ok(t) => spawner.spawn(t),
        Err(_) => info!("[INFO] rc_capture: A: the reader task could not be spawned"),
    }

    let pad = pac::PADS_BANK0.gpio(RX_GPIO).read();
    let ctrl = pac::IO_BANK0.gpio(RX_GPIO).ctrl().read();
    info!(
        "[INFO] rc_capture: GPIO17 after setup: funcsel {=u8} (2 = UART0), pull-down {=bool}, pull-up {=bool}, input enabled {=bool}, isolated {=bool}, level {=u32}",
        ctrl.funcsel(),
        pad.pde(),
        pad.pue(),
        pad.ie(),
        pad.iso(),
        line_level()
    );
    info!(
        "[INFO] rc_capture: [EVT] on start/stop (stop = none for {=u64} ms), [SUM] counts per second, [VAL] latest values, [WIN] totals of the last 10 s; the log is judged afterwards",
        STOP_MS
    );

    // `common` is held here for ever: dropping PIO1's last part would reset GPIO17's function.
    let _common = common;
    run_logger().await
}

/// What the logger last reported of one stream.
struct Watch {
    name: &'static str,
    seen_starts: u32,
    running: bool,
}

impl Watch {
    const fn new(name: &'static str) -> Self {
        Self {
            name,
            seen_starts: 0,
            running: false,
        }
    }

    /// Prints the stream's events since the last look; true when it printed a timed stop.
    fn look(&mut self, s: &Stream, now_ms: u64) -> bool {
        if s.starts != self.seen_starts {
            let unseen = s.starts.wrapping_sub(self.seen_starts).wrapping_sub(1);
            if self.running {
                info!(
                    "[EVT] t={=u64} {=str} stopped last={=u64}",
                    now_ms,
                    self.name,
                    s.start_ms.saturating_sub(s.gap_ms)
                );
            }
            if unseen > 0 {
                info!(
                    "[EVT] t={=u64} {=str} {=u32} more start/stop pairs came between two looks",
                    now_ms, self.name, unseen
                );
            }
            if s.gap_ms == FIRST {
                info!(
                    "[EVT] t={=u64} {=str} started at={=u64} first since power-up",
                    now_ms, self.name, s.start_ms
                );
            } else {
                info!(
                    "[EVT] t={=u64} {=str} started at={=u64} gap_ms={=u64}",
                    now_ms, self.name, s.start_ms, s.gap_ms
                );
            }
            self.seen_starts = s.starts;
            self.running = true;
        }
        if self.running && now_ms.saturating_sub(s.last_ms) >= STOP_MS {
            info!(
                "[EVT] t={=u64} {=str} stopped last={=u64} ran_ms={=u64}",
                now_ms,
                self.name,
                s.last_ms,
                s.last_ms.saturating_sub(s.start_ms)
            );
            self.running = false;
            return true;
        }
        false
    }
}

fn d(now: u32, before: u32) -> u32 {
    now.wrapping_sub(before)
}

async fn run_logger() -> ! {
    let mut watches = [
        Watch::new("crsf_rc"),
        Watch::new("crsf_ls"),
        Watch::new("mav_ovr"),
        Watch::new("mav_rst"),
    ];
    let mut sum_c = snapshot(&CRSF);
    let mut sum_m = snapshot(&MAV);
    let mut win_c = sum_c;
    let mut win_m = sum_m;
    let mut sums = 0u32;
    let mut next_sum = Instant::now().as_millis() + SUM_MS;
    let mut ticker = Ticker::every(Duration::from_millis(TICK_MS));
    loop {
        ticker.next().await;
        let c = snapshot(&CRSF);
        let m = snapshot(&MAV);
        let now_ms = Instant::now().as_millis();
        for (i, (w, s)) in watches
            .iter_mut()
            .zip([&c.rc, &c.ls, &m.ovr, &m.rst])
            .enumerate()
        {
            if w.look(s, now_ms) && i == 1 {
                // Observation 1: whether the last link statistics before a loss carries LQ 0.
                info!(
                    "[EVT] t={=u64} crsf_ls last frame: lq={=u8} rssi=-{=u8}/-{=u8} snr={=i8} rf={=u8}",
                    now_ms, c.lq, c.rssi1, c.rssi2, c.snr, c.rf_mode
                );
            }
        }
        if now_ms < next_sum {
            continue;
        }
        next_sum += SUM_MS;
        info!(
            "[SUM] t={=u64} crsf: bytes={=u32} rc={=u32} ls={=u32} other={=u32} crc={=u32} badlen={=u32} garb={=u32} uart(fr/pa/br/ov)={=u32}/{=u32}/{=u32}/{=u32} | mav: bytes={=u32} ovr={=u32} rst={=u32} hb={=u32} unk={=u32} crc={=u32} garb={=u32} v1={=u32} pio(fr/ov/runt)={=u32}/{=u32}/{=u32}",
            now_ms,
            d(c.bytes, sum_c.bytes),
            d(c.rc.total, sum_c.rc.total),
            d(c.ls.total, sum_c.ls.total),
            d(c.other, sum_c.other),
            d(c.crc, sum_c.crc),
            d(c.badlen, sum_c.badlen),
            d(c.garbage, sum_c.garbage),
            d(c.framing, sum_c.framing),
            d(c.parity, sum_c.parity),
            d(c.brk, sum_c.brk),
            d(c.overrun, sum_c.overrun),
            d(m.bytes, sum_m.bytes),
            d(m.ovr.total, sum_m.ovr.total),
            d(m.rst.total, sum_m.rst.total),
            d(m.hb, sum_m.hb),
            d(m.unknown, sum_m.unknown),
            d(m.crc, sum_m.crc),
            d(m.garbage, sum_m.garbage),
            d(m.mav1, sum_m.mav1),
            d(m.pio_framing, sum_m.pio_framing),
            d(m.pio_overrun, sum_m.pio_overrun),
            d(m.pio_runt, sum_m.pio_runt)
        );
        info!(
            "[VAL] t={=u64} line={=u32} rc_us={=[?]} lq={=u8} rssi=-{=u8}/-{=u8} snr={=i8} rf={=u8} | ovr_us={=[?]} ovr_src={=u8}/{=u8} ovr_tgt={=u8}/{=u8} txbuf={=u8} rs_rssi={=u8} rs_noise={=u8} rs_rem={=u8}/{=u8} rs_src={=u8}/{=u8} B={=bool}",
            now_ms,
            line_level(),
            &c.rc_us[..],
            c.lq,
            c.rssi1,
            c.rssi2,
            c.snr,
            c.rf_mode,
            &m.ovr_us[..],
            m.ovr_src.0,
            m.ovr_src.1,
            m.ovr_tgt.0,
            m.ovr_tgt.1,
            m.txbuf,
            m.rs_rssi,
            m.rs_noise,
            m.rs_remrssi,
            m.rs_remnoise,
            m.rs_src.0,
            m.rs_src.1,
            m.running
        );
        if c.uart_other != sum_c.uart_other || m.own != sum_m.own {
            info!(
                "[SUM] t={=u64} rare: uart other errors={=u32} mav own-system frames={=u32}",
                now_ms,
                d(c.uart_other, sum_c.uart_other),
                d(m.own, sum_m.own)
            );
        }
        sum_c = c;
        sum_m = m;
        sums += 1;
        if sums.is_multiple_of(WIN_SUMS) {
            let uart_err = d(c.framing, win_c.framing)
                .wrapping_add(d(c.parity, win_c.parity))
                .wrapping_add(d(c.brk, win_c.brk))
                .wrapping_add(d(c.overrun, win_c.overrun));
            info!(
                "[WIN] t={=u64} last 10 s: crsf_rc={=u32} crsf_ls={=u32} mav_ovr={=u32} mav_rst={=u32} | mav_hb={=u32} crsf_bytes={=u32} mav_bytes={=u32} uart_err={=u32} uart_brk={=u32} uart_ov={=u32} pio_fr={=u32} pio_ov={=u32}",
                now_ms,
                d(c.rc.total, win_c.rc.total),
                d(c.ls.total, win_c.ls.total),
                d(m.ovr.total, win_m.ovr.total),
                d(m.rst.total, win_m.rst.total),
                d(m.hb, win_m.hb),
                d(c.bytes, win_c.bytes),
                d(m.bytes, win_m.bytes),
                uart_err,
                d(c.brk, win_c.brk),
                d(c.overrun, win_c.overrun),
                d(m.pio_framing, win_m.pio_framing),
                d(m.pio_overrun, win_m.pio_overrun)
            );
            win_c = c;
            win_m = m;
        }
    }
}
