//! rc_follow: `rc_port::RcPort::run` on the real UART0, following the ExpressLRS receiver between
//! CRSF (420000) and MAVLink 2 (460800), and LOGGING what the port reports, so that milestones R2,
//! R3, R4 and R5 of the radio-link decision can be judged from the log afterwards. Nothing is
//! decided on the board: no PASS/FAIL line, no failsafe, no loss decision, no arming. It runs until
//! the probe's timeout ends it.
//!
//! What it transmits, and nothing else: the CRSF battery frame of `Hooks::crsf_frame` while locked on
//! CRSF (a FIXED value from the build, `RC_FOLLOW_BATT_DV` and `RC_FOLLOW_BATT_PCT`; the ADC is not
//! read), and the MAVLink node's own messages while locked on MAVLink (system 1, component 1, no
//! streams). It never binds, flashes or writes the receiver's settings.
//!
//! Pins and peripherals: UART0 only, `BufferedUart` (interrupt-driven, no DMA), TX on GPIO14 (pad
//! T0, the only pin driven) and RX on GPIO17 (pad R0, pull-up turned on after the UART is built).
//! PIO0, PIO1, PIO2, GPIO36-39 (the four ESCs, DShot), the ADC and every DMA channel are never
//! touched.
//!
//! Tasks: the port's task runs on an interrupt executor (`SWI_IRQ_0`, NVIC priority 0xE0, below the
//! UART0 and timer interrupts, above thread mode); its `Hooks` only copy into one
//! `critical_section::Mutex` snapshot, never block and never format. The logger runs in thread mode,
//! looks at the snapshot every 10 ms and does all the formatting.
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use defmt::info;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_rp::{bind_interrupts, interrupt, pac};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use panic_probe as _;
use rc_port::crsf_codec::{Battery, MAX_FRAME_LEN, encode_battery};
use rc_port::mavlink_device::{
    self, Outgoing, ParamError, ParamName, ParamStore, ParamValue, Telemetry,
};
use rc_port::{
    Config, CrsfLinkReport, Hooks, LOCK_FRAMES, MAVLINK_BAUD, Mode, RadioStatusReport, RcPort,
    Report, RunError, SetBaud, Source, Stats, StickReport, T_LEAVE_CRSF_US, TxBusy,
};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

/// The port's numbers: the defaults (`crsf_tx_interval_us` included).
const PORT_CONFIG: Config = Config::new();
/// The MAVLink node: system 1, component 1 (decision section 5), defaults otherwise; the
/// application's accept list is empty (the port adds RC_CHANNELS_OVERRIDE and RADIO_STATUS).
const MAV_CONFIG: mavlink_device::Config = mavlink_device::Config::new(1, 1);
/// UART receive ring. `at_us` is the time the bytes are READ (rc-port README, "Reports"), so the ring
/// bounds how many stale frames one stall can deliver with a fresh time: 256 bytes are about 9 RC
/// frames at 250 Hz, 6.1 ms of line at 420000 baud (5.6 ms at 460800). The port's task runs above
/// thread mode and reads 64 bytes at a time, so it is not held up by the logger.
const RX_BUF_LEN: usize = 256;
/// UART transmit ring: more than a whole MAVLink frame (at most 280 bytes) is not needed, since the
/// port waits for each write to be taken.
const TX_BUF_LEN: usize = 256;
const RX_GPIO: usize = 17;
const TX_GPIO: usize = 14;
/// Stick reports have stopped when none came for this long; the next one starts them again.
const STOP_US: u64 = 100_000;
/// The logger looks at the snapshot this often.
const TICK_MS: u64 = 10;
/// `[STK]`, `[LNK]` and `[CNT]` every second.
const SUM_MS: u64 = 1000;
/// Pause before `run` is called again after it returned.
const RESTART_MS: u64 = 100;
/// The port's interrupt executor: below every hardware interrupt (UART0 and the timer run at 0),
/// above thread mode, where the logger runs.
const PORT_PRIORITY: u8 = 0xE0;
/// Mode changes kept for the logger between two looks.
const MODE_RING: usize = 8;
/// `Run::gap_us` of the first start since power-up.
const FIRST: u64 = u64::MAX;

/// The fixed battery value sent as CRSF telemetry: decivolts, default 123 (12.3 V).
const BATT_DV: u16 = match option_env!("RC_FOLLOW_BATT_DV") {
    Some(s) => parse_dec(s, u16::MAX as u32) as u16,
    None => 123,
};
/// Remaining percent, default 50.
const BATT_PCT: u8 = match option_env!("RC_FOLLOW_BATT_PCT") {
    Some(s) => parse_dec(s, 100) as u8,
    None => 50,
};
const BATTERY: Battery = Battery {
    voltage_dv: BATT_DV,
    current_da: 0,
    capacity_used_mah: 0,
    remaining_percent: BATT_PCT,
};

const fn parse_dec(s: &str, max: u32) -> u32 {
    let b = s.as_bytes();
    assert!(!b.is_empty(), "RC_FOLLOW_BATT_* is empty");
    let mut v: u32 = 0;
    let mut i = 0;
    while i < b.len() {
        let d = b[i];
        assert!(
            d.is_ascii_digit(),
            "RC_FOLLOW_BATT_* must be decimal digits"
        );
        v = match v.checked_mul(10) {
            Some(x) => x,
            None => panic!("RC_FOLLOW_BATT_* is too large"),
        };
        v = match v.checked_add((d - b'0') as u32) {
            Some(x) => x,
            None => panic!("RC_FOLLOW_BATT_* is too large"),
        };
        i += 1;
    }
    assert!(
        v <= max,
        "RC_FOLLOW_BATT_DV must be at most 65535, RC_FOLLOW_BATT_PCT at most 100"
    );
    v
}

// ---------------------------------------------------------------------------------------------
// The UART as the port sees it: BufferedUart with every byte written counted by baud rate.
// ---------------------------------------------------------------------------------------------

/// Bytes the UART took at the CRSF baud and at 460800. The port writes only while locked, in the
/// locked protocol at its baud, so these are the bytes written in CRSF mode and in MAVLink mode.
static TX_CRSF: AtomicU32 = AtomicU32::new(0);
static TX_MAV: AtomicU32 = AtomicU32::new(0);

/// Forwards everything to the `BufferedUart` (read, write, flush, `SetBaud`, `TxBusy`), and counts
/// the bytes each write took.
struct CountingUart {
    uart: BufferedUart,
    baud: u32,
}

impl embedded_io_async::ErrorType for CountingUart {
    type Error = uart::Error;
}

impl Read for CountingUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, uart::Error> {
        self.uart.read(buf).await
    }
}

impl Write for CountingUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, uart::Error> {
        let r = self.uart.write(buf).await;
        if let Ok(n) = r {
            let counter = if self.baud == MAVLINK_BAUD {
                &TX_MAV
            } else {
                &TX_CRSF
            };
            counter.fetch_add(n as u32, Ordering::Relaxed);
        }
        r
    }

    async fn flush(&mut self) -> Result<(), uart::Error> {
        self.uart.flush().await
    }
}

impl SetBaud for CountingUart {
    fn set_baud(&mut self, baud: u32) {
        self.uart.set_baud(baud);
        self.baud = baud;
    }
}

impl TxBusy for CountingUart {
    fn tx_busy(&self) -> bool {
        self.uart.tx_busy()
    }
}

// ---------------------------------------------------------------------------------------------
// The node's parameters (two harmless test values in RAM) and telemetry (no streams).
// ---------------------------------------------------------------------------------------------

const PARAM_NAMES: [&str; 2] = ["RCF_TEST_I", "RCF_TEST_F"];

struct Params {
    values: [ParamValue; 2],
}

impl Params {
    const fn new() -> Self {
        Self {
            values: [ParamValue::I32(42), ParamValue::F32(1.5)],
        }
    }
}

impl ParamStore for Params {
    fn count(&self) -> u16 {
        PARAM_NAMES.len() as u16
    }

    fn name(&self, i: u16) -> Option<ParamName> {
        ParamName::new(PARAM_NAMES.get(usize::from(i))?).ok()
    }

    fn get(&self, i: u16) -> Option<ParamValue> {
        self.values.get(usize::from(i)).copied()
    }

    fn find(&self, n: &ParamName) -> Option<u16> {
        let i = PARAM_NAMES.iter().position(|p| *p == n.as_str())?;
        u16::try_from(i).ok()
    }

    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        let slot = self
            .values
            .get_mut(usize::from(i))
            .ok_or(ParamError::NotFound)?;
        *slot = v;
        Ok(v)
    }
}

/// No streams: the node sends only its HEARTBEAT and its replies.
struct NoTelemetry;

impl Telemetry for NoTelemetry {
    fn sample(&mut self, _: u32, _: u64, _: &mut Outgoing<'_>) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------------------------
// The snapshot: written by the hooks in the port's task, read by the logger.
// ---------------------------------------------------------------------------------------------

/// One change of `Stats::mode`, as the stats hook saw it.
#[derive(Clone, Copy)]
struct ModeEvt {
    mode: Mode,
    since_us: u64,
    /// The board's time when the stats hook saw the change.
    seen_us: u64,
    changes: u32,
    prev: Mode,
    prev_since_us: u64,
    /// `lock_discarded` increments during the previous mode.
    discarded: u32,
    /// When the stats hook saw the last two `lock_discarded` increments (older first).
    disc_at: [Option<u64>; 2],
    last_sticks_us: Option<u64>,
}

impl ModeEvt {
    const NEW: Self = Self {
        mode: Mode::Searching,
        since_us: 0,
        seen_us: 0,
        changes: 0,
        prev: Mode::Searching,
        prev_since_us: 0,
        discarded: 0,
        disc_at: [None; 2],
        last_sticks_us: None,
    };
}

/// The first stick report after a lock.
#[derive(Clone, Copy)]
struct FirstStick {
    count: u32,
    mode: Mode,
    at_us: u64,
    mode_since_us: u64,
    /// When the first of the two discarded frames before the lock was seen.
    disc1_us: Option<u64>,
}

/// One run of stick reports: starts after `STOP_US` or more without one.
#[derive(Clone, Copy)]
struct Run {
    starts: u32,
    seen: bool,
    last_us: u64,
    start_us: u64,
    gap_us: u64,
}

impl Run {
    const NEW: Self = Self {
        starts: 0,
        seen: false,
        last_us: 0,
        start_us: 0,
        gap_us: FIRST,
    };

    /// Takes one report; returns the interval from the previous one when both are in one run.
    fn report(&mut self, at_us: u64) -> Option<u64> {
        let gap = at_us.saturating_sub(self.last_us);
        let interval = if !self.seen || gap >= STOP_US {
            self.starts = self.starts.wrapping_add(1);
            self.start_us = at_us;
            self.gap_us = if self.seen { gap } else { FIRST };
            None
        } else {
            Some(gap)
        };
        self.seen = true;
        self.last_us = at_us;
        interval
    }
}

/// Stick reports since the logger's last second.
#[derive(Clone, Copy)]
struct Window {
    n: u32,
    intervals: u32,
    iv_min_us: u64,
    iv_max_us: u64,
}

impl Window {
    const NEW: Self = Self {
        n: 0,
        intervals: 0,
        iv_min_us: 0,
        iv_max_us: 0,
    };

    fn interval(&mut self, iv: u64) {
        if self.intervals == 0 {
            self.iv_min_us = iv;
            self.iv_max_us = iv;
        } else {
            self.iv_min_us = self.iv_min_us.min(iv);
            self.iv_max_us = self.iv_max_us.max(iv);
        }
        self.intervals = self.intervals.wrapping_add(1);
    }
}

#[derive(Clone, Copy)]
struct Shared {
    /// The latest counters (`None` until the first stats call).
    stats: Option<Stats>,
    evts: [ModeEvt; MODE_RING],
    evt_count: u32,
    disc_in_mode: u32,
    disc_at: [Option<u64>; 2],
    run: Run,
    win: Window,
    stick: Option<StickReport>,
    /// `at_us` of the first stick report since the last stats call.
    stick_since_stats: Option<u64>,
    first_wanted: bool,
    first: FirstStick,
    crsf_link: Option<CrsfLinkReport>,
    radio: Option<RadioStatusReport>,
    restarts: u32,
    err: Option<RunError<uart::Error>>,
    err_slowdown_ms: u32,
}

impl Shared {
    const NEW: Self = Self {
        stats: None,
        evts: [ModeEvt::NEW; MODE_RING],
        evt_count: 0,
        disc_in_mode: 0,
        disc_at: [None; 2],
        run: Run::NEW,
        win: Window::NEW,
        stick: None,
        stick_since_stats: None,
        first_wanted: false,
        first: FirstStick {
            count: 0,
            mode: Mode::Searching,
            at_us: 0,
            mode_since_us: 0,
            disc1_us: None,
        },
        crsf_link: None,
        radio: None,
        restarts: 0,
        err: None,
        err_slowdown_ms: 0,
    };

    fn first_stick(&mut self, at_us: u64, mode: Mode, mode_since_us: u64) {
        self.first = FirstStick {
            count: self.first.count.wrapping_add(1),
            mode,
            at_us,
            mode_since_us,
            disc1_us: self.disc_at[0],
        };
    }

    fn on_report(&mut self, report: Report) {
        match report {
            Report::Sticks(s) => {
                if self.stick_since_stats.is_none() {
                    self.stick_since_stats = Some(s.at_us);
                }
                if self.first_wanted {
                    self.first_wanted = false;
                    let st = self.stats.unwrap_or_default();
                    self.first_stick(s.at_us, st.mode, st.mode_since_us);
                }
                if let Some(iv) = self.run.report(s.at_us) {
                    self.win.interval(iv);
                }
                self.win.n = self.win.n.wrapping_add(1);
                self.stick = Some(s);
            }
            Report::CrsfLink(l) => self.crsf_link = Some(l),
            Report::RadioStatus(r) => self.radio = Some(r),
            _ => {}
        }
    }

    fn on_stats(&mut self, s: &Stats, now_us: u64) {
        let prev = self.stats.unwrap_or_default();
        let d = s.lock_discarded.wrapping_sub(prev.lock_discarded);
        if d > 0 {
            self.disc_in_mode = self.disc_in_mode.saturating_add(d);
            self.disc_at = if d >= 2 {
                [Some(now_us); 2]
            } else {
                [self.disc_at[1], Some(now_us)]
            };
        }
        if s.mode != prev.mode {
            let evt = ModeEvt {
                mode: s.mode,
                since_us: s.mode_since_us,
                seen_us: now_us,
                changes: s.mode_changes,
                prev: prev.mode,
                prev_since_us: prev.mode_since_us,
                discarded: self.disc_in_mode,
                disc_at: self.disc_at,
                last_sticks_us: s.last_sticks_us,
            };
            if let Some(slot) = self.evts.get_mut(self.evt_count as usize % MODE_RING) {
                *slot = evt;
            }
            self.evt_count = self.evt_count.wrapping_add(1);
            self.disc_in_mode = 0;
            self.first_wanted = false;
            if s.mode != Mode::Searching {
                match self.stick_since_stats {
                    Some(at) => self.first_stick(at, s.mode, s.mode_since_us),
                    None => self.first_wanted = true,
                }
            }
        }
        self.stick_since_stats = None;
        self.stats = Some(*s);
    }
}

static SHARED: Mutex<RefCell<Shared>> = Mutex::new(RefCell::new(Shared::NEW));

/// The application's side of the port: copies into the snapshot, and the fixed battery frame.
struct Bench;

impl Hooks for Bench {
    fn report(&mut self, report: Report) {
        critical_section::with(|cs| SHARED.borrow_ref_mut(cs).on_report(report));
    }

    fn crsf_frame(&mut self, _now_us: u64, out: &mut [u8; MAX_FRAME_LEN]) -> Option<usize> {
        encode_battery(&BATTERY, out).ok()
    }

    fn stats(&mut self, stats: &Stats) {
        let now_us = Instant::now().as_micros();
        critical_section::with(|cs| SHARED.borrow_ref_mut(cs).on_stats(stats, now_us));
    }
}

#[embassy_executor::task]
async fn port_task(mut port: RcPort<'static>, mut uart: CountingUart) -> ! {
    let mut params = Params::new();
    let mut tel = NoTelemetry;
    let mut hooks = Bench;
    loop {
        let e = port
            .run(
                &mut uart,
                &mut Delay,
                || Instant::now().as_micros(),
                &mut params,
                &mut tel,
                &mut hooks,
            )
            .await;
        let slowdown = port.node().stream_slowdown_ms();
        critical_section::with(|cs| {
            let mut sh = SHARED.borrow_ref_mut(cs);
            sh.restarts = sh.restarts.wrapping_add(1);
            sh.err = Some(e);
            sh.err_slowdown_ms = slowdown;
        });
        Timer::after_millis(RESTART_MS).await;
    }
}

static TX_BUF: StaticCell<[u8; TX_BUF_LEN]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; RX_BUF_LEN]> = StaticCell::new();
static PORT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    // SAFETY: the executor was started on this interrupt, and this is its only handler.
    unsafe { PORT_EXECUTOR.on_interrupt() }
}

fn line_level() -> u32 {
    (pac::SIO.gpio_in(0).read() >> RX_GPIO) & 1
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Built at 460800: the port's first actions set 460800 whatever the UART was built with.
    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = MAVLINK_BAUD;
    let tx_buf = TX_BUF.init([0; TX_BUF_LEN]);
    let rx_buf = RX_BUF.init([0; RX_BUF_LEN]);
    let uart = BufferedUart::new(p.UART0, p.PIN_14, p.PIN_17, Irqs, tx_buf, rx_buf, uart_cfg);
    // The HAL writes GPIO17's whole pad word with only input enable set (no pull). An idle UART line
    // is high: turn the pull-up on, every other bit of the pad word kept as read. Nothing else
    // writes this pad afterwards (`set_baudrate` touches only the UART's registers).
    pac::PADS_BANK0.gpio(RX_GPIO).modify(|w| w.set_pue(true));

    info!(
        "[INFO] rc_follow: RcPort::run on UART0 (BufferedUart, no DMA), TX GPIO14 (T0, driven), RX GPIO17 (R0); rings rx {=usize} / tx {=usize} bytes; built at {=u32}, then the port sets the baud",
        RX_BUF_LEN, TX_BUF_LEN, MAVLINK_BAUD
    );
    info!(
        "[INFO] rc_follow: transmits only CRSF battery telemetry (CRSF mode) and the MAVLink node's own messages (MAVLink mode); never binds, flashes or writes settings; DShot pins untouched (PIO2 and GPIO36-39 not used)"
    );
    info!(
        "[INFO] rc_follow: Config: crsf_baud={=u32} mavlink_baud={=u32} lock_frames={=u8} t_leave_crsf_us={=u32} t_leave_mav_us={=u32} d_mav_us={=u32} d_crsf_us={=u32} crsf_tx_interval_us={=u32}",
        PORT_CONFIG.crsf_baud,
        MAVLINK_BAUD,
        LOCK_FRAMES,
        T_LEAVE_CRSF_US,
        PORT_CONFIG.t_leave_mav_us,
        PORT_CONFIG.d_mav_us,
        PORT_CONFIG.d_crsf_us,
        PORT_CONFIG.crsf_tx_interval_us
    );
    info!(
        "[INFO] rc_follow: CRSF battery telemetry is a FIXED value: {=u16}.{=u16} V ({=u16} dV), 0 A, 0 mAh, {=u8} % (build-time RC_FOLLOW_BATT_DV, RC_FOLLOW_BATT_PCT); the ADC is not read",
        BATT_DV / 10,
        BATT_DV % 10,
        BATT_DV,
        BATT_PCT
    );
    info!(
        "[INFO] rc_follow: MAVLink node system 1 component 1, accept list empty (the port adds 70 and 109), params RCF_TEST_I=42 RCF_TEST_F=1.5, no streams; stream_slowdown_ms cannot be read while run is awaited: printed on [ERR] only"
    );

    let pad = pac::PADS_BANK0.gpio(RX_GPIO).read();
    let ctrl = pac::IO_BANK0.gpio(RX_GPIO).ctrl().read();
    let tx_ctrl = pac::IO_BANK0.gpio(TX_GPIO).ctrl().read();
    info!(
        "[INFO] rc_follow: GPIO17 after setup: funcsel {=u8} (2 = UART0), pull-down {=bool}, pull-up {=bool}, input enabled {=bool}, isolated {=bool}, level {=u32}; GPIO14 funcsel {=u8} (2 = UART0)",
        ctrl.funcsel(),
        pad.pde(),
        pad.pue(),
        pad.ie(),
        pad.iso(),
        line_level(),
        tx_ctrl.funcsel()
    );

    let port = match RcPort::new(PORT_CONFIG, MAV_CONFIG, &mut []) {
        Ok(port) => port,
        Err(_) => {
            info!("[INFO] rc_follow: RcPort::new refused the configuration; nothing runs");
            loop {
                Timer::after_secs(3600).await;
            }
        }
    };
    let uart = CountingUart {
        uart,
        baud: MAVLINK_BAUD,
    };

    // SAFETY: sets the priority of an interrupt no one else uses, before it is enabled.
    unsafe {
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(interrupt::SWI_IRQ_0, PORT_PRIORITY);
    }
    let port_spawner = PORT_EXECUTOR.start(interrupt::SWI_IRQ_0);
    match port_task(port, uart) {
        Ok(t) => port_spawner.spawn(t),
        Err(_) => info!("[INFO] rc_follow: the port's task could not be spawned"),
    }
    info!(
        "[INFO] rc_follow: port task on SWI_IRQ_0 at NVIC priority 0xE0, logger in thread mode; [MODE] on each mode change, [EDGE] stick reports start/stop (stop = none for {=u64} ms) and first after a lock, [STK] [LNK] [CNT] every second, [ERR] if run returns; the log is judged afterwards",
        STOP_US / 1000
    );
    info!("[MODE] t=0 starts Searching (460800 first)");
    run_logger().await
}

// ---------------------------------------------------------------------------------------------
// The logger: all formatting, in thread mode.
// ---------------------------------------------------------------------------------------------

fn mode_name(m: Mode) -> &'static str {
    match m {
        Mode::Searching => "Searching",
        Mode::Crsf => "Crsf",
        Mode::Mavlink => "Mavlink",
    }
}

/// The time without a valid frame after which a lock in `m` is left.
fn leave_us(m: Mode) -> Option<u64> {
    match m {
        Mode::Crsf => Some(u64::from(T_LEAVE_CRSF_US)),
        Mode::Mavlink => Some(u64::from(PORT_CONFIG.t_leave_mav_us)),
        Mode::Searching => None,
    }
}

fn d(now: u32, before: u32) -> u32 {
    now.wrapping_sub(before)
}

/// Prints one mode change. `old_last_frame_us` is the last valid frame of the lock left before this
/// one (from the previous `Searching` line); returns it for the next line.
fn print_mode(e: &ModeEvt, now_ms: u64, old_last_frame_us: Option<u64>) -> Option<u64> {
    let prev_lasted_us = e.since_us.saturating_sub(e.prev_since_us);
    if e.mode == Mode::Searching {
        // `mode_since_us` of a left lock is its last valid frame plus the leave time.
        let last_frame_us = leave_us(e.prev).map(|l| e.since_us.saturating_sub(l));
        info!(
            "[MODE] t={=u64} -> Searching since_us={=u64} seen_us={=u64} changes={=u32} prev={=str} prev_lasted_us={=u64} last_valid_frame_us={} last_stick_us={} seen_after_last_stick_us={}",
            now_ms,
            e.since_us,
            e.seen_us,
            e.changes,
            mode_name(e.prev),
            prev_lasted_us,
            last_frame_us,
            e.last_sticks_us,
            e.last_sticks_us.map(|t| e.seen_us.saturating_sub(t))
        );
        last_frame_us
    } else {
        info!(
            "[MODE] t={=u64} -> {=str} since_us={=u64} seen_us={=u64} changes={=u32} prev={=str} prev_lasted_us={=u64} discarded={=u32} discard1_to_lock_us={} discard2_to_lock_us={} old_last_frame_to_lock_us={}",
            now_ms,
            mode_name(e.mode),
            e.since_us,
            e.seen_us,
            e.changes,
            mode_name(e.prev),
            prev_lasted_us,
            e.discarded,
            e.disc_at[0].map(|t| e.since_us.saturating_sub(t)),
            e.disc_at[1].map(|t| e.since_us.saturating_sub(t)),
            old_last_frame_us.map(|t| e.since_us.saturating_sub(t))
        );
        None
    }
}

async fn run_logger() -> ! {
    let mut seen_evts = 0u32;
    let mut old_last_frame_us: Option<u64> = None;
    let mut seen_first = 0u32;
    let mut seen_starts = 0u32;
    let mut running = false;
    let mut seen_restarts = 0u32;
    let mut sum = Stats::default();
    let mut sum_tx = (0u32, 0u32);
    let mut next_sum = Instant::now().as_millis() + SUM_MS;
    let mut ticker = Ticker::every(Duration::from_millis(TICK_MS));
    loop {
        ticker.next().await;
        let now_ms = Instant::now().as_millis();
        let second = now_ms >= next_sum;
        let sh = critical_section::with(|cs| {
            let mut g = SHARED.borrow_ref_mut(cs);
            let s = *g;
            if second {
                g.win = Window::NEW;
            }
            s
        });
        let st = sh.stats.unwrap_or_default();
        let now_us = Instant::now().as_micros();

        let unseen = sh.evt_count.wrapping_sub(seen_evts);
        if unseen as usize > MODE_RING {
            info!(
                "[MODE] t={=u64} {=u32} mode changes were overwritten before the logger saw them",
                now_ms,
                unseen - MODE_RING as u32
            );
            seen_evts = sh.evt_count.wrapping_sub(MODE_RING as u32);
        }
        while seen_evts != sh.evt_count {
            if let Some(e) = sh.evts.get(seen_evts as usize % MODE_RING) {
                old_last_frame_us = print_mode(e, now_ms, old_last_frame_us);
            }
            seen_evts = seen_evts.wrapping_add(1);
        }

        let r = &sh.run;
        if r.starts != seen_starts {
            if running {
                info!(
                    "[EDGE] t={=u64} sticks stopped last_us={=u64}",
                    now_ms,
                    r.start_us.saturating_sub(r.gap_us)
                );
            }
            if r.gap_us == FIRST {
                info!(
                    "[EDGE] t={=u64} sticks started at_us={=u64} first since power-up",
                    now_ms, r.start_us
                );
            } else {
                info!(
                    "[EDGE] t={=u64} sticks started at_us={=u64} gap_ms={=u64}",
                    now_ms,
                    r.start_us,
                    r.gap_us / 1000
                );
            }
            seen_starts = r.starts;
            running = true;
        }
        if running && now_us.saturating_sub(r.last_us) >= STOP_US {
            info!(
                "[EDGE] t={=u64} sticks stopped last_us={=u64} ran_ms={=u64}",
                now_ms,
                r.last_us,
                r.last_us.saturating_sub(r.start_us) / 1000
            );
            running = false;
        }
        if sh.first.count != seen_first {
            let f = &sh.first;
            info!(
                "[EDGE] t={=u64} first stick report in {=str} at_us={=u64} after_lock_us={=u64} after_discard1_us={}",
                now_ms,
                mode_name(f.mode),
                f.at_us,
                f.at_us.saturating_sub(f.mode_since_us),
                f.disc1_us.map(|t| f.at_us.saturating_sub(t))
            );
            seen_first = f.count;
        }

        if sh.restarts != seen_restarts {
            match sh.err {
                Some(RunError::Write(e)) => info!(
                    "[ERR] t={=u64} run returned Write({}); restart {=u32} in {=u64} ms; node stream_slowdown_ms={=u32}",
                    now_ms, e, sh.restarts, RESTART_MS, sh.err_slowdown_ms
                ),
                Some(RunError::WriteZero) => info!(
                    "[ERR] t={=u64} run returned WriteZero; restart {=u32} in {=u64} ms; node stream_slowdown_ms={=u32}",
                    now_ms, sh.restarts, RESTART_MS, sh.err_slowdown_ms
                ),
                Some(RunError::EndOfStream) => info!(
                    "[ERR] t={=u64} run returned EndOfStream; restart {=u32} in {=u64} ms; node stream_slowdown_ms={=u32}",
                    now_ms, sh.restarts, RESTART_MS, sh.err_slowdown_ms
                ),
                None => {}
            }
            seen_restarts = sh.restarts;
        }

        if !second {
            continue;
        }
        next_sum += SUM_MS;

        let w = &sh.win;
        match sh.stick {
            Some(s) if w.n > 0 => {
                let ch = &s.channels_us;
                match s.source {
                    Source::Crsf => info!(
                        "[STK] t={=u64} n={=u32} src=crsf iv_us={=u64}..{=u64} ivs={=u32} ch1-8={=[?]} ch15={=u16} ch16={=u16} seq={=u32}",
                        now_ms,
                        w.n,
                        w.iv_min_us,
                        w.iv_max_us,
                        w.intervals,
                        &ch[..8],
                        ch[14],
                        ch[15],
                        s.seq
                    ),
                    Source::Mavlink {
                        system_id,
                        component_id,
                    } => info!(
                        "[STK] t={=u64} n={=u32} src=mavlink {=u8}/{=u8} iv_us={=u64}..{=u64} ivs={=u32} ch1-8={=[?]} ch15={=u16} ch16={=u16} seq={=u32}",
                        now_ms,
                        w.n,
                        system_id,
                        component_id,
                        w.iv_min_us,
                        w.iv_max_us,
                        w.intervals,
                        &ch[..8],
                        ch[14],
                        ch[15],
                        s.seq
                    ),
                }
            }
            _ if st.mode != Mode::Searching => {
                info!(
                    "[STK] t={=u64} none mode={=str}",
                    now_ms,
                    mode_name(st.mode)
                )
            }
            _ => {}
        }

        let n_ls = d(st.crsf_links, sum.crsf_links);
        let n_rs = d(st.radio_status, sum.radio_status);
        if let (true, Some(l)) = (n_ls > 0, sh.crsf_link) {
            let ls = &l.stats;
            info!(
                "[LNK] t={=u64} crsf n={=u32} lq={=u8} rssi=-{=u8}/-{=u8} snr={=i8} rf={=u8} txpwr={=u8} seq={=u32}",
                now_ms,
                n_ls,
                ls.uplink_link_quality,
                ls.uplink_rssi_1,
                ls.uplink_rssi_2,
                ls.uplink_snr,
                ls.rf_mode,
                ls.uplink_tx_power,
                l.seq
            );
        }
        if let (true, Some(r)) = (n_rs > 0, sh.radio) {
            let rs = &r.status;
            info!(
                "[LNK] t={=u64} radio_status n={=u32} rssi={=u8} remrssi={=u8} txbuf={=u8} noise={=u8} remnoise={=u8} rxerrors={=u16} fixed={=u16} from={=u8}/{=u8} seq={=u32}",
                now_ms,
                n_rs,
                rs.rssi,
                rs.remrssi,
                rs.txbuf,
                rs.noise,
                rs.remnoise,
                rs.rxerrors,
                rs.fixed,
                r.system_id,
                r.component_id,
                r.seq
            );
        }
        if n_ls == 0 && n_rs == 0 {
            info!("[LNK] t={=u64} none", now_ms);
        }

        let tx = (
            TX_CRSF.load(Ordering::Relaxed),
            TX_MAV.load(Ordering::Relaxed),
        );
        info!(
            "[CNT] t={=u64} mode={=str} sticks=+{=u32}/{=u32} crsf_links=+{=u32}/{=u32} radio_status=+{=u32}/{=u32} lock_discarded=+{=u32}/{=u32} overrides_not_for_us=+{=u32}/{=u32} crc_errors=+{=u32}/{=u32} garbage_bytes=+{=u32}/{=u32} uart_framing=+{=u32}/{=u32} uart_parity=+{=u32}/{=u32} uart_break=+{=u32}/{=u32} uart_overrun=+{=u32}/{=u32} uart_other=+{=u32}/{=u32} tx_busy_polls=+{=u32}/{=u32} mode_changes=+{=u32}/{=u32} tx_bytes_crsf=+{=u32}/{=u32} tx_bytes_mav=+{=u32}/{=u32} restarts={=u32}",
            now_ms,
            mode_name(st.mode),
            d(st.sticks, sum.sticks),
            st.sticks,
            n_ls,
            st.crsf_links,
            n_rs,
            st.radio_status,
            d(st.lock_discarded, sum.lock_discarded),
            st.lock_discarded,
            d(st.overrides_not_for_us, sum.overrides_not_for_us),
            st.overrides_not_for_us,
            d(st.crc_errors, sum.crc_errors),
            st.crc_errors,
            d(st.garbage_bytes, sum.garbage_bytes),
            st.garbage_bytes,
            d(st.uart_framing, sum.uart_framing),
            st.uart_framing,
            d(st.uart_parity, sum.uart_parity),
            st.uart_parity,
            d(st.uart_break, sum.uart_break),
            st.uart_break,
            d(st.uart_overrun, sum.uart_overrun),
            st.uart_overrun,
            d(st.uart_other, sum.uart_other),
            st.uart_other,
            d(st.tx_busy_polls, sum.tx_busy_polls),
            st.tx_busy_polls,
            d(st.mode_changes, sum.mode_changes),
            st.mode_changes,
            d(tx.0, sum_tx.0),
            tx.0,
            d(tx.1, sum_tx.1),
            tx.1,
            sh.restarts
        );
        sum = st;
        sum_tx = tx;
    }
}
