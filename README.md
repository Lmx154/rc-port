# rc-port

The UART of an ExpressLRS receiver on the MARV V2 flight controller. When the transmitter changes model, the receiver changes by itself between CRSF at 420000 baud and MAVLink 2 at 460800 baud; this crate follows it, at the baud and in the protocol the receiver is using, and **reports** what arrives. It **decides nothing**: no loss flag, no loss threshold, no failsafe, no "safe values", no choice of stick source, no arming or disarmed rule, no write to the receiver's settings. Those are the firmware's (the owner's split: the driver reports, the firmware decides).

The crate is `no_std`, `forbid(unsafe_code)`, with no allocation, no `static` and no spinning. `RcPort` is a sans-IO core over a [`crsf-codec`](../crsf-codec) `Deframer` and a [`mavlink-device`](../mavlink-device) `Node`: bytes, UART errors and a time in; actions (writes, baud changes) and reports out. `RcPort::run` drives it on a UART with `embedded-io-async` 0.7 and `embedded-hal-async` 1.0. The feature `embassy-rp-0-10` implements the two small traits `run` needs for embassy-rp 0.10's `BufferedUart`.

**Status: version 0.1.0 (`publish = false`); the API may still change. Host tests only. NOTHING has run on a board or against a receiver.** Every number below that comes from the bench is a default waiting for the bench (R2a). On ExpressLRS 4.0.1 the receiver may send **no stick messages in MAVLink mode** after it has output CRSF since power-up (inferred from its source, NOT observed; see "The MAVLink stick messages may never come"); in that case this crate reports none, because none arrive.

## Works with

- Any MCU and HAL whose UART implements `embedded_io_async::{Read, Write}` (the reader cancel-safe, and returning `Pending` when it has nothing; see "Use"), plus this crate's `SetBaud` (change the baud rate of the running UART) and `TxBusy` (is the transmitter still sending), and an `embedded_hal_async::delay::DelayNs`. Any executor; no embassy crate in the core.
- embassy-rp 0.10, RP2350, the unsplit `embassy_rp::uart::BufferedUart`, with the feature `embassy-rp-0-10` (the chip feature, for example `embassy-rp/rp235xb`, comes from the application). It only builds tonight; it has never run.
- The sans-IO core on its own, with any I/O.

Dependencies: `crsf-codec` is a **path dependency** (`../crsf-codec`: it has no remote yet). `mavlink-device` is on GitHub (main at 92ea23d, the accept list; 09dfd17, `AcceptList::iter` and `contains`, which this crate uses, is on branch `wip-accept-list` and is to follow) but stays a **path dependency** (`../mavlink-device`), because `crsf-codec` has no remote: the crate cannot be built from git alone either way. `marv-dialect` is a git dependency pinned to the same commit as `mavlink-device`'s own (`61915e6`), so that both use one copy. Both crates are re-exported (`rc_port::crsf_codec`, `rc_port::mavlink_device`) so that the application uses the same versions.

## States and numbers

Three states: **Searching**, **Crsf**, **Mavlink**.

- Searching: listen at 460800 baud for `D_MAV`, then at the CRSF baud for `D_CRSF`, then 460800 again, and so on. At power-up and after leaving CRSF the search starts at 460800; after leaving MAVLink it starts at the CRSF baud (always "the other baud first"). Valid frames are counted per listening period; the count restarts at each baud change.
- Lock: `LOCK_FRAMES` valid frames in one listening period. Frames 1 and 2 are counted (`Stats::lock_discarded`) and **not reported**; the 3rd and every later one are.
- Leave: Crsf after `T_LEAVE_CRSF` without a valid CRSF frame, Mavlink after `T_LEAVE_MAV` without a valid MAVLink frame. Then Searching.
- A valid CRSF frame is any frame the deframer accepts (address 0xC8, length 2 to 62, CRC-8), of any type. A valid MAVLink frame is one the node hands over through `Node::receive_with`'s hook: RC_CHANNELS_OVERRIDE (70), RADIO_STATUS (109) or a message of the application's accept list (see "Writes and baud changes"), MAVLink 2, incompatibility flags 0, checksum right with the CRC_EXTRA, sender not the node's own system id. HEARTBEAT, parameter and command frames do not count (the node gives no per-frame event for them). RADIO_STATUS counts, so MAVLink mode holds with the radio link down, as the receiver keeps sending it.
- `Stats::mode_changes` counts locks whose protocol differs from the previous lock's: Crsf, Searching, Crsf counts 0; the first lock after power-up counts 0.

| Constant | Value | Source | Set from the bench? |
|---|---|---|---|
| `CRSF_BAUD` (`Config::crsf_baud`) | 420000 | ExpressLRS `options.cpp:198` default; can be changed when the receiver is flashed | yes, observation 1 |
| `MAVLINK_BAUD` | 460800 | ExpressLRS `rx_main.cpp:1287-1290`, fixed | no |
| `LOCK_FRAMES` | 3 | ArduPilot `requires_3_frames` (`AP_RCProtocol.h:146-175`) | no |
| `T_LEAVE_CRSF_US` | 1 000 000 | ArduPilot's 1000 ms dwell (`AP_RCProtocol.cpp:359-418`); a fade under 1 s never touches the baud | no |
| `T_LEAVE_MAV_US` (`Config::t_leave_mav_us`) | 200 000 | 20 RADIO_STATUS periods; must exceed any same-mode re-open gap | yes, observations 2 and 6 |
| `D_MAV_US` (`Config::d_mav_us`) | 50 000 | at least 5 RADIO_STATUS periods | yes, observations 2 and 6 |
| `D_CRSF_US` (`Config::d_crsf_us`) | 200 000 | at least 5 RC frame periods at the slowest packet rate | yes, observation 5 |
| `CRSF_TX_INTERVAL_US` (`Config::crsf_tx_interval_us`) | 100 000 | no pacing rule is established (`facts-rc-link.md` section 3: the receiver's telemetry buffer limits were not read); a conservative fixed minimum interval | yes, R3 |

The values come from `_plan/decision-rc-link-20261113.md` section 4; "observation N" is item N of section 6 of `_plan/facts-rc-link.md`. `Config` holds the bench values (`Config::new().with_d_mav_us(..)`, `#[non_exhaustive]`); a duration of 0 behaves as 1 us.

## Reports

One stick report type for both protocols, and two raw link reports, never merged. Each comes through `Hooks::report`, once per frame, from inside `RcPort::receive`.

```text
pub struct StickReport { pub channels_us: [u16; 16], pub source: Source, pub at_us: u64, pub seq: u32 }
pub enum Source { Crsf, Mavlink { system_id: u8, component_id: u8 } }
pub struct CrsfLinkReport { pub stats: crsf_codec::LinkStatistics, pub at_us: u64, pub seq: u32 }
pub struct RadioStatusReport { pub status: RadioStatus, pub system_id: u8, pub component_id: u8, pub at_us: u64, pub seq: u32 }
pub struct RadioStatus { pub rssi: u8, pub remrssi: u8, pub txbuf: u8, pub noise: u8, pub remnoise: u8, pub rxerrors: u16, pub fixed: u16 }
pub enum Report { Sticks(StickReport), CrsfLink(CrsfLinkReport), RadioStatus(RadioStatusReport) } // non_exhaustive
```

- CRSF sticks: `crsf_codec::raw_to_us` of each of the 16 raw values (172 gives 988, 992 gives 1500, 1811 gives 2012, not clamped). In the switch modes that do not send 16 channels, channels 15 and 16 carry link quality and RSSI; they are passed as received.
- MAVLink sticks: `chan1_raw` to `chan16_raw` of RC_CHANNELS_OVERRIDE **exactly as received**, 0 and 65535 ("ignore") included; channels 9 to 16 of a payload cut before them (MAVLink 2 drops trailing zeros) read 0; channels 17 and 18 are dropped. Only when `target_system` is the node's system id or 0; another target is not reported and is counted in `Stats::overrides_not_for_us` (the frame still counts as a valid MAVLink frame). The sender is reported; **which senders to accept is the firmware's** (ArduPilot accepts only its ground station's system id, `GCS_Common.cpp:4128`).
- `at_us` is the `now_us` of the `receive` call that delivered the frame: the time the bytes were **read** from the UART, not the time they arrived. After a stall of the caller, frames the UART buffered meanwhile are delivered with a fresh time, so they look new however long the stall was. Keep the UART's receive buffer small: the buffer, not this crate, bounds how many such frames one stall can deliver. At 250 Hz CRSF (a 26-byte RC frame every 4 ms) a buffer of B bytes holds about B / 26 RC frames, that is B / 26 x 4 ms of stream: 64 bytes about 2 frames (8 ms), 256 bytes about 9 (36 ms), 1024 bytes about 39 (156 ms). `seq` counts the reports of that kind, the first is 1, and wraps.
- Every stick report comes from exactly one frame that passed its CRC while the port was locked, as the 3rd or a later frame of the lock: never a held, repeated, interpolated or default value. Across a change there is only a gap.

Counters and timestamps (`RcPort::stats()`, or `Hooks::stats` while `run` is awaited), nothing else: `mode`, `mode_since_us`, `mode_changes`, `sticks` and `last_sticks_us`, `crsf_links` and `last_crsf_link_us`, `radio_status` and `last_radio_status_us`, `lock_discarded`, `overrides_not_for_us`, `crc_errors` (CRSF CRC plus MAVLink checksum, handled and listed messages), `garbage_bytes` (both decoders, plus bytes received during a baud change, plus the bytes the CRSF deframer held when a baud change ended), the UART errors one by one: `uart_framing`, `uart_parity`, `uart_break`, `uart_overrun`, `uart_other`, and `tx_busy_polls`: the checks of `TxBusy::tx_busy` that `run` made while waiting for the transmitter to go idle before a baud change and that found it still busy (one check per 20 us of waiting; saturating; always 0 with the sans-IO core alone, which does not wait). The wait has no timeout and the driver sets none: a transmitter that never goes idle holds `run` in it, and shows as `tx_busy_polls` rising, because `run` calls `Hooks::stats` after each busy check as well as once per wake. The node's own counters are in `port.node().stats()`.

## Use

The sans-IO loop (compiled by `cargo test`):

```rust
use rc_port::mavlink_device::{self, Outgoing, ParamError, ParamName, ParamStore, ParamValue, Telemetry};
use rc_port::{Action, Config, Hooks, Mode, RcPort, Report, MAVLINK_BAUD};

struct NoParams;
impl ParamStore for NoParams {
    fn count(&self) -> u16 { 0 }
    fn name(&self, _: u16) -> Option<ParamName> { None }
    fn get(&self, _: u16) -> Option<ParamValue> { None }
    fn find(&self, _: &ParamName) -> Option<u16> { None }
    fn set(&mut self, _: u16, _: ParamValue) -> Result<ParamValue, ParamError> { Err(ParamError::NotFound) }
}
struct NoTelemetry;
impl Telemetry for NoTelemetry {
    fn sample(&mut self, _: u32, _: u64, _: &mut Outgoing<'_>) -> bool { false }
}
struct Firmware { sticks: Option<[u16; 16]> }
impl Hooks for Firmware {
    fn report(&mut self, report: Report) {
        if let Report::Sticks(s) = report { self.sticks = Some(s.channels_us); }
    }
}

let mut slots = [];
let mut port = RcPort::new(Config::new(), mavlink_device::Config::new(1, 1), &mut slots)
    .expect("at most 16 accept list entries");
let (mut params, mut tel, mut fw) = (NoParams, NoTelemetry, Firmware { sticks: None });
// After power-up the port sets the UART to 460800 baud: flush, wait for the transmitter, set.
let mut bauds = Vec::new();
while let Some(action) = port.poll_action(0, &params, &mut tel, &mut fw) {
    match action {
        Action::Write(bytes) => { let n = bytes.len(); /* uart.write(bytes) took n */ port.written(n) }
        Action::Flush | Action::WaitTxIdle => port.done(0),
        Action::SetBaud(baud) => { bauds.push(baud); /* uart.set_baud(baud) */ port.done(0) }
    }
}
assert_eq!(bauds, [MAVLINK_BAUD]);
// Then, on each wake: bytes read go to `receive`, read errors to `uart_error`, then the actions
// until None, then sleep until `next_deadline_us()` or the next bytes.
port.receive(&[0x00, 0x55], 1_000, &mut params, &mut fw);
assert_eq!(port.stats().mode, Mode::Searching);
assert_eq!(port.next_deadline_us(), 50_000); // the end of the 460800 listening period
assert_eq!(fw.sticks, None);
```

The async loop (not compiled here: it needs a UART and an executor):

```rust,ignore
// embassy-rp 0.10, feature `embassy-rp-0-10`; UART0, GPIO14 TX, GPIO17 RX.
let mut uart = BufferedUart::new(p.UART0, p.PIN_14, p.PIN_17, Irqs, tx_buf, rx_buf, uart_config);
let mut port = RcPort::new(rc_port::Config::new(), mavlink_device::Config::new(1, 1), &mut slots)?;
let end = port.run(&mut uart, &mut Delay, || Instant::now().as_micros(), &mut params, &mut telemetry, &mut hooks).await;
// RunError::Write(_) | WriteZero | EndOfStream: call run again; the port kept its state.
```

`run` carries out every action (a write with `write`, acknowledged byte-exactly; a baud change as `flush`, then `tx_busy` checked every 20 us through the delay until false, then `set_baud`), then waits for the first of bytes and `next_deadline_us` (at most 100 ms). A read error is converted to `UartError` and counted, and `run` reads again at once (it is not `Node::run`, which returns on any read error). It returns only on a write or flush error, a write of 0 bytes or a read of 0 bytes. **The reader must return `Pending` when it has nothing**: `run` sleeps only by waiting on the read and the timer together, so a reader that is always ready (bytes every time, or an error returned at once every time, which `run` counts and then reads again) makes `run` spin without sleeping. The UART's error type must convert into `UartError` (`From` is implemented for embassy-rp's `uart::Error`).

## Writes and baud changes

- Writes only while locked, only in the locked protocol. CRSF: `Hooks::crsf_frame` is asked for one whole frame at most once per `crsf_tx_interval_us`, the first time at the first poll after the lock; the application writes it, for example the battery frame with `crsf_codec::encode_battery`. MAVLink: the node's `poll_transmit` frames (heartbeat, streams, replies). **Nothing is written while searching.** When a lock is left, the unsent rest of a frame is dropped.
- A baud change is always the action sequence `Flush` (the software buffer is empty), `WaitTxIdle` (the transmitter sent its last stop bit), `SetBaud(baud)`, each acknowledged with `RcPort::done`, with nothing written in between. After `SetBaud` both decoders' buffers are cleared: `Deframer::clear` for CRSF (the bytes it held, the start of a frame cut by the change, are counted in `garbage_bytes`; its other counters carry on) and `Node::on_link_down` for MAVLink (streams back to their defaults, replies and lists dropped, ground station no longer present, slowdown 0; the bytes its receive buffer held are dropped without being counted, as `mavlink-device` does). Bytes already in the UART's receive buffer from the old baud (embassy-rp does not clear it) are fed to the new decoder as garbage; the lock threshold is what keeps them harmless. Bytes received while a baud change is in progress are counted as garbage.
- The node is built by the port (`RcPort::new(config, mavlink_config, streams)`, which returns a `Result`). Its accept list is the application's (`mavlink_config.accept`, in its order) followed by RC_CHANNELS_OVERRIDE (70) and RADIO_STATUS (109) where the application does not list them; where it does, the entry keeps its place with the port's CRC_EXTRA for that id, whatever the application gave. The node holds at most 16 entries (`mavlink_device::MAX_ACCEPT`): a merged list longer than that is `Err(NewError::AcceptListFull)`, nothing is dropped. The application's other listed messages go to `Hooks::mavlink` (default: nothing), only while locked on MAVLink and from the 3rd valid frame of the lock on, once per frame, exactly like stick reports; before the lock they are counted in `lock_discarded` and not handed over. Each such message is a valid MAVLink frame for the lock and for leaving it, as RC_CHANNELS_OVERRIDE and RADIO_STATUS are. While the port listens at 460800 the node acts on HEARTBEAT, parameter and command frames as `mavlink-device` always does, but it writes only once locked, and **no parameter is written before the lock**: until the port is locked on MAVLink (and for the frames before the locking frame in the same `receive` call) the node gets a view of the application's `ParamStore` that reads it as it is and refuses `set` with `ParamError::Rejected`, so a PARAM_SET heard while listening (wrong-baud garbage that happens to form one included) changes nothing. The node queues its answer with the value unchanged; it is written only if the port locks on MAVLink before the next baud change drops it. Once locked, a PARAM_SET calls `ParamStore::set` as usual.
- Each call that takes `now_us` first applies the deadline due at that time (leaving a lock, ending a listening period), then its input. `now_us` must be non-decreasing and, for `run`, on the clock of the delay.

## Lock time, proved on the model

For one change from protocol A to protocol B, with t_last the end of the last A frame and G the gap to the start of the first B frame (host tests `mode_following_10_changes_each_way` and `lock_bound_holds_for_random_gaps_and_phases`):

```text
lock - t_last <= max(T_LEAVE_A, G) + D_A + 5 P + 2 d + 2 eps + 3 w
```

D_A is the listening time at A's baud, P the longest interval between the starts of consecutive valid B frames, d the longest B frame, eps the longest baud change (flush, transmitter drain, 15 bit periods), w the time from a frame's end to the `receive` call that hands it over. When the B frames already flow as the A lock is left (G <= T_LEAVE_A) it is sharper: lock - t_last <= T_LEAVE_A + eps + 3 P + d + 2 w. With the defaults and MAVLink as B (P = 10 ms, d = 1.09 ms): max(T_LEAVE_A, G) + 252.2 ms + 2 eps + 3 w, under the decision's max(T_LEAVE_old, G) + 280 ms while 2 eps + 3 w < 27.8 ms. On the model (w = 1 ms, gap 100 ms) the locks came 1020.5 ms after the last CRSF frame and 209.1 ms after the last MAVLink frame.

Re-lock after a total loss in CRSF mode, frames returning at t_r while searching (`crsf_link_lost_keeps_searching_writes_nothing_relocks_within_bound`): relock - t_r <= D_MAV + 5 P + 2 d + 2 eps + 2 w (the argument is at the top of `src/tests.rs`). Frames that return near the end of a CRSF listening period start a lock-on that the next baud change cuts; the lock then waits for one 460800 period and two baud changes. On the model at 250 Hz the worst re-lock seen was 64.7 ms (bound 73.3 ms), at 50 Hz 110.7 ms (bound 153.3 ms).

These are host results on the model (see "Tests"), not board results.

## What the application must do

- Provide the clock (`now_us`, microseconds), the executor and the task that awaits `run` (or the loop of the sans-IO calls), the `[StreamSlot]` slice, the `ParamStore` and `Telemetry` of `mavlink-device`, and the `Hooks` (reports, CRSF telemetry frames, counters).
- Make its own loss and failsafe decisions from the timestamps and counters: ages are `now - last_sticks_us` and so on. For reference only, not as policy here (decision section 3): Betaflight's `crsf.c` only reports frames, RXLOSS after 150 ms without a frame (`rx.c:137, 565-631`), failsafe stage 2 after 1.5 s (`failsafe.c:89`); ArduPilot's backend reports input with failsafe false (`AP_RCProtocol_CRSF.cpp:248`), the radio failsafe fires after `RC_FS_TIMEOUT`, default 1.0 s (ArduCopter `radio.cpp:115-132`), overrides count as RC input and expire after `RC_OVERRIDE_TIME`, default 3.0 s (`RC_Channel.cpp:563-578`). Neither uses link quality for failsafe.
- Choose which stick source is in charge, which MAVLink senders to accept, and whether a change of protocol is allowed while armed (the decision flags "disarmed only" as beyond both Betaflight and ArduPilot; the driver always follows, and while armed a change simply looks like a gap in stick reports).
- While `run` is awaited the port cannot be touched: everything goes through `Hooks`, `ParamStore` and `Telemetry`, which are called from the port's task and must not block.

## The MAVLink stick messages may never come

ExpressLRS 4.0.1, **inferred from its source, NOT observed** (decision, "The finding that gates the MAVLink half"): in MAVLink mode the receiver sends RC_CHANNELS_OVERRIDE from a `timeout()` that, once CRSF output has run since power-up, is probably never called again; this includes the first change Normal to MAVLink after booting in Normal (ExpressLRS issue 3860 reports the same after MAVLink, Normal, MAVLink on 4.1.0). RADIO_STATUS and ground-station traffic are not affected. If so, this crate locks on MAVLink (RADIO_STATUS keeps it there), reports RADIO_STATUS, and reports **no sticks**, because none arrive. The first board step (R2a) observes it.

## Not covered and not verified

- **Anything on a board or with a receiver.** No baud change on a real UART, no real ExpressLRS stream, no re-open gap, no real `busy()` timing, no executor, no cancel on the real HAL. The embassy-rp back-end only compiles.
- The bench numbers (CRSF baud, re-open gap, RADIO_STATUS period, RC frame period, telemetry pacing) are defaults; the bound above uses the model's frame periods (CRSF at 250 Hz and 50 Hz, RADIO_STATUS every 10 ms, RC_CHANNELS_OVERRIDE every 20 ms).
- While locked, the CRC is the only defence: wrong-baud garbage that happens to form a valid frame of the locked protocol (CRSF: about 2^-24 per byte position, an RC frame about 2^-32) would count and could be reported. Not seen in the tests.
- A dwell is not extended when a lock-on is in progress (the design says "alternate"); that is why the re-lock bound above has 5 P + 2 d + 2 eps + 2 w after D_MAV.
- No stick report or lock from a CRSF baud equal to 460800 has been tested (`Config::crsf_baud` can be set to it; the search then alternates between the two protocols at one rate).
- The simulated UART cannot produce noise, baud-rate error, the real receiver's oversampling and majority voting, FIFO overruns, or idle gaps inside a frame. How much wrong-baud garbage a real line yields, and what it looks like, is a bench question; the I16 results hold for the model.
- No `defmt` feature yet.
- `TxBusy` waiting has no timeout: with no hardware flow control the transmitter drains in at most its FIFO's time. A transmitter that stays busy shows as `Stats::tx_busy_polls` rising in `Hooks::stats`; what to do about it is the application's.

## Tests

`cargo test` runs 27 host tests (and the example above). The model (`src/sim.rs`, test-only): a wire carrying 8N1 bytes sent at one baud rate and a UART receiver at another that resamples it at its own bit centres, giving the bytes, framing errors and breaks a real UART would; a transmitter with a 32-byte FIFO that drains at the baud rate; a made-up clock; a driver that wakes every `step_us` or at the port's deadline. Its assumptions are listed at the top of the file (exact edge detection where a PL011 has 1/16 bit of jitter, no clock error, no noise, no overrun, a baud change abandons the character in progress and keeps the receive buffer, the receiver off for 15 bit periods).

- `uart_model_same_baud_is_exact_wrong_baud_is_garbage`: the model's control (and what makes I16 mean something): at the sender's baud the bytes are exact, at the other baud garbage with framing errors and no valid frame.
- `i13_every_stick_report_is_one_valid_frame_of_the_input` (proptest): random valid streams of both protocols with mode changes, unique stick values per frame, 0, 65535 and cut payloads, overrides for other systems, garbage and corrupted frames; every report equals exactly one valid frame of the input, in order, never twice, after the frame ended.
- `i14_crsf_frames_stop_reports_stop_and_counters_freeze`, `i14_mavlink_overrides_stop_radio_status_continues_mode_stays`.
- `i16_no_report_from_valid_frames_planted_in_wrong_baud_garbage` (proptest): CRSF sent at 420000 received at 460800 and MAVLink sent at 460800 received at 420000, with 1 or 2 valid frames of the listened protocol and up to 4 of the other planted at random byte positions: no report, no lock. `i16_negative_control_fails_with_lock_frames_1`: the same check run with LOCK_FRAMES = 1 (a test-only hook) fails.
- `i17_nothing_written_while_searching_tx_idle_before_every_baud_change` (proptest): on the recorded trace, every write is in a locked mode at its protocol's baud and parses as whole frames of it (CRSF frames at least `crsf_tx_interval_us` apart); every baud change comes after a Flush and a WaitTxIdle with no write between, with the fake transmitter flushed and idle.
- `i18_same_mode_gap_shorter_than_t_leave_changes_nothing` (proptest): gaps up to T_LEAVE less 30 ms, silent or with garbage at either baud.
- `mode_following_10_changes_each_way`: CRSF, 100 ms silence, MAVLink (RADIO_STATUS every 10 ms, overrides every 20 ms), and back, 10 changes each way in one run: mode-change counter 20, every lock within both bounds, every report the expected value of its protocol, every stick frame after the lock reported. `lock_bound_holds_for_random_gaps_and_phases` (proptest): gaps of 0 to 1.5 s, both directions, CRSF at 250 or 50 Hz, random wake periods; also under max(T_LEAVE_old, gap) + 280 ms.
- `crsf_link_lost_keeps_searching_writes_nothing_relocks_within_bound`: leave at exactly 1000 ms, alternating 460800 for 50 ms and CRSF for 200 ms, nothing written, re-lock within the bound for return times swept 2 ms apart over more than one search cycle, at 250 and 50 Hz.
- `mavlink_link_lost_stays_mavlink_sticks_frozen_radio_status_advances`: 5 s of RADIO_STATUS only.
- `chunking_does_not_change_reports_counters_or_actions` (proptest, CONTRACT B4): whole runs, random pieces and single bytes give the same reports, counters and trace.
- `override_target_filter_and_exact_values`, `uart_error_burst_counted_no_report_no_mode_change_before_t_leave`.
- `late_receive_applies_the_deadline_before_the_bytes`: a valid frame in a `receive` whose `now_us` is at the leave deadline is not reported and does not keep the lock (the bytes are garbage); a third valid frame at the end of a listening period does not lock; both protocols.
- `unsent_rest_of_a_frame_dropped_when_the_lock_is_left`: a frame written in part when the caller stalls past the leave deadline; after the baud change and the lock on the other protocol nothing of its rest is written, and every write is whole frames of the new protocol, the first one included; both directions.
- `listed_mavlink_message_handed_over_only_while_locked`: an application message (an id outside the common dialect) handed to `Hooks::mavlink` once per frame from the 3rd frame of the lock on, never before, never in CRSF mode (sent at the CRSF baud), an unlisted id never; it alone makes and keeps the lock. `listed_mavlink_message_never_handed_over_from_wrong_baud_garbage` (proptest): the I16 construction with that message planted. `merged_accept_list_over_16_is_an_error`, `application_listing_rc_channels_override_changes_nothing` (listed with a wrong CRC_EXTRA: the port's wins).
- `param_set_applied_only_once_locked_on_mavlink`: a valid PARAM_SET between the two lock-on frames leaves the store unchanged; after the lock it is applied.
- Async front end, against a fake UART on the same model and a fake delay, polled by hand: `run_follows_crsf_to_mavlink_and_back`, `run_flushes_then_waits_tx_idle_then_sets_baud` (with a slowed fake transmitter so that the wait is exercised), `run_dropped_at_any_await_loses_nothing` (CONTRACT A2: the future dropped after every 1 to 5 polls, over 10 000 times; the same reports and counters as an undisturbed run), `run_future_is_send` (CONTRACT A5, compile-only), `run_counts_idle_wait_polls_that_found_the_transmitter_busy` (the stats hook sees `tx_busy_polls` rise by one per busy check, and only then).

Mutations of the core and of `run` were checked to fail the tests meant to catch them (lock-on frames reported, no target filter, a held value, the lock count per call, the search order, swapped dwells, a default report after silence, an early CRSF leave, writes while searching, no idle wait, no flush, `run` returning on a read error; and, for the tests added since, no deadline applied at the top of `receive`, the unsent rest of a frame kept at a mode change, and parameter writes allowed before the lock); they are not in the repository.

Lints and builds: `cargo clippy --all-targets -- -D warnings` on the host; the back-end only for the target: `cargo clippy --lib --target thumbv8m.main-none-eabihf --features embassy-rp-0-10,embassy-rp/rp235xb -- -D warnings` and the same with `cargo build`. (`--all-features` on the host cannot build embassy-rp, which needs a chip feature and the ARM target.)

## Cargo features

All off by default: `embassy-rp-0-10` (`SetBaud` and `TxBusy` for `embassy_rp::uart::BufferedUart`, `From<embassy_rp::uart::Error>` for `UartError`; the chip feature comes from the application).

## Layout and licence

```text
src/lib.rs        crate documentation, lints, re-exports
src/config.rs     the constants and Config
src/port.rs       the sans-IO core: RcPort, Action, Hooks, Stats, Mode, UartError, NewError
src/report.rs     the report types
src/run.rs        RcPort::run, SetBaud, TxBusy, RunError
src/embassy.rs    feature embassy-rp-0-10
src/sim.rs        test-only: the wire, the resampling UART, the transmitter, the driver
src/tests.rs      host tests of the core
src/run_tests.rs  host tests of run
```

MIT, see `LICENSE`.
