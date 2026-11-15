# rc-port bench programs (`hil/`)

Status: `rc_capture` has RUN on the board twice (2026-11-13, logs `rc_capture-20261113-101707.log` and `-102410.log`, kept locally under `logs/`): both listeners worked on the shared pin, with no overrun reported. What it showed is recorded in the project's notes, not here. A README claims only what ran; when it runs, name the log here.

## `rc_capture`: listen-only capture of the receiver's line (decision R2a)

It listens to the ExpressLRS receiver (RadioMaster RP4TD-M, ELRS 4.0.1) on UART0 RX and logs what arrives, for facts-file section 6, items 1, 2, 3 and 5 (`_plan/facts-rc-link.md`). It decides nothing on the board: there is no PASS/FAIL or RESULT line, and **the log is judged afterwards**, by reading it. It does not use `rc-port`: it decodes with `crsf-codec` and `mavlink-device` only to count.

**Listen only.** Nothing is transmitted and no pin is driven. GPIO14 (UART0 TX, pad T0) is never configured; the MAVLink node is only fed, never asked for a frame to send. PIO0, PIO2 and GPIO36-39 (the four ESCs, DShot) are never touched, with or without the battery.

Two listeners hear GPIO17 at once, so both protocols are seen without knowing which one is on:

| | A | B |
|---|---|---|
| peripheral | UART0 RX, `embassy_rp::uart::BufferedUartRx` (RX only) | PIO1, state machine 0, `pio_uart::UartRx` |
| pin | GPIO17 (pad R0); the UART owns the pad's function and pad control | GPIO17, read as a PIO input |
| baud | 420000 8N1 (build-time `RC_CAPTURE_CRSF_BAUD`) | 460800 8N1 |
| decoder | `crsf_codec::Deframer` | `mavlink_device::Node`, accept list RC_CHANNELS_OVERRIDE (70) and RADIO_STATUS (109), never transmits |
| interrupt | `UART0_IRQ` (HAL handler); reader task in thread mode | `PIO1_IRQ_0`; reader task on an interrupt executor, `SWI_IRQ_0` at NVIC priority 0xE0 |
| DMA | none | none |
| buffer | 8192-byte ring (195 ms of line at 420000), reads of 64 bytes | the 8-word joined FIFO (174 us at 460800), reads of up to 32 bytes |
| losses counted | the HAL's error kinds apart: framing, parity, break, overrun (an overrun is also how a full ring shows) | `UartRx` stats: framing errors, overrun events, runt starts |

Each listener hears the other protocol as garbage; its bytes and errors are counted too.

The pin is shared as in dshot's `dshot_smoke` (dshot README rule B9): PIO1 is given GPIO17 first (`make_pio_pin` on a second handle, then `UartRx::new`, which turns the pad pull-up on), and THEN the UART takes the pad (function UART0 RX). The UART's pad write in embassy-rp 0.10 writes the whole word from rp-pac's default 0 with only input enable set (`uart/mod.rs` `init`, the `rx` pad write): by reading the code, **both pulls end up off and the pad floats**, the pull-up `UartRx` set and the reset pull-down alike, exactly as rc-port's own UART will leave it. This is not what the decision's section 7 assumed ("whether the RX pad's reset pull-down stays on"); the banner prints the pad's state after setup (`pull-down`, `pull-up`, `input enabled`), which settles it on the board: in the captures it read function 2 (UART0), no pull-up, no pull-down, input enabled, level 1.

### Wiring

- Receiver TX -> R0 (GPIO17).
- Receiver RX -> T0 (GPIO14): may be connected or left open; the program never drives GPIO14.
- Ground to ground.
- The receiver's supply: its own 5 V supply. NOT the board's 3.3 V pins unless the owner confirms the receiver accepts 3.3 V.
- The flight battery need not be connected (the ESCs then stay unpowered). The program never touches the DShot pins either way.

### Run

```sh
tools/run.sh rc_capture 600
```

from the crate root (`rc-port/`). The probe's timeout (600 s) ends the run; there is no other end. The terminal shows only the `[INFO]` banner (run.sh prints lines tagged `[PASS|FAIL|SKIP|INFO|STAT]`); the whole log is in `hil/logs/rc_capture-<date>.log`. Read it with `grep -E '\[(EVT|SUM|VAL|WIN)\]'`. To try another CRSF rate: `RC_CAPTURE_CRSF_BAUD=416666 tools/run.sh rc_capture 600`.

### Log lines

Every line starts with `t=` the board's time in ms.

- `[EVT]`, for each of four streams, `crsf_rc` (CRSF 0x16), `crsf_ls` (CRSF 0x14), `mav_ovr` (RC_CHANNELS_OVERRIDE), `mav_rst` (RADIO_STATUS):
  `[EVT] t=15020 mav_rst started at=15012 gap_ms=1840` (the first valid frame after 100 ms or more without one; `first since power-up` for the first),
  `[EVT] t=13280 crsf_rc stopped last=13172 ran_ms=9480` (none for 100 ms; `last` is the time of the last frame),
  and after `crsf_ls` stops, `[EVT] t=13290 crsf_ls last frame: lq=0 rssi=-110/-110 snr=0 rf=7`.
- `[SUM]` once a second, counts in that second:
  `[SUM] t=12000 crsf: bytes=… rc=… ls=… other=… crc=… badlen=… garb=… uart(fr/pa/br/ov)=…/…/…/… | mav: bytes=… ovr=… rst=… hb=… unk=… crc=… garb=… v1=… pio(fr/ov/runt)=…/…/…`
  (`v1`: MAVLink 1 frames, which the node drops; a second `[SUM] … rare:` line appears only if a UART error of another kind or a frame with system id 0 came).
- `[VAL]` once a second, the latest values:
  `[VAL] t=12000 line=1 rc_us=[ch1, ch2, ch3, ch4, ch5, ch15, ch16] lq=… rssi=-…/-… snr=… rf=… | ovr_us=[ch1, ch2, ch3, ch4, ch5, ch15, ch16] ovr_src=sys/comp ovr_tgt=sys/comp txbuf=… rs_rssi=… rs_noise=… rs_rem=remrssi/remnoise rs_src=sys/comp B=true`
  (`rc_us` through `crsf_codec::raw_to_us`; `ovr_us` as received; `line` the GPIO17 level at that instant; `B=false` if listener B could not be built).
- `[WIN]` every 10 s, totals of the last 10 s:
  `[WIN] t=20000 last 10 s: crsf_rc=… crsf_ls=… mav_ovr=… mav_rst=… | mav_hb=… crsf_bytes=… mav_bytes=… uart_err=… uart_brk=… uart_ov=… pio_fr=… pio_ov=…`

### Sequences, by hand

Each sequence three times. Pause about 10 s between steps, so that each step has whole `[WIN]` windows of its own. Note the wall-clock order of the steps; the log's `t=` gives the rest.

1. Normal (CRSF) model: link up; move every stick to both ends; centre; radio off for 5 s; radio on.
2. The same in the MAVLink model.
3. Receiver powered up in the Normal model, link up; then the FIRST switch to the MAVLink model.
4. MAVLink -> Normal.
5. Normal -> MAVLink again (with the receiver powered all along since 3).
6. A receiver power cycle in each mode (Normal; MAVLink).
7. Negative control: receiver unpowered for 10 s (at least one whole `[WIN]`).

### What to look for

- **Observation 1, the CRSF baud and the last LQ-0 frame.** In the Normal model: `[SUM]` `rc` near the packet rate with `crc=0`, `badlen=0`, `uart(fr/pa/br/ov)` all 0 means the baud is right. Few `rc` with many `garb`, `crc` and framing errors means another baud: rebuild with `RC_CAPTURE_CRSF_BAUD`. Positive control (decision R2a): `[WIN]` `crsf_rc` divided by 10 is 0.9 to 1.0 times the packet rate shown in the Lua script. At radio off: the `crsf_ls last frame` line after `crsf_ls stopped` (lq=0 or not), or, if link statistics keep coming, `[VAL]` `lq=0`.
- **Observation 2, a model change.** The `[EVT]` lines in order: the old protocol's streams `stopped last=X`, the new one's `started at=Y gap_ms=…`; the re-open gap G is Y - X. In the `[SUM]` seconds between: garbage at the old baud (A's `garb`, `crc`, framing errors while MAVLink is on; B's `pio fr` while CRSF is on), breaks (`uart br`) and `line=0` while the receiver re-opens, and the first good frame (`started`).
- **Observation 3, issue 3860 and the wider inference.** Per 10 s window in the MAVLink model: `[WIN]` `mav_ovr=0` with `mav_rst` above 500 is **reproduced**; "not reproduced" needs 3 of 3 with `mav_ovr` resuming, in both switch sequences (3 and 5). `mav_ovr started` / `mav_rst started` lines show which came. Sequence 6 (power cycle in MAVLink) shows whether a fresh boot in MAVLink gives sticks.
- **Observation 5, range and layout.** `[VAL]` `rc_us` and `ovr_us` with the sticks at each end and centred (ranges, which channel moves for which stick); channels 15 and 16 in each mode (LQ and RSSI in CRSF mode?). Channels 6 to 14 are not printed: the resolution of channels 5 to 16 in Hybrid and 16ch/2 is not answered by this program beyond channel 5.
- **Negative control.** Receiver unpowered: `[WIN]` all four streams 0. `uart_brk`, framing errors and `line` show what the released line does with the pad as the HAL leaves it (by the code: no pull, floating; decision section 7 expected a pull-down and a continuous break).
- **Losses.** `uart ov` and `pio ov` should stay 0; any other value means bytes were lost at that moment and the counts around it are short.

### Not verified (needs the board)

- That the two listeners really share GPIO17 as designed: in the two runs both listeners counted frames and the banner read function 2 with no pull-up and no pull-down.
- That nothing is lost: in the two runs UART overruns were 0 and PIO overruns 0; the buffer sizes and the interrupt executor are otherwise argued from the rates, not measured.
- The pio-uart divider at 460800 and 150 MHz is 20 + 88/256 (about +64 ppm, inside its tolerance by arithmetic); the PL011 at 420000 from 150 MHz is about +400 ppm. Neither is measured.
- The HAL reports one error per read when its ring is empty, the most severe of those seen (overrun, then break, parity, framing), and drops the byte that carried it: the error counts are counts of indications, not of bytes.

## `rc_follow`: the port itself, following the receiver (decision R2, R3, R4, R5)

Status: has run once (2026-11-13, one 600 s run, log `rc_follow-20261113-111239.log`, kept locally under `logs/`). What it showed is in the main README, section "Measured on the MARV V2 Rev 1 board", with what has not yet been observed. `cargo build --release --bin rc_follow` in this folder builds it.

It runs `rc_port::RcPort::run` (feature `embassy-rp-0-10`) on UART0 and logs what the port reports: modes, stick reports, link reports and every counter. It decides nothing on the board: no PASS/FAIL or RESULT line, no failsafe, no loss decision, no arming; **the log is judged afterwards**. It runs until the probe's timeout ends it.

**What it transmits, and nothing else**: in CRSF mode the CRSF battery frame of `Hooks::crsf_frame` (at most one per `crsf_tx_interval_us`, 100 ms), a FIXED value chosen at build time (`RC_FOLLOW_BATT_DV` in decivolts, default 123 = 12.3 V; `RC_FOLLOW_BATT_PCT`, default 50; current 0, capacity 0; the ADC is NOT read, the battery is unplugged); in MAVLink mode the MAVLink node's own messages (system 1, component 1, defaults otherwise: HEARTBEAT and replies; no streams; two test parameters `RCF_TEST_I` = 42 and `RCF_TEST_F` = 1.5). Nothing while searching (the port's rule). It never binds, flashes or writes the receiver's settings. PIO0, PIO1, PIO2, GPIO36-39 (the four ESCs, DShot), the ADC and the DMA channels are never touched; the only pin driven is GPIO14 (UART0 TX).

| | |
|---|---|
| peripheral | UART0, unsplit `embassy_rp::uart::BufferedUart` (interrupt-driven, no DMA), wrapped in a counting adapter that forwards read, write, flush, `SetBaud` and `TxBusy` and counts the bytes each write took, by baud rate |
| pins | TX GPIO14 (pad T0, function UART0 TX, set by the HAL), RX GPIO17 (pad R0, function UART0 RX) |
| RX pad | after `BufferedUart::new` (which writes GPIO17's whole pad word with only input enable set, so no pull): one read-modify-write of `PADS_BANK0` GPIO17 setting PUE (pull-up enable) to 1, every other bit as read (`pac::PADS_BANK0.gpio(17).modify(|w| w.set_pue(true))`); `set_baudrate` does not touch the pad. The banner prints the pad as it then is |
| baud | built at 460800 (the port's first actions set 460800 anyway); then the port's `SetBaud` |
| rings | RX 256 bytes: `at_us` is the READ time, so the ring bounds the stale frames one stall can deliver with a fresh time (rc-port README, "Reports"): about 9 RC frames at 250 Hz, 6.1 ms of line at 420000 (5.6 ms at 460800). TX 256 bytes |
| interrupts | `UART0_IRQ` (HAL handler, priority 0); the port's task on an interrupt executor, `SWI_IRQ_0` at NVIC priority 0xE0; the logger in thread mode |
| clock, delay | `embassy_time::Instant` in us; `embassy_time::Delay` |
| node | `mavlink_device::Config::new(1, 1)`, application accept list empty (the port adds 70 and 109), `Hooks::mavlink` default |

The hooks only copy into one `critical_section::Mutex` snapshot (they never block and never format); the stats hook also notes the board's time when it sees a mode change or a `lock_discarded` increment. The logger looks at the snapshot every 10 ms and does all the formatting.

### Wiring

- Receiver TX -> R0 (GPIO17).
- Receiver RX -> T0 (GPIO14): **now driven**, connect it (CRSF telemetry and MAVLink go out on it).
- Ground to ground; the receiver powered from the board, as in the `rc_capture` runs.
- The flight battery need not be connected. The program never touches the DShot pins either way.

### Run

```sh
tools/run.sh rc_follow 600
```

from the crate root (`rc-port/`). The probe's timeout ends the run (exit status 124 is that, not a failure). The terminal shows only the `[INFO]` banner; the whole log is in `hil/logs/rc_follow-<date>.log`; read it with `grep -E '\[(MODE|EDGE|STK|LNK|CNT|ERR)\]'`. Another battery value: `RC_FOLLOW_BATT_DV=111 RC_FOLLOW_BATT_PCT=75 tools/run.sh rc_follow 600` (a value out of range fails the build).

### Log lines

Every line has `t=` the board's time in ms when the logger printed it; `_us` fields are on the port's clock (`Instant`, us). `Some(x)`/`None` is how an optional value prints.

- `[MODE]` on every change of `Stats::mode` (lost changes, if more than 8 came between two looks of the logger, are counted on a line of their own):
  `[MODE] t=… -> Crsf since_us=… seen_us=… changes=… prev=Searching prev_lasted_us=… discarded=2 discard1_to_lock_us=Some(…) discard2_to_lock_us=Some(…) old_last_frame_to_lock_us=Some(…)`
  `[MODE] t=… -> Searching since_us=… seen_us=… changes=… prev=Crsf prev_lasted_us=… last_valid_frame_us=Some(…) last_stick_us=Some(…) seen_after_last_stick_us=Some(…)`
  `since_us` is `Stats::mode_since_us` (for a lock the time of the locking frame; for Searching the deadline at which the lock was left, its last valid frame plus the leave time, so `last_valid_frame_us` is `since_us` minus 1000 ms or 200 ms). `seen_us` is when the stats hook saw the change. `prev_lasted_us` is how long the previous mode lasted: on a lock line, the time since the previous lock ended. `discarded` counts `lock_discarded` during the search, `discardN_to_lock_us` the time from the last two discards (frames 1 and 2 of the lock, as the hook saw them) to the lock (0 when they came in the same read), `old_last_frame_to_lock_us` the time from the previous lock's last valid frame to this lock.
- `[EDGE]` `t=… sticks started at_us=… gap_ms=…` (first stick report after 100 ms or more without one; `first since power-up` for the first), `t=… sticks stopped last_us=… ran_ms=…` (none for 100 ms; `last_us` is the last report), and `t=… first stick report in Crsf at_us=… after_lock_us=… after_discard1_us=Some(…)` (the first stick report after each lock).
- `[STK]` once a second while stick reports arrive: `t=… n=… src=crsf iv_us=MIN..MAX ivs=… ch1-8=[…] ch15=… ch16=… seq=…` (or `src=mavlink 255/68`): reports in that second, the shortest and longest interval between consecutive reports of one run from `at_us` (`ivs` intervals; 0 is two frames in one read), the latest report's channels in us. While locked without stick reports: `[STK] t=… none mode=Mavlink`.
- `[LNK]` once a second: `t=… crsf n=… lq=… rssi=-…/-… snr=… rf=… txpwr=… seq=…`, or `t=… radio_status n=… rssi=… remrssi=… txbuf=… noise=… remnoise=… rxerrors=… fixed=… from=255/68 seq=…`, whichever came in that second (both if both), else `t=… none`.
- `[CNT]` once a second, `+delta/total` of every `Stats` counter: `sticks`, `crsf_links`, `radio_status`, `lock_discarded`, `overrides_not_for_us`, `crc_errors`, `garbage_bytes`, `uart_framing`, `uart_parity`, `uart_break`, `uart_overrun`, `uart_other`, `tx_busy_polls`, `mode_changes`; then `tx_bytes_crsf` and `tx_bytes_mav` (bytes the UART took at the CRSF baud and at 460800: the port writes only while locked, so these are the bytes written in each mode) and `restarts`.
- `[ERR]` if `run` returns: `t=… run returned Write(…)|WriteZero|EndOfStream; restart N in 100 ms; node stream_slowdown_ms=…`. The node's `stream_slowdown_ms` cannot be read while `run` is awaited (no hook carries it), so it is printed only here.

### What the operator does, and what to read

Note the wall-clock order of the steps; the log's `t=` gives the rest.

- **R2, CRSF in.** Receiver in the Normal model, link up. Move each stick to both ends, slowly, then centre; then leave them. `[STK]` `ch1-8` follow the sticks; each stick end at least 400 us from centre in the right direction; at rest the spread of the values over several `[STK]` lines is the rest noise; `iv_us` near the frame period (about 4 ms at the owner's rate); `[LNK] crsf` about 10 a second. Radio off for 5 s or more: `[EDGE] sticks stopped` at once, `[STK] none`, `sticks` frozen in `[CNT]`, then `[MODE] -> Searching` with `seen_after_last_stick_us` about 1 000 000 (1000 ms, the decision's +-20 ms). Radio on: `[MODE] -> Crsf` with `discarded=2`, `[EDGE] first stick report` with `after_discard1_us` within 3 frame periods (about 12 ms at 250 Hz). `crc_errors` and the `uart_*` counters 0 while locked.
- **R3, CRSF telemetry out.** In the Normal model, on the Pocket: telemetry screen, "discover sensors": the battery sensor shows the fixed value of the banner (12.3 V, 50 % by default). `[CNT]` `tx_bytes_crsf` about +120 a second in CRSF mode (one 12-byte frame per 100 ms), `tx_bytes_mav` +0. The decision's R3 check against the board's `adc-battery` voltage is NOT made here (no ADC, no battery).
- **R4, MAVLink in.** Power the receiver up with the transmitter in the MAVLink model (a receiver power cycle: the decision's "gate"). `[MODE] -> Mavlink`, `[STK] src=mavlink 255/68` following the sticks as in R2, `[LNK] radio_status` about 91 a second (R2a), `tx_bytes_mav` the node's HEARTBEAT and replies. Radio off: `[EDGE] sticks stopped`, `[STK] none mode=Mavlink`, `[LNK] radio_status` continuing with `rssi=0`, and NO `[MODE]` line (the mode stays Mavlink). Then, in the same power-up, after the receiver has output CRSF (Normal and back to MAVLink): `[MODE] -> Mavlink`, `[LNK] radio_status`, `[STK] none mode=Mavlink` and no `first stick report` line: the expected result on ELRS 4.0.1. `txbuf` is now meaningful (the board drives T0).
- **R5, following the change.** Receiver booted in the MAVLink model; ten changes each way with no reset (MAVLink -> Normal -> MAVLink …). `[CNT]` `mode_changes` +1 for each, 20 at the end; each lock's `[MODE]` line `old_last_frame_to_lock_us` against the bound max(T_LEAVE_old, G) + 280 ms (G about 1.2 s to CRSF, 3.4 s to MAVLink, R2a); `[EDGE] first stick report` after each lock into CRSF. Sticks held still: every `[STK]` within the rest noise of the value before, in both modes (after the first change back to MAVLink there are no MAVLink stick reports, see R4). No stick value the radio did not send: no `[STK]` line while searching and no report across a change (only a gap: `sticks stopped` before the change, `sticks started` after the lock).
- **Losses.** `uart_overrun` should stay 0; `[ERR]` should never appear.

### Not verified (needs the board)

- Seen in the one run: the port followed 16 changes each way, so `RcPort::run` changed the baud on the real PL011 and `tx_busy` went false (`tx_busy_polls` 9); the banner read the pull-up true; 0 `uart_overrun` with the 256-byte RX ring in that run; the handset showed the battery frame (12.3 V, 0.0 A, 0 mAh, 50 %).
- Not observed by `rc_follow`: the radio off in MAVLink mode through the port, an overload of the downlink and the stream slowdown, stick reports in MAVLink mode through the port, a message of an application's own dialect through `Hooks::mavlink`, any other receiver, radio or ExpressLRS version, flight. `rc_link` (below) later observed the radio off in MAVLink mode, the stream slowdown, an overload as seen from the board, and a message of the project's own dialect through `Hooks::mavlink`. A long stall of the logger (defmt-rtt holds a critical section while it encodes each line) would show as `uart_overrun`; none came in 600 s.

## `rc_link`: radio off in MAVLink mode, own-dialect message each way, downlink overload

Status: **has run** on the board on 2026-11-13 (logs `rc_link-20261113-115428.log`, `rc_link-20261113-115915.log` and `rc_link-20261113-121455.log`, kept locally under `hil/logs/`). What it showed is in the crate's `README.md`, section "Measured on the MARV V2 Rev 1 board". `cargo build --release --bin rc_link` in this folder builds it.

A copy of `rc_follow` (same UART0 set-up, GPIO14 TX driven, GPIO17 RX with its pull-up turned on after the UART is built; the same interrupt executor and thread-mode logger; the same snapshot; the same banner; the same `[MODE]`, `[EDGE]`, `[STK]`, `[LNK]`, `[CNT]` and `[ERR]` lines; the same fixed CRSF battery frame with `RC_FOLLOW_BATT_DV` and `RC_FOLLOW_BATT_PCT`), with three additions. `rc_follow` is unchanged. It decides nothing: **the log is judged afterwards**. It never touches PIO0, PIO1, PIO2, GPIO36-39 (the ESCs), the ADC or DMA; the only pin driven is GPIO14 (UART0 TX).

- **The test message** `MARV_TEST` (id 42420, CRC_EXTRA 100; `marv_dialect::marv::MARV_TEST_DATA` of marv-dialect `e49c15f`, a second, renamed dependency `marv-dialect-new`; `mavlink-device`, `rc-port` and this crate's first copy are on `af6baa1` since 2026-11-14, which carries the same message; both copies use the one `mavlink-core` 0.21.0). Received: it is on the application's accept list, so the port hands it to `Hooks::mavlink` (only while locked on MAVLink, from the 3rd valid frame of the lock on); the hook decodes it and counts. Sent: one stream slot, `MARV_TEST { seq: n, time_ms: board ms, value: -n, tag: 1 }`, `n` from 0 and one more per frame handed to the node, 25 bytes on the wire.
- **Load stream** (only with `RC_LINK_LOAD_HZ` above 0): ATTITUDE (id 30, 28-byte payload, 40 bytes on the wire), a counter from 1 in `time_boot_ms`, angles and rates 0 except `yawspeed` fixed at 0.001 rad/s, so that MAVLink 2 does not cut the trailing zeros and every frame is 40 bytes. The node adds its stream slowdown to both streams' intervals.
- **`[CNT]`** also prints `stream_slowdown_ms` (the node's slowdown, now in `rc_port::Stats`), next to `tx_bytes_mav`.

Build-time variables (decimal; above 1000 fails the build): `RC_LINK_TEST_HZ`, `MARV_TEST` frames per second, default 10 (0: no test stream); `RC_LINK_LOAD_HZ`, ATTITUDE frames per second, default 0 (no load slot at all). The banner prints the rates and the bytes per second they make before any slowdown (test 10 Hz: 250 B/s; load 100 Hz: 4000 B/s; HEARTBEAT 21 B/s). The documented downlink is about 1470 to 2375 bytes per second, so 100 Hz of load is about twice what the link carries.

New line, once a second:

`[TST] t=… sent=+10/1234 load_sent=+0/0 recv=+10/987 last_seq=986 tag=2 from=255/190 value=-986 time_ms=123456 lost=3 restarts=1 decode_failures=0`

`sent` and `load_sent`: `MARV_TEST` and ATTITUDE frames handed to the node this second / in all (a frame handed over just before a baud change can still be dropped by it). `recv`: `MARV_TEST` frames decoded this second / in all; then the latest one's `seq`, `tag` (2 = ground station), sender (system/component), `value` and `time_ms`; `lost` is the sum of `seq - previous - 1` over the frames whose `seq` was more than the previous + 1; `restarts` counts frames whose `seq` was not more than the previous (reordered, repeated or the sender restarted); `decode_failures` counts hand-overs that did not decode. Before the first: `[TST] t=… sent=… load_sent=… recv=+0/0 none yet decode_failures=…`.

### Run

From the crate root (`rc-port/`), as for `rc_follow` (wiring the same):

```sh
tools/run.sh rc_link 600                      # checks 1 and 2 (test 10 Hz, no load)
RC_LINK_LOAD_HZ=100 tools/run.sh rc_link 300  # check 3, overloaded
tools/run.sh rc_link 300                      # check 3, recovery: a second build with load 0
```

Read the log with `grep -E '\[(MODE|EDGE|STK|LNK|CNT|TST|ERR)\]' hil/logs/rc_link-<date>.log`.

### The three checks, and what to read

- **(1) Radio off in MAVLink mode, through the port.** Locked on MAVLink (`[MODE] -> Mavlink`), the ground station receiving. Radio off for 10 s or more, then on. Expected: NO `[MODE]` line (the mode stays Mavlink); `[LNK] radio_status` continues about 91 a second with `rssi=0`; `[TST]` `recv=+0/…` (the receive count stops) while `sent` goes on; no baud change (`mode_changes` unchanged, no `[MODE] -> Searching`). Radio on: `recv` counts again; `lost` rises by about the frames sent meanwhile, or `restarts` by one if the ground station restarted its count.
- **(2) `MARV_TEST` each way through the radio.** The ground-station tool sends `MARV_TEST` with `tag` 2 and counts what it receives (`tag` 1). Each way 100 frames or more: compare `[TST]` `recv` with what the tool sent, and the tool's received count with `[TST]` `sent`; at least 95 % intact each way, `decode_failures` 0, `from=` the ground station's system/component as the radio passes it, `value` and `seq` as the tool sent them.
- **(3) Downlink overloaded** (`RC_LINK_LOAD_HZ=100`). Expected: `[LNK] radio_status` `txbuf` falls (below 50, then below 20); `[CNT]` `stream_slowdown_ms` rises (by 20 or 60 per RADIO_STATUS, up to 2000 and a little over); `[CNT]` `tx_bytes_mav` per second and `[TST]` `sent` and `load_sent` per second fall toward what the link carries; recovery when the load is removed (the second build with load 0): `txbuf` back above 90, `stream_slowdown_ms` falling to 0, `tx_bytes_mav` back to about 271 a second (250 + 21). Note that a baud change (any leave of the MAVLink lock) also sets the slowdown to 0.

### Ground-station link: notes for whoever repeats it

- The telemetry network comes from Backpack, Telemetry, WiFi in the ELRS Lua script, not from "Enable Backpack WiFi" (that starts the update page). The network is named "ExpressLRS TX Backpack 000000" (password `expresslrs`); the computer got 10.0.0.100 and the backpack is 10.0.0.1.
- It needs TX backpack firmware 1.5.0 or newer; with 1.3.0 nothing was forwarded (nothing arrived on UDP 14550 and the board received 0 `MARV_TEST`). The run that worked used 1.5.9.
- The backpack sends MAVLink from 10.0.0.1:14555 to UDP port 14550 without being spoken to first, so a station that listens on 14550 and learns its peer from what arrives works.
- The backpack's access point answers every DNS name with its own address, so a computer joined to it as it comes loses name resolution. Set the computer's connection to ignore the network's DNS and never be a default route; on the owner's machine: `nmcli connection modify "ExpressLRS TX Backpack" ipv4.ignore-auto-dns yes ipv4.never-default yes` (the connection name is the one of the network joined).

### Not verified

- The overload as seen from the ground station; the slowdown's negative control (the slowdown switched off); a counted test over minutes; any other receiver, radio or ExpressLRS version.
