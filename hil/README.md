# rc-port bench programs (`hil/`)

Status: **BUILT ONLY, never run.** `rc_capture` compiles for the RP2350 (`cargo build --release --bin rc_capture` in this folder); it has not been flashed, and nothing below has been observed. A README claims only what ran; when it runs, name the log here.

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

The pin is shared as in dshot's `dshot_smoke` (dshot README rule B9): PIO1 is given GPIO17 first (`make_pio_pin` on a second handle, then `UartRx::new`, which turns the pad pull-up on), and THEN the UART takes the pad (function UART0 RX). The UART's pad write in embassy-rp 0.10 writes the whole word from rp-pac's default 0 with only input enable set (`uart/mod.rs` `init`, the `rx` pad write): by reading the code, **both pulls end up off and the pad floats**, the pull-up `UartRx` set and the reset pull-down alike, exactly as rc-port's own UART will leave it. This is not what the decision's section 7 assumed ("whether the RX pad's reset pull-down stays on"); the banner prints the pad's state after setup (`pull-down`, `pull-up`, `input enabled`), which settles it on the board.

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

- That the two listeners really share GPIO17 as designed (the banner's pad line shows funcsel 2 and the pull; both listeners counting frames shows the rest).
- That nothing is lost: the buffer sizes and the interrupt executor are argued from the rates, not measured.
- The pio-uart divider at 460800 and 150 MHz is 20 + 88/256 (about +64 ppm, inside its tolerance by arithmetic); the PL011 at 420000 from 150 MHz is about +400 ppm. Neither is measured.
- The HAL reports one error per read when its ring is empty, the most severe of those seen (overrun, then break, parity, framing), and drops the byte that carried it: the error counts are counts of indications, not of bytes.
