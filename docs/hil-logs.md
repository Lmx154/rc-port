# Reading the bench program output

Part of [rc-port](../README.md)

The programs in [hil](../hil/README.md) print tagged lines and decide nothing: there is no PASS/FAIL, and the log is read
afterwards. Every line starts with `t=`, the board's time in ms; `_us` fields are on the port's clock. `Some(x)`/`None` is an
optional value.

## `rc_capture`

Two listeners on GPIO17 count what arrives; the program does not use `rc-port`.

| Tag | When | Content |
|---|---|---|
| `[EVT]` | A stream starts or stops | For `crsf_rc` (0x16), `crsf_ls` (0x14), `mav_ovr` (RC_CHANNELS_OVERRIDE), `mav_rst` (RADIO_STATUS): `started at=.. gap_ms=..` (first valid frame after 100 ms or more without one), `stopped last=.. ran_ms=..` (none for 100 ms); after `crsf_ls` stops, its last frame (`lq`, `rssi`, `snr`, `rf`) |
| `[SUM]` | Each second | Counts: `crsf: bytes rc ls other crc badlen garb uart(fr/pa/br/ov) \| mav: bytes ovr rst hb unk crc garb v1 pio(fr/ov/runt)`; `v1` is MAVLink 1 frames, which the node drops |
| `[VAL]` | Each second | Latest values: `rc_us` through `raw_to_us`, `ovr_us` as received, `line` (GPIO17 level), `lq`, `rssi`, `snr`, `rf`, override and RADIO_STATUS senders, `txbuf`, noise; `B=false` if listener B could not be built |
| `[WIN]` | Each 10 s | Totals of the last 10 s of every stream and error counter |

Reading it: in the Normal model `rc` near the link frame rate with `crc=0`, `badlen=0` and no UART errors means the CRSF baud is right
(many `garb`, `crc` and framing errors mean another baud; rebuild with `RC_CAPTURE_CRSF_BAUD`). On a model change the `[EVT]` pairs
give the re-open gap (started minus last). In the MAVLink model `mav_ovr=0` with `mav_rst` above 500 per 10 s window shows the
missing stick messages. With the receiver unpowered all four streams read 0. `uart ov` and `pio ov` should stay 0.

## `rc_follow` and `rc_link`

| Tag | When | Content |
|---|---|---|
| `[MODE]` | Every change of `Stats::mode` | `-> Crsf` / `-> Mavlink` with `since_us`, `seen_us`, `changes`, `prev`, `prev_lasted_us`, `discarded`, `discardN_to_lock_us`, `old_last_frame_to_lock_us`; `-> Searching` with `last_valid_frame_us`, `last_stick_us`, `seen_after_last_stick_us`. `since_us` is the locking frame's time, or for Searching the deadline at which the lock was left (last valid frame plus the leave time) |
| `[EDGE]` | A stick stream starts or stops | `sticks started at_us gap_ms`, `sticks stopped last_us ran_ms`, `first stick report in Crsf at_us after_lock_us after_discard1_us` |
| `[STK]` | Each second with stick reports | `n`, `src` (`crsf` or `mavlink 255/68`), `iv_us=MIN..MAX`, `ch1-8`, `ch15`, `ch16`, `seq`; or `none mode=..` while locked without sticks |
| `[LNK]` | Each second | `crsf n lq rssi snr rf txpwr seq`, and/or `radio_status n rssi remrssi txbuf noise remnoise rxerrors fixed from seq`, else `none` |
| `[CNT]` | Each second | `+delta/total` of every `Stats` counter, then `tx_bytes_crsf`, `tx_bytes_mav` (bytes the UART took at each baud) and `restarts`; `rc_link` adds `stream_slowdown_ms` |
| `[ERR]` | If `run` returns | `run returned Write(..)\|WriteZero\|EndOfStream; restart N in 100 ms; node stream_slowdown_ms=..` (the slowdown cannot be read while `run` is awaited) |
| `[TST]` (`rc_link`) | Each second | `sent=+n/total load_sent recv=+n/total last_seq tag from value time_ms lost restarts decode_failures`: `sent` and `load_sent` are `MARV_TEST` and ATTITUDE frames handed to the node; `recv` is `MARV_TEST` frames decoded; `tag` 2 is the ground station; `lost` is the sum of `seq - previous - 1` where seq jumped; `restarts` counts frames whose `seq` was not above the previous |

The hooks only copy into one `critical_section::Mutex` snapshot (they never block or format); the logger looks at it every 10 ms and
does the formatting.

## What to read for each check

| Check | Expected in the log |
|---|---|
| CRSF in | `[STK] ch1-8` follow the sticks, `iv_us` near the frame period, `[LNK] crsf` about 10 a second. Radio off: `[EDGE] sticks stopped`, `[STK] none`, then `[MODE] -> Searching` with `seen_after_last_stick_us` about 1 000 000. Radio on: `[MODE] -> Crsf` with `discarded=2` and the first stick report within 3 frame periods. `crc_errors` and `uart_*` 0 while locked |
| CRSF telemetry out | The radio's sensor screen shows the fixed battery value; `tx_bytes_crsf` about +120 a second, `tx_bytes_mav` +0 |
| MAVLink in | `[MODE] -> Mavlink`, `[LNK] radio_status` about 91 a second, `[STK] src=mavlink 255/68`. Radio off: `[EDGE] sticks stopped`, `radio_status` continuing with `rssi=0`, and no `[MODE]` line |
| Following | `mode_changes` +1 per change; each lock's `old_last_frame_to_lock_us` against `max(T_LEAVE_old, G) + 280 ms`; no `[STK]` line while searching and no report across a change |
| `rc_link` 1 | Radio off in MAVLink mode: no `[MODE]` line, `recv` stops while `sent` goes on, no baud change; `lost` rises or `restarts` by one on return |
| `rc_link` 2 | `MARV_TEST` each way, 100 frames or more: at least 95 % intact each way, `decode_failures` 0 |
| `rc_link` 3 | Overload: `txbuf` falls below 50 then 20, `stream_slowdown_ms` rises toward 2000, `tx_bytes_mav` falls toward what the link carries; after removing the load `txbuf` back above 90, slowdown to 0, `tx_bytes_mav` about 271 a second (250 + 21) |

`uart_overrun` should stay 0 and `[ERR]` should never appear.

## Related

[hil](../hil/README.md), [reports](reports.md).
