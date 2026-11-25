# rc-port hil

Part of [rc-port](../README.md)

Three on-board programs (own workspace) for the MARV V2 board and an ExpressLRS receiver. They decide nothing on the board: the log is
read afterwards ([how to read the output](../docs/hil-logs.md)). PIO0, PIO2, GPIO36-39 (the ESCs, DShot), the
ADC and DMA are never touched.

| Program | Does | Transmits |
|---|---|---|
| `rc_capture` | Listen-only capture of GPIO17 by two listeners at once: UART0 at the CRSF rate (`crsf_codec::Deframer`) and a `pio-uart` receiver on PIO1 at 460800 (`mavlink_device::Node`). Does not use `rc-port` | Nothing; GPIO14 is never configured |
| `rc_follow` | `RcPort::run` on UART0 (feature `embassy-rp-0-10`), logging modes, reports and counters | A fixed CRSF battery frame (`RC_FOLLOW_BATT_DV` decivolts, default 123; `RC_FOLLOW_BATT_PCT`, default 50; current and capacity 0; the ADC is not read) in CRSF mode, the node's own messages (system 1, component 1: heartbeat, replies, test parameters `RCF_TEST_I` 42 and `RCF_TEST_F` 1.5) in MAVLink mode; nothing while searching |
| `rc_link` | A copy of `rc_follow` plus the project's `MARV_TEST` message (id 42420) each way through `Hooks::mavlink`, an optional ATTITUDE load stream and `stream_slowdown_ms` in `[CNT]` | As `rc_follow`, plus `MARV_TEST` and the load |

## Wiring

| Receiver | Board |
|---|---|
| TX | R0 (GPIO17) |
| RX | T0 (GPIO14): optional for `rc_capture`; connect it for `rc_follow` and `rc_link` (driven) |
| Ground | Ground |
| Supply | Its own 5 V supply, or the board's if the receiver accepts it; the flight battery need not be connected |

`rc_follow` and `rc_link` use UART0 (`BufferedUart`, 256-byte rings, interrupt-driven, no DMA) with the RX pad's pull-up turned on
after the UART is built, the port's task on an interrupt executor (`SWI_IRQ_0`, NVIC priority 0xE0) and the logger in thread mode.

## Run

From the crate root; the probe's timeout ends a run (status 124 is that, not a failure).

```sh
tools/run.sh rc_capture 600
tools/run.sh rc_follow 600
tools/run.sh rc_link 600                      # test message 10 Hz, no load
RC_LINK_LOAD_HZ=100 tools/run.sh rc_link 300  # overload; build again with load 0 to see recovery
```

The terminal shows only the `[INFO]` banner; the full log is under `hil/logs/` (git-ignored). Read it with
`grep -E '\[(MODE|EDGE|STK|LNK|CNT|TST|ERR)\]'` (`rc_capture`: `\[(EVT|SUM|VAL|WIN)\]`).

| Build-time variable | Default | Meaning |
|---|---|---|
| `RC_CAPTURE_CRSF_BAUD` | 420000 | CRSF rate of `rc_capture`'s UART listener |
| `RC_FOLLOW_BATT_DV`, `RC_FOLLOW_BATT_PCT` | 123, 50 | The fixed battery frame (out of range fails the build) |
| `RC_LINK_TEST_HZ` | 10 | `MARV_TEST` frames per second (0: none; above 1000 fails the build) |
| `RC_LINK_LOAD_HZ` | 0 | ATTITUDE frames per second (0: no load slot); the downlink carries about 1470 to 2375 bytes per second |

## Ground-station link (for `rc_link`)

- The telemetry network comes from Backpack, Telemetry, WiFi in the ELRS Lua script, not from "Enable Backpack WiFi" (that starts the
  update page). It is named "ExpressLRS TX Backpack 000000" (password `expresslrs`); the computer gets 10.0.0.100, the backpack is 10.0.0.1.
- It needs TX backpack firmware 1.5.0 or newer; with 1.3.0 nothing is forwarded.
- The backpack sends MAVLink from 10.0.0.1:14555 to UDP port 14550 without being spoken to first, so a station that listens on 14550
  and learns its peer from what arrives works.
- The backpack's access point answers every DNS name with its own address; set the connection to ignore the network's DNS and never
  be a default route (`nmcli connection modify "<connection>" ipv4.ignore-auto-dns yes ipv4.never-default yes`).
