//! Invariant I5.2: a command heard before the receiver port's lock runs only if the port locks
//! on MAVLink before the next baud change. It goes through `RcPort` as it is, with no change to
//! the crate: the node's `Telemetry::command` hook runs when its reply comes up in
//! `poll_transmit`, which the port calls only while locked on MAVLink, and every baud change
//! drops the waiting replies (`Node::on_link_down`).
//!
//! The model and helpers of `src/tests.rs` and `src/sim.rs` are private to the crate and are
//! `cfg(test)` code under `src/`, so this test drives the sans-IO core by hand on a made-up clock.

use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, MessageData};
use mavlink_device::dialect::RADIO_STATUS_DATA;
use mavlink_device::marv::{COMMAND_ACK_DATA, COMMAND_LONG_DATA, MavCmd, MavResult};
use mavlink_device::{
    CommandOutcome, CommandRequest, Outgoing, ParamError, ParamName, ParamStore, ParamValue,
    Telemetry,
};
use rc_port::{Action, CRSF_BAUD, Config, Hooks, MAVLINK_BAUD, Mode, RcPort, Report};

const OWN_SYS: u8 = 1;
/// The ground station that sends the command, through the radio.
const GCS: (u8, u8) = (255, 190);
/// The receiver, sender of RADIO_STATUS.
const ELRS: (u8, u8) = (51, 68);

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

/// The firmware's command hook: counts the calls and accepts.
#[derive(Default)]
struct Firmware {
    calls: Vec<(CommandRequest, u64)>,
}

impl Telemetry for Firmware {
    fn sample(&mut self, _: u32, _: u64, _: &mut Outgoing<'_>) -> bool {
        false
    }
    fn command(&mut self, req: &CommandRequest, now_us: u64) -> CommandOutcome {
        self.calls.push((*req, now_us));
        CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED)
    }
}

struct NoHooks;

impl Hooks for NoHooks {
    fn report(&mut self, _: Report) {}
}

fn mav<M: MessageData>(sys_comp: (u8, u8), seq: u8, m: &M) -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message_data(
        MavHeader {
            system_id: u32::from(sys_comp.0),
            component_id: sys_comp.1,
            sequence: seq,
        },
        m,
    );
    raw.raw_bytes().to_vec()
}

fn calibration_command() -> Vec<u8> {
    mav(
        GCS,
        0,
        &COMMAND_LONG_DATA {
            param1: 1.0,
            command: MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            target_system: OWN_SYS,
            target_component: 1,
            ..Default::default()
        },
    )
}

fn radio_status(seq: u8) -> Vec<u8> {
    mav(
        ELRS,
        seq,
        &RADIO_STATUS_DATA {
            rssi: 200,
            txbuf: 100,
            ..Default::default()
        },
    )
}

/// A CRSF link statistics frame (address 0xC8, type 0x14, 10 bytes, CRC-8 of 0xD5).
fn crsf_link_frame() -> Vec<u8> {
    let mut f = vec![0xC8, 12, 0x14, 60, 61, 100, 5, 0, 7, 3, 70, 99, 8];
    let mut crc = 0u8;
    for &b in &f[2..] {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    f.push(crc);
    f
}

struct Rig<'a> {
    port: RcPort<'a>,
    fw: Firmware,
    now: u64,
    /// Every baud the port set, in order.
    bauds: Vec<u32>,
    /// Every frame the port wrote, whole (the node writes whole frames).
    written: Vec<Vec<u8>>,
}

impl Rig<'_> {
    fn receive(&mut self, bytes: &[u8]) {
        self.port
            .receive(bytes, self.now, &mut NoParams, &mut NoHooks);
    }

    /// `poll_action` until `None`, doing what each action asks, at `self.now`.
    fn pump(&mut self) {
        loop {
            let action = self
                .port
                .poll_action(self.now, &NoParams, &mut self.fw, &mut NoHooks);
            match action {
                None => return,
                Some(Action::Write(bytes)) => {
                    let frame = bytes.to_vec();
                    self.port.written(frame.len());
                    self.written.push(frame);
                }
                Some(Action::SetBaud(baud)) => {
                    self.bauds.push(baud);
                    self.port.done(self.now);
                }
                Some(Action::Flush | Action::WaitTxIdle) => self.port.done(self.now),
            }
        }
    }

    /// Advances the clock by `us` and pumps.
    fn wait(&mut self, us: u64) {
        self.now += us;
        self.pump();
    }

    /// Three valid frames at the current baud lock the port (RADIO_STATUS at 10 ms).
    fn feed_radio_status_x3(&mut self) {
        for seq in 0..3 {
            self.now += 10_000;
            self.receive(&radio_status(seq));
        }
    }

    /// The COMMAND_ACKs among what the port wrote.
    fn acks(&self) -> Vec<COMMAND_ACK_DATA> {
        self.written
            .iter()
            .filter(|f| u32::from_le_bytes([f[7], f[8], f[9], 0]) == COMMAND_ACK_DATA::ID)
            .map(|f| {
                let len = usize::from(f[1]);
                COMMAND_ACK_DATA::deser(MavlinkVersion::V2, &f[10..10 + len]).unwrap()
            })
            .collect()
    }
}

fn rig<'a>() -> Rig<'a> {
    let port = RcPort::new(
        Config::new(),
        mavlink_device::Config::new(OWN_SYS, 1),
        &mut [],
    )
    .unwrap();
    let mut rig = Rig {
        port,
        fw: Firmware::default(),
        now: 1_000,
        bauds: Vec::new(),
        written: Vec::new(),
    };
    rig.pump();
    assert_eq!(rig.bauds, [MAVLINK_BAUD]);
    assert_eq!(rig.port.stats().mode, Mode::Searching);
    rig
}

#[test]
fn i5_2_a_command_heard_before_the_lock_runs_only_if_the_port_locks_on_mavlink_before_the_next_baud_change()
 {
    // Case 1: heard while searching at 460800; the port then locks on MAVLink in the same
    // listening period (the dwell is 50 ms).
    let mut r = rig();
    r.now += 1_000;
    r.receive(&calibration_command());
    r.pump();
    assert_eq!(r.port.stats().mode, Mode::Searching);
    assert_eq!(
        r.fw.calls.len(),
        0,
        "not run by receive, not run while searching"
    );
    assert!(r.written.is_empty(), "nothing is written while searching");
    r.feed_radio_status_x3();
    r.pump();
    assert_eq!(
        r.port.stats().mode,
        Mode::Mavlink,
        "locked on the third frame"
    );
    assert_eq!(r.fw.calls.len(), 1, "run once, when the port may write");
    let (req, at) = r.fw.calls[0];
    assert_eq!(req.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
    assert_eq!(req.params[0], 1.0);
    assert_eq!((req.sender.system_id, req.sender.component_id), GCS);
    assert!(
        at >= r.now - 1_000,
        "called from poll_action after the lock"
    );
    let acks = r.acks();
    assert_eq!(acks.len(), 1, "the ACK is written, once");
    assert_eq!(acks[0].command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
    assert_eq!(acks[0].result, MavResult::MAV_RESULT_ACCEPTED);
    assert_eq!((acks[0].target_system, acks[0].target_component), GCS);
    r.wait(10_000);
    r.wait(10_000);
    assert_eq!(r.fw.calls.len(), 1);
    assert_eq!(r.acks().len(), 1);

    // Case 2: heard while searching at 460800; the port moves on (the listening period ends
    // with fewer than 3 frames, the baud changes to CRSF), and then locks on CRSF.
    let mut r = rig();
    r.now += 1_000;
    r.receive(&calibration_command());
    r.now += 10_000;
    r.receive(&radio_status(0)); // frame 1 of 3 only
    r.pump();
    assert_eq!(r.fw.calls.len(), 0);
    r.wait(60_000); // the dwell of 50 ms ends: Flush, WaitTxIdle, SetBaud
    assert_eq!(
        r.bauds,
        [MAVLINK_BAUD, CRSF_BAUD],
        "the next baud change happened"
    );
    for _ in 0..3 {
        r.now += 20_000;
        r.receive(&crsf_link_frame());
    }
    r.pump();
    assert_eq!(r.port.stats().mode, Mode::Crsf, "locked on CRSF instead");
    r.wait(100_000);
    assert_eq!(r.fw.calls.len(), 0, "the hook never runs");
    assert!(
        r.written.is_empty(),
        "no MAVLink frame is written in CRSF mode"
    );

    // ... and later the port goes back to MAVLink and locks there: the command is gone.
    r.wait(1_100_000); // no CRSF frame for T_LEAVE_CRSF: leave, the search starts at 460800
    assert_eq!(r.bauds.last(), Some(&MAVLINK_BAUD));
    r.feed_radio_status_x3();
    r.pump();
    assert_eq!(r.port.stats().mode, Mode::Mavlink);
    r.wait(10_000);
    assert_eq!(
        r.fw.calls.len(),
        0,
        "the command was dropped by the baud change"
    );
    assert!(r.acks().is_empty(), "no ACK is ever written");
    assert!(
        !r.written.is_empty(),
        "the node does write (its heartbeat): the lock is real"
    );
}

fn heartbeat(sys_comp: (u8, u8), mavtype: mavlink_device::dialect::MavType) -> Vec<u8> {
    mav(
        sys_comp,
        0,
        &mavlink_device::dialect::HEARTBEAT_DATA {
            mavtype,
            mavlink_version: 3,
            ..Default::default()
        },
    )
}

#[test]
fn stats_last_gcs_heartbeat_us_is_the_receive_time_of_the_station_heartbeat_and_survives_a_baud_change() {
    use mavlink_device::dialect::MavType;
    let mut r = rig();
    assert_eq!(r.port.stats().last_gcs_heartbeat_us, None);
    r.feed_radio_status_x3();
    r.pump();
    assert_eq!(r.port.stats().mode, Mode::Mavlink);
    r.now += 5_000;
    r.receive(&heartbeat(ELRS, MavType::MAV_TYPE_GENERIC));
    assert_eq!(r.port.stats().last_gcs_heartbeat_us, None, "not a ground station");
    r.now += 5_000;
    let t = r.now;
    r.receive(&heartbeat(GCS, MavType::MAV_TYPE_GCS));
    let s = r.port.stats();
    assert_eq!(s.last_gcs_heartbeat_us, Some(t), "the receive time");
    assert!(s.last_radio_status_us.is_some_and(|rs| rs < t), "the clock RADIO_STATUS is stamped in");
    assert_eq!(r.port.node().gcs_seen_us(), Some(t));
    // Silence: the port leaves the lock and changes baud; the node forgets the station, the stat does not.
    for _ in 0..200 {
        r.wait(10_000);
    }
    assert!(r.bauds.len() > 1, "a baud change happened");
    assert_eq!(r.port.node().gcs_seen_us(), None);
    assert_eq!(r.port.stats().last_gcs_heartbeat_us, Some(t));
    assert_eq!(r.port.stats().last_gcs_heartbeat_sender, Some(GCS));
}

#[test]
fn stats_last_gcs_heartbeat_sender_follows_the_heartbeat_that_set_the_time() {
    use mavlink_device::dialect::MavType;
    let mut r = rig();
    assert_eq!(r.port.stats().last_gcs_heartbeat_sender, None);
    r.feed_radio_status_x3();
    r.pump();
    r.now += 5_000;
    r.receive(&heartbeat(ELRS, MavType::MAV_TYPE_GENERIC));
    let s = r.port.stats();
    assert_eq!((s.last_gcs_heartbeat_us, s.last_gcs_heartbeat_sender), (None, None));
    r.now += 5_000;
    let t1 = r.now;
    r.receive(&heartbeat(GCS, MavType::MAV_TYPE_GCS));
    let s = r.port.stats();
    assert_eq!((s.last_gcs_heartbeat_us, s.last_gcs_heartbeat_sender), (Some(t1), Some(GCS)));
    r.now += 5_000;
    r.receive(&heartbeat((7, 9), MavType::MAV_TYPE_GENERIC));
    let s = r.port.stats();
    assert_eq!((s.last_gcs_heartbeat_us, s.last_gcs_heartbeat_sender), (Some(t1), Some(GCS)));
    r.now += 5_000;
    let t2 = r.now;
    r.receive(&heartbeat(ELRS, MavType::MAV_TYPE_GCS));
    let s = r.port.stats();
    assert_eq!((s.last_gcs_heartbeat_us, s.last_gcs_heartbeat_sender), (Some(t2), Some(ELRS)));
    r.now += 5_000;
    let t3 = r.now;
    r.receive(&heartbeat(GCS, MavType::MAV_TYPE_GCS));
    let s = r.port.stats();
    assert_eq!((s.last_gcs_heartbeat_us, s.last_gcs_heartbeat_sender), (Some(t3), Some(GCS)));
    r.now += 5_000;
    let t4 = r.now;
    r.receive(&heartbeat(ELRS, MavType::MAV_TYPE_GCS));
    for _ in 0..200 {
        r.wait(10_000);
    }
    assert!(r.bauds.len() > 1, "a baud change happened");
    let s = r.port.stats();
    assert_eq!((s.last_gcs_heartbeat_us, s.last_gcs_heartbeat_sender), (Some(t4), Some(ELRS)));
}
