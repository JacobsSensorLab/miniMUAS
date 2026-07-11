//! RC-CONTROL R2 pilot surface — the dashboard's send side.
//!
//! # How a stick sample travels (proof this is NOT a shortcut)
//!
//! The browser's Gamepad/keyboard poll maps sticks to AETR channels and
//! streams them over the dashboard WebSocket (`{"cmd":"rc",…}`). The WS
//! handler feeds the latest sample into this host, whose 50 Hz loop
//! synthesizes a real [`uas_rc::RcFrame`], paces it through the same
//! [`uas_rc::RatePacer`] a console pilot node uses, and sends it with a
//! [`uas_rc::UdpRcSender`] to the vehicle's agent `--rc listen:<addr>` UDP
//! socket. That socket is the agent's PRODUCTION RC receiver
//! ([`UdpRcReceiver`](uas_rc::UdpRcReceiver) folded into its
//! `RcReceiverTask`): the frames pass its seq-gap ledger, its engage
//! arbitration, its silence-ladder failsafe, and drive its flight backend —
//! exactly as a bound transmitter's would. Nothing is short-circuited in
//! the dashboard process; the wire is the interface.
//!
//! R2 rides the plain-UDP `listen:` binding (R1's default). The spark
//! carriage (`--rc spark:`) is a receiver-side option the agent also
//! offers; wiring the dashboard's `SparkRcSender` onto the NDN fabric is
//! deferred to a later increment (it needs the `spark` feature and the
//! keying ceremony of RC-CONTROL R4), and is intentionally NOT a
//! dashboard-only path either.
//!
//! # AETR mapping (pinned to the wire layout)
//!
//! Channels are RC_CHANNELS_OVERRIDE µs (`1000..=2000`, `1500` center,
//! `65535` = ignore). The four flight axes occupy channels 1..4 in AETR
//! order — the order [`uas_rc::RcFrame`] lays out at wire offsets 10/12/14/16
//! and the order ArduPilot's default `RCMAP` expects:
//!
//! | channel | index | axis     |
//! |---------|-------|----------|
//! | ch1     | 0     | roll  (Aileron)  |
//! | ch2     | 1     | pitch (Elevator) |
//! | ch3     | 2     | throttle         |
//! | ch4     | 3     | yaw   (Rudder)   |

use std::collections::BTreeMap;
use std::net::SocketAddr;
use std::sync::Mutex;
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio_util::sync::CancellationToken;
use tracing::{info, warn};
use uas_rc::{RatePacer, RcFlags, RcFrame, RcTarget, UdpRcSender, DEFAULT_RC_HZ};

use crate::lock;

/// AETR channel indices into the 8-wide [`RcFrame::channels`] array.
pub const CH_ROLL: usize = 0;
/// Pitch (elevator) → channel 2.
pub const CH_PITCH: usize = 1;
/// Throttle → channel 3.
pub const CH_THROTTLE: usize = 2;
/// Yaw (rudder) → channel 4.
pub const CH_YAW: usize = 3;

/// Neutral stick µs.
pub const CENTER_US: u16 = 1500;
/// Minimum commanded PWM µs.
pub const MIN_US: u16 = 1000;
/// Maximum commanded PWM µs.
pub const MAX_US: u16 = 2000;
/// RC_CHANNELS_OVERRIDE "ignore this channel" sentinel.
pub const IGNORE_US: u16 = 65535;

/// Sender tick (matches [`DEFAULT_RC_HZ`] = 50 Hz; the pacer gates to the
/// same rate, so at steady state one synthesized frame leaves per tick).
const RC_TICK: Duration = Duration::from_millis(20);

/// Centered flight axes with channels 5..8 left ignored — the resting
/// frame a freshly engaged session emits until sticks move.
pub fn center_channels() -> [u16; 8] {
    let mut ch = [IGNORE_US; 8];
    ch[CH_ROLL] = CENTER_US;
    ch[CH_PITCH] = CENTER_US;
    ch[CH_THROTTLE] = CENTER_US;
    ch[CH_YAW] = CENTER_US;
    ch
}

/// Clamp a browser-supplied channel array into wire-legal values: the
/// ignore sentinel passes untouched, everything else is pinned to
/// `1000..=2000` (a malformed/oversized value can never command an
/// out-of-band PWM). Short arrays pad with ignore, long ones truncate.
pub fn sanitize_channels(raw: &[i64]) -> [u16; 8] {
    let mut out = [IGNORE_US; 8];
    for (o, &v) in out.iter_mut().zip(raw.iter()) {
        *o = if v == i64::from(IGNORE_US) {
            IGNORE_US
        } else {
            v.clamp(i64::from(MIN_US), i64::from(MAX_US)) as u16
        };
    }
    out
}

/// Compose the discrete-control flags a frame carries. `mode` is masked to
/// the 3-bit mode field by [`RcFlags::with_mode`].
pub fn compose_flags(arm: bool, estop: bool, mode: u8) -> RcFlags {
    let mut f = RcFlags::EMPTY;
    if arm {
        f |= RcFlags::ARM_GESTURE;
    }
    if estop {
        f |= RcFlags::EMERGENCY_STOP;
    }
    f.with_mode(mode)
}

/// Expand a [`RcTarget`] against the configured roster into the vehicle ids
/// that actually receive frames (broadcast fans to every RC-reachable
/// vehicle in stable order; a `One` target that is not RC-reachable expands
/// to nothing). Mirrors [`RcTarget::stream_names`] selector semantics, but
/// keyed to the dashboard's `vid -> udp addr` map instead of NDN names.
pub fn expand_targets(target: &RcTarget, roster: &BTreeMap<String, SocketAddr>) -> Vec<String> {
    match target {
        RcTarget::One(vid) => {
            if roster.contains_key(vid) {
                vec![vid.clone()]
            } else {
                Vec::new()
            }
        }
        RcTarget::Broadcast => roster.keys().cloned().collect(),
    }
}

/// One UDP lane toward a vehicle's agent RC receiver: its fire-and-forget
/// sender, its own send-side pacer, and its monotonic frame counter.
struct RcLink {
    sender: UdpRcSender,
    pacer: RatePacer,
    seq: u32,
}

impl RcLink {
    /// Assign the next seq to `frame` and advance the counter; a paced-out
    /// (dropped) frame therefore surfaces on the receiver as an honest seq
    /// gap, never a repair.
    fn stamp(&mut self, mut frame: RcFrame) -> RcFrame {
        frame.seq = self.seq;
        self.seq = self.seq.wrapping_add(1);
        frame
    }
}

/// Loop-local pilot input (latest-wins): the engaged target (or `None`),
/// the current sticks, and the discrete controls. The e-stop latch is held
/// here so every subsequent paced frame keeps carrying the flag until the
/// operator clears it — the agent's release gesture.
#[derive(Clone)]
struct RcInput {
    session: Option<RcTarget>,
    channels: [u16; 8],
    arm: bool,
    estop: bool,
    mode: u8,
}

impl Default for RcInput {
    fn default() -> Self {
        Self { session: None, channels: center_channels(), arm: false, estop: false, mode: 0 }
    }
}

/// The pilot-surface send host: one UDP lane per RC-reachable vehicle plus
/// the shared latest-input slot the 50 Hz loop drains.
pub struct RcHost {
    roster: BTreeMap<String, SocketAddr>,
    links: Mutex<BTreeMap<String, RcLink>>,
    input: Mutex<RcInput>,
    t0: Instant,
}

impl RcHost {
    /// Build a host over the configured `vid -> agent rc listen addr` map.
    /// A lane whose socket cannot be opened is logged and skipped (the
    /// vehicle is simply not RC-reachable); an empty roster yields an inert
    /// host.
    pub fn new(roster: &BTreeMap<String, SocketAddr>) -> Arc<Self> {
        let mut links = BTreeMap::new();
        let mut reachable = BTreeMap::new();
        for (vid, addr) in roster {
            match UdpRcSender::connect(addr) {
                Ok(sender) => {
                    links.insert(
                        vid.clone(),
                        RcLink { sender, pacer: RatePacer::new(DEFAULT_RC_HZ), seq: 0 },
                    );
                    reachable.insert(vid.clone(), *addr);
                    info!(vid, %addr, "rc pilot lane up");
                }
                Err(err) => warn!(vid, %addr, %err, "rc pilot lane failed; vehicle not pilotable"),
            }
        }
        Arc::new(Self {
            roster: reachable,
            links: Mutex::new(links),
            input: Mutex::new(RcInput::default()),
            t0: Instant::now(),
        })
    }

    /// The RC-reachable vehicle ids (drives the Pilot selector + hello).
    pub fn vehicles(&self) -> Vec<String> {
        self.roster.keys().cloned().collect()
    }

    /// Engage a target. Returns the vehicle ids now under this session
    /// (empty = the target expanded to no reachable vehicle, so nothing
    /// engaged). Sticks reset to center on engage.
    pub fn engage(&self, target: RcTarget) -> Vec<String> {
        let vids = expand_targets(&target, &self.roster);
        if vids.is_empty() {
            return vids;
        }
        let mut inp = lock(&self.input);
        inp.session = Some(target);
        inp.channels = center_channels();
        inp.arm = false;
        inp.estop = false;
        inp.mode = 0;
        vids
    }

    /// Update the latest sticks + discrete controls (the 50 Hz browser
    /// stream). No-op fields (e-stop latch) are preserved.
    pub fn set_input(&self, channels: [u16; 8], arm: bool, mode: u8) {
        let mut inp = lock(&self.input);
        inp.channels = channels;
        inp.arm = arm;
        inp.mode = mode;
    }

    /// Release the session locally: stop synthesizing frames and drop the
    /// e-stop latch. Returns the vehicle ids that were engaged so the caller
    /// can send each the agent-side `rc_disengage` service op. (Stream
    /// silence would release them on its own; this is the explicit path.)
    pub fn disengage(&self) -> Vec<String> {
        let mut inp = lock(&self.input);
        let vids = inp.session.take().map(|t| expand_targets(&t, &self.roster)).unwrap_or_default();
        inp.estop = false;
        inp.channels = center_channels();
        vids
    }

    /// Set/clear the e-stop latch. Turning it ON sends a flagged frame
    /// IMMEDIATELY to every engaged lane, bypassing the pacer (P11: the
    /// safing e-stop must reach the vehicle in under one frame period);
    /// subsequent paced frames keep the flag until it is cleared.
    pub fn estop(&self, on: bool) {
        let target = {
            let mut inp = lock(&self.input);
            inp.estop = on;
            inp.session.clone()
        };
        if on {
            if let Some(target) = target {
                self.send_immediate(&target);
            }
        }
    }

    /// Whether a session is currently engaged.
    pub fn engaged(&self) -> bool {
        lock(&self.input).session.is_some()
    }

    /// Send one un-paced frame from the current input to each engaged lane
    /// (the e-stop bypass).
    fn send_immediate(&self, target: &RcTarget) {
        let (channels, flags) = self.frame_fields();
        let t_ms = self.t_ms();
        let mut links = lock(&self.links);
        for vid in expand_targets(target, &self.roster) {
            if let Some(link) = links.get_mut(&vid) {
                let frame = link.stamp(RcFrame { seq: 0, t_ms, channels, flags });
                if let Err(err) = link.sender.send(&frame) {
                    warn!(vid, %err, "rc e-stop frame send failed");
                }
            }
        }
    }

    /// One paced send pass — the body of the 50 Hz loop, exposed so tests
    /// drive it deterministically. Offers a freshly synthesized frame to
    /// each engaged lane's pacer and sends whatever the pacer releases.
    pub fn tick(&self, now_ms: u64) {
        let (target, channels, flags) = {
            let inp = lock(&self.input);
            let Some(target) = inp.session.clone() else { return };
            (target, inp.channels, compose_flags(inp.arm, inp.estop, inp.mode))
        };
        let t_ms = now_ms as u32;
        let mut links = lock(&self.links);
        for vid in expand_targets(&target, &self.roster) {
            if let Some(link) = links.get_mut(&vid) {
                let frame = link.stamp(RcFrame { seq: 0, t_ms, channels, flags });
                if let Some(out) = link.pacer.offer(frame, now_ms) {
                    if let Err(err) = link.sender.send(&out) {
                        warn!(vid, %err, "rc frame send failed");
                    }
                }
            }
        }
    }

    fn frame_fields(&self) -> ([u16; 8], RcFlags) {
        let inp = lock(&self.input);
        (inp.channels, compose_flags(inp.arm, inp.estop, inp.mode))
    }

    fn t_ms(&self) -> u32 {
        self.t0.elapsed().as_millis() as u32
    }

    /// Run the 50 Hz sender loop until cancelled.
    pub async fn run(self: Arc<Self>, cancel: CancellationToken) {
        let mut interval = tokio::time::interval(RC_TICK);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                () = cancel.cancelled() => break,
                _ = interval.tick() => {}
            }
            let now_ms = self.t0.elapsed().as_millis() as u64;
            self.tick(now_ms);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant as StdInstant;
    use uas_rc::{RcFrame, UdpRcReceiver, RC_FRAME_VERSION};

    fn roster(pairs: &[(&str, &str)]) -> BTreeMap<String, SocketAddr> {
        pairs.iter().map(|(v, a)| (v.to_string(), a.parse().unwrap())).collect()
    }

    #[test]
    fn aetr_order_pinned_to_the_wire_layout() {
        // Distinct per-axis values so a mis-ordered map can't pass.
        let mut ch = center_channels();
        ch[CH_ROLL] = 1100;
        ch[CH_PITCH] = 1200;
        ch[CH_THROTTLE] = 1300;
        ch[CH_YAW] = 1400;
        let b = RcFrame { seq: 0, t_ms: 0, channels: ch, flags: RcFlags::EMPTY }.encode();
        assert_eq!(b[0], RC_FRAME_VERSION);
        // ch1..4 sit at wire offsets 10,12,14,16 (frames.rs layout).
        assert_eq!(&b[10..12], &1100u16.to_le_bytes(), "ch1 = roll");
        assert_eq!(&b[12..14], &1200u16.to_le_bytes(), "ch2 = pitch");
        assert_eq!(&b[14..16], &1300u16.to_le_bytes(), "ch3 = throttle");
        assert_eq!(&b[16..18], &1400u16.to_le_bytes(), "ch4 = yaw");
        // channels 5..8 ride the ignore sentinel.
        assert_eq!(&b[18..20], &IGNORE_US.to_le_bytes(), "ch5 ignored");
    }

    #[test]
    fn sanitize_clamps_and_preserves_ignore() {
        let got = sanitize_channels(&[500, 2500, 1500, 65535, 1750]);
        assert_eq!(got[CH_ROLL], MIN_US, "under-range clamps up");
        assert_eq!(got[CH_PITCH], MAX_US, "over-range clamps down");
        assert_eq!(got[CH_THROTTLE], 1500);
        assert_eq!(got[3], IGNORE_US, "the ignore sentinel passes untouched");
        assert_eq!(got[4], 1750);
        assert_eq!(got[5], IGNORE_US, "unspecified channels default to ignore");
        // Over-long input truncates to 8.
        assert_eq!(sanitize_channels(&[1500; 12]).len(), 8);
    }

    #[test]
    fn compose_flags_carries_arm_estop_and_mode() {
        assert_eq!(compose_flags(false, false, 0), RcFlags::EMPTY);
        assert!(compose_flags(true, false, 0).contains(RcFlags::ARM_GESTURE));
        assert!(compose_flags(false, true, 0).contains(RcFlags::EMERGENCY_STOP));
        let both = compose_flags(true, true, 5);
        assert!(both.contains(RcFlags::ARM_GESTURE));
        assert!(both.contains(RcFlags::EMERGENCY_STOP));
        assert_eq!(both.mode(), 5, "mode rides the 3-bit field alongside the flags");
    }

    #[test]
    fn expand_targets_honors_one_and_broadcast() {
        let r = roster(&[
            ("iuas-01", "127.0.0.1:1"),
            ("iuas-02", "127.0.0.1:2"),
            ("wuas-01", "127.0.0.1:3"),
        ]);
        assert_eq!(expand_targets(&RcTarget::One("iuas-02".into()), &r), vec!["iuas-02"]);
        assert!(
            expand_targets(&RcTarget::One("ghost".into()), &r).is_empty(),
            "an unreachable vehicle expands to nothing"
        );
        // Broadcast fans to every reachable vehicle, in stable (BTreeMap) order.
        assert_eq!(
            expand_targets(&RcTarget::Broadcast, &r),
            vec!["iuas-01", "iuas-02", "wuas-01"]
        );
        assert!(expand_targets(&RcTarget::Broadcast, &BTreeMap::new()).is_empty());
    }

    /// Poll a non-blocking receiver until it yields a frame or the budget
    /// expires.
    fn recv_frame(rx: &mut UdpRcReceiver) -> Option<RcFrame> {
        let deadline = StdInstant::now() + Duration::from_secs(2);
        while StdInstant::now() < deadline {
            if let Some(frame) = rx.poll().expect("poll") {
                return Some(frame);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        None
    }

    /// The honesty test: a dashboard [`RcHost`] lane sends real frames into
    /// a uas-rc [`UdpRcReceiver`] — the same receiver the agent binds — with
    /// the pilot's AETR channels intact.
    #[test]
    fn loopback_frames_reach_a_uas_rc_receiver_with_sticks() {
        let mut rx = UdpRcReceiver::bind("127.0.0.1:0").unwrap();
        let addr = rx.local_addr().unwrap();
        let host = RcHost::new(&roster(&[("iuas-02", &addr.to_string())]));

        assert_eq!(host.engage(RcTarget::One("iuas-02".into())), vec!["iuas-02"]);
        // Full right roll, a touch of up-throttle.
        let mut sticks = center_channels();
        sticks[CH_ROLL] = MAX_US;
        sticks[CH_THROTTLE] = 1600;
        host.set_input(sticks, false, 0);
        host.tick(0);

        let frame = recv_frame(&mut rx).expect("a paced frame arrived at the agent's RC socket");
        assert_eq!(frame.channels[CH_ROLL], MAX_US, "roll deflection survived the wire");
        assert_eq!(frame.channels[CH_THROTTLE], 1600);
        assert_eq!(frame.channels[CH_PITCH], CENTER_US);
        assert!(!frame.flags.contains(RcFlags::EMERGENCY_STOP));
    }

    /// E-stop bypasses the pacer: the flagged frame lands WITHOUT any 50 Hz
    /// tick having run.
    #[test]
    fn estop_bypasses_the_pacer() {
        let mut rx = UdpRcReceiver::bind("127.0.0.1:0").unwrap();
        let addr = rx.local_addr().unwrap();
        let host = RcHost::new(&roster(&[("iuas-02", &addr.to_string())]));
        host.engage(RcTarget::One("iuas-02".into()));

        // No tick() — the only thing that can put a frame on the wire is the
        // e-stop's own immediate send.
        host.estop(true);
        let frame = recv_frame(&mut rx).expect("e-stop frame sent immediately");
        assert!(
            frame.flags.contains(RcFlags::EMERGENCY_STOP),
            "the un-paced frame carries the e-stop flag"
        );
    }

    #[test]
    fn broadcast_reaches_every_reachable_lane() {
        let mut a = UdpRcReceiver::bind("127.0.0.1:0").unwrap();
        let mut b = UdpRcReceiver::bind("127.0.0.1:0").unwrap();
        let host = RcHost::new(&roster(&[
            ("iuas-01", &a.local_addr().unwrap().to_string()),
            ("iuas-02", &b.local_addr().unwrap().to_string()),
        ]));
        assert_eq!(host.engage(RcTarget::Broadcast).len(), 2);
        host.tick(0);
        assert!(recv_frame(&mut a).is_some(), "iuas-01 lane received the broadcast");
        assert!(recv_frame(&mut b).is_some(), "iuas-02 lane received the broadcast");
    }

    #[test]
    fn disengage_reports_vehicles_and_stops_sending() {
        let mut rx = UdpRcReceiver::bind("127.0.0.1:0").unwrap();
        let host = RcHost::new(&roster(&[("iuas-02", &rx.local_addr().unwrap().to_string())]));
        host.engage(RcTarget::One("iuas-02".into()));
        assert_eq!(host.disengage(), vec!["iuas-02"], "reports who was engaged");
        assert!(!host.engaged());
        host.tick(0); // no session: nothing synthesized
        // Give any stray datagram a moment, then assert the socket is quiet.
        std::thread::sleep(Duration::from_millis(20));
        assert!(rx.poll().unwrap().is_none(), "a disengaged host sends nothing");
    }
}
