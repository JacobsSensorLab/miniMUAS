//! RC-CONTROL R2 pilot surface — the dashboard's send side, over NAMED DATA.
//!
//! # How a stick sample travels (proof this is NOT a shortcut)
//!
//! The browser's Gamepad/keyboard poll maps sticks to AETR channels and
//! streams them over the dashboard WebSocket (`{"cmd":"rc",…}`). The WS
//! handler feeds the latest sample into this host, whose 50 Hz loop
//! synthesizes a real [`uas_rc::RcFrame`], paces it through the same
//! [`uas_rc::RatePacer`] a console pilot node uses, and PUBLISHES it as the
//! latest-wins Data content of the vehicle's `/muas/v3/<vid>/rc` name over
//! the dashboard's own [`ForwarderEngine`] — the same engine + faces that
//! carry telemetry. The agent fetches that name over ITS engine (across the
//! ndn-sim SimLinks in sim, real faces in the field) and its real
//! `RcReceiverTask` admits the frame: seq-gap ledger, engage arbitration,
//! silence-ladder failsafe, flight backend. Nothing rides a side UDP socket;
//! the NAME is the interface, and the frames cross the same fabric as every
//! other stream (see `docs/v3/RC-CONTROL.md` "Transport correction").
//!
//! # Loss honesty over the pull carriage
//!
//! Publishing is a latest-wins slot update: the agent's ~50 Hz fetch pulls
//! whatever frame is current, so a paced-out (dropped) frame or a Data lost
//! on the SimLink surfaces on the receiver as an honest seq gap, never a
//! repair. The e-stop bypass writes the flagged frame into the slot
//! immediately (no pacer wait); the agent picks it up within its next fetch
//! tick — sub-frame-period, the P11 budget.
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
use std::sync::Arc;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use bytes::Bytes;
use ndn_app::{EngineAppExt, ServeGuard};
use ndn_engine::ForwarderEngine;
use ndn_packet::Name;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};
use uas_rc::{EngineSparkSender, RatePacer, RcFlags, RcFrame, RcTarget, DEFAULT_RC_HZ};

use muas_contracts::names;

use crate::config::RcCarriage;
use crate::lock;

/// Anchor cadence for the engine-Spark carriage: cut a checkpoint Block every
/// this many Sparks (~0.5 s at 50 Hz). Small enough that a checkpoint appears
/// promptly (the `--verify` anchor proof), large enough to keep the
/// one-signature-per-window economy.
const ANCHOR_EVERY: usize = 25;

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
/// same rate, so at steady state one synthesized frame is published per tick).
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

/// Expand a [`RcTarget`] against the RC-reachable vehicle roster into the
/// vehicle ids that actually receive frames (broadcast fans to every vehicle
/// in stable order; a `One` target not on the roster expands to nothing).
/// Mirrors [`RcTarget::stream_names`] selector semantics — the roster is now
/// the set of vehicle ids whose `<vid>/rc` names the surface publishes.
pub fn expand_targets(target: &RcTarget, roster: &[String]) -> Vec<String> {
    match target {
        RcTarget::One(vid) => {
            if roster.iter().any(|v| v == vid) {
                vec![vid.clone()]
            } else {
                Vec::new()
            }
        }
        RcTarget::Broadcast => roster.to_vec(),
    }
}

/// One vehicle's latest-wins frame slot (the Data the serve handler answers
/// with) shared between the 50 Hz loop and the engine serve task.
type FrameSlot = Arc<Mutex<Option<Bytes>>>;

/// Per-vehicle send state: the send-side pacer and the monotonic frame
/// counter. The frame BYTES land in the vehicle's [`FrameSlot`].
struct RcLink {
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

/// The pilot-surface send host. RC targeting is by NAME; the carriage decides
/// HOW the frames ride the fabric (transport correction 2026-07-11):
///
/// - [`RcCarriage::Spark`] (**default**) — a per-vehicle [`EngineSparkSender`]
///   stamps each frame into a Spark, carries it over the engine under
///   `/muas/v3/<vid>/rc/spark/<index>`, and cuts checkpoint Blocks; the agent
///   fetches + SP-3-judges them.
/// - [`RcCarriage::Data`] (comparison bearer) — a latest-wins frame slot per
///   vehicle served under `/muas/v3/<vid>/rc/frame`.
///
/// Either way the send-side pacer (50 Hz) + e-stop bypass are identical; only
/// the byte carriage differs.
pub struct RcHost {
    /// RC-reachable vehicle ids, stable (sorted) order.
    vehicles: Vec<String>,
    /// Which carriage the sticks ride.
    carriage: RcCarriage,
    /// Latest-wins frame bytes per vehicle — the [`RcCarriage::Data`] bearer
    /// (empty in Spark mode); read by the engine serve handlers, written by
    /// the pacing loop / e-stop bypass.
    slots: BTreeMap<String, FrameSlot>,
    /// Per-vehicle engine-Spark senders — the [`RcCarriage::Spark`] carriage
    /// (empty in Data mode). Each owns a SparkEmitter + engine carrier and cuts
    /// checkpoint Blocks; built in [`serve`](Self::serve).
    senders: Mutex<BTreeMap<String, EngineSparkSender>>,
    links: Mutex<BTreeMap<String, RcLink>>,
    input: Mutex<RcInput>,
    /// Engine serve guards for the Data carriage; held for the host's lifetime
    /// so the `<vid>/rc/frame` names keep being answered (the Spark senders
    /// hold their own serve guards).
    serve_guards: Mutex<Vec<ServeGuard>>,
    t0: Instant,
}

impl RcHost {
    /// Build a host over the RC-reachable vehicle ids on the default
    /// engine-Spark carriage. An empty roster yields an inert host.
    pub fn new(vehicles: &[String]) -> Arc<Self> {
        Self::with_carriage(vehicles, RcCarriage::default())
    }

    /// Build a host on an explicit carriage (`--rc-data` selects
    /// [`RcCarriage::Data`]).
    pub fn with_carriage(vehicles: &[String], carriage: RcCarriage) -> Arc<Self> {
        let mut sorted: Vec<String> = vehicles.to_vec();
        sorted.sort();
        sorted.dedup();
        let mut slots = BTreeMap::new();
        let mut links = BTreeMap::new();
        for vid in &sorted {
            slots.insert(vid.clone(), Arc::new(Mutex::new(None)));
            links.insert(vid.clone(), RcLink { pacer: RatePacer::new(DEFAULT_RC_HZ), seq: 0 });
            info!(vid, ?carriage, "rc pilot lane up (named data)");
        }
        Arc::new(Self {
            vehicles: sorted,
            carriage,
            slots,
            senders: Mutex::new(BTreeMap::new()),
            links: Mutex::new(links),
            input: Mutex::new(RcInput::default()),
            serve_guards: Mutex::new(Vec::new()),
            t0: Instant::now(),
        })
    }

    /// Bring each vehicle's RC carriage up on `engine`: in Spark mode a
    /// per-vehicle [`EngineSparkSender`] serving `/muas/v3/<vid>/rc/spark`; in
    /// Data mode a latest-wins slot served under `/muas/v3/<vid>/rc/frame`. The
    /// agent fetches these over the fabric exactly like a telemetry stream.
    pub async fn serve(
        self: &Arc<Self>,
        engine: &ForwarderEngine,
        cancel: &CancellationToken,
    ) -> Result<(), String> {
        match self.carriage {
            RcCarriage::Spark => {
                let mut senders = BTreeMap::new();
                for vid in &self.vehicles {
                    let prefix: Name = names::vehicle_stream(vid, "rc/spark")
                        .parse()
                        .map_err(|e| format!("bad rc spark name for {vid}: {e:?}"))?;
                    let sender =
                        EngineSparkSender::serve(engine, prefix, Some(ANCHOR_EVERY), cancel).await?;
                    senders.insert(vid.clone(), sender);
                }
                *lock(&self.senders) = senders;
                info!(vehicles = self.vehicles.len(), "rc pilot sparks served over the engine");
            }
            RcCarriage::Data => {
                let node = engine.app_node(cancel.child_token());
                let mut guards = Vec::new();
                for vid in &self.vehicles {
                    // `rc/frame` (a sibling of the agent's `rc/status`) so the
                    // frame name is not a prefix of `rc/status` — see the agent
                    // fetch loop.
                    let name: Name = names::vehicle_stream(vid, "rc/frame")
                        .parse()
                        .map_err(|e| format!("bad rc frame name for {vid}: {e:?}"))?;
                    let slot = self.slots[vid].clone();
                    let guard = node
                        .serve(name, move |interest, responder| {
                            let latest = lock(&slot).clone();
                            async move {
                                if let Some(bytes) = latest {
                                    let _ =
                                        responder.respond((*interest.name).clone(), bytes).await;
                                }
                                // No frame published yet: drop the Interest —
                                // the agent sees silence (the failsafe's signal).
                            }
                        })
                        .await
                        .map_err(|e| format!("serve rc frame {vid}: {e}"))?;
                    guards.push(guard);
                }
                *lock(&self.serve_guards) = guards;
                info!(vehicles = self.vehicles.len(), "rc pilot frames served over the engine");
            }
        }
        Ok(())
    }

    /// Total checkpoint Blocks cut across all engine-Spark lanes (0 in Data
    /// mode) — the durable-record count surfaced for observability.
    pub fn checkpoints_cut(&self) -> u64 {
        lock(&self.senders).values().map(EngineSparkSender::checkpoints_cut).sum()
    }

    /// The RC-reachable vehicle ids (drives the Pilot selector + hello).
    pub fn vehicles(&self) -> Vec<String> {
        self.vehicles.clone()
    }

    /// Engage a target. Returns the vehicle ids now under this session
    /// (empty = the target expanded to no reachable vehicle, so nothing
    /// engaged). Sticks reset to center on engage.
    pub fn engage(&self, target: RcTarget) -> Vec<String> {
        let vids = expand_targets(&target, &self.vehicles);
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

    /// Release the session locally: stop publishing frames and drop the
    /// e-stop latch. Clears each engaged vehicle's frame slot so the agent
    /// fetches nothing (silence releases the agent-side session on its own;
    /// the caller also sends the explicit `rc_disengage` op). Returns the
    /// vehicle ids that were engaged.
    pub fn disengage(&self) -> Vec<String> {
        let mut inp = lock(&self.input);
        let vids = inp.session.take().map(|t| expand_targets(&t, &self.vehicles)).unwrap_or_default();
        inp.estop = false;
        inp.channels = center_channels();
        drop(inp);
        for vid in &vids {
            if let Some(slot) = self.slots.get(vid) {
                *lock(slot) = None;
            }
        }
        vids
    }

    /// Set/clear the e-stop latch. Turning it ON publishes a flagged frame
    /// IMMEDIATELY into every engaged lane's slot, bypassing the pacer (P11:
    /// the safing e-stop must reach the vehicle in under one frame period —
    /// the agent's next fetch tick picks it up); subsequent paced frames
    /// keep the flag until it is cleared.
    pub fn estop(&self, on: bool) {
        let target = {
            let mut inp = lock(&self.input);
            inp.estop = on;
            inp.session.clone()
        };
        if on {
            if let Some(target) = target {
                self.publish_immediate(&target);
            }
        }
    }

    /// Whether a session is currently engaged.
    pub fn engaged(&self) -> bool {
        lock(&self.input).session.is_some()
    }

    /// Publish one un-paced frame from the current input onto each engaged
    /// lane's carriage (the e-stop bypass — reaches the vehicle in under one
    /// frame period, whichever carriage rides).
    fn publish_immediate(&self, target: &RcTarget) {
        let (channels, flags) = self.frame_fields();
        let t_ms = self.t_ms();
        let mut links = lock(&self.links);
        for vid in expand_targets(target, &self.vehicles) {
            if let Some(link) = links.get_mut(&vid) {
                let frame = link.stamp(RcFrame { seq: 0, t_ms, channels, flags });
                self.emit_frame(&vid, &frame);
            }
        }
    }

    /// One paced publish pass — the body of the 50 Hz loop, exposed so tests
    /// drive it deterministically. Offers a freshly synthesized frame to
    /// each engaged lane's pacer and publishes whatever the pacer releases.
    pub fn tick(&self, now_ms: u64) {
        let (target, channels, flags) = {
            let inp = lock(&self.input);
            let Some(target) = inp.session.clone() else { return };
            (target, inp.channels, compose_flags(inp.arm, inp.estop, inp.mode))
        };
        let t_ms = now_ms as u32;
        let mut links = lock(&self.links);
        for vid in expand_targets(&target, &self.vehicles) {
            if let Some(link) = links.get_mut(&vid) {
                // RcLink assigns the monotonic RcFrame seq (the agent's frame
                // ledger); the Spark carriage additionally stamps a SparkPayload
                // seq (SP-3) — both advance per sent frame, in lockstep, so a
                // paced-out frame is an honest gap on either ledger.
                let frame = link.stamp(RcFrame { seq: 0, t_ms, channels, flags });
                if let Some(out) = link.pacer.offer(frame, now_ms) {
                    self.emit_frame(&vid, &out);
                }
            }
        }
    }

    /// Carry one frame onto the vehicle's configured carriage: the engine-Spark
    /// sender (default — stamp + carry + checkpoint) or the frame-as-Data slot
    /// (comparison bearer).
    fn emit_frame(&self, vid: &str, frame: &RcFrame) {
        match self.carriage {
            RcCarriage::Spark => {
                let mut senders = lock(&self.senders);
                if let Some(sender) = senders.get_mut(vid) {
                    sender.send_sync(frame, i64::from(frame.t_ms) * 1_000);
                } else {
                    warn!(vid, "rc: no spark sender for engaged vehicle");
                }
            }
            RcCarriage::Data => self.write_slot(vid, frame),
        }
    }

    /// Write one frame's 26-byte wire form into a vehicle's latest-wins slot
    /// (the [`RcCarriage::Data`] comparison bearer).
    fn write_slot(&self, vid: &str, frame: &RcFrame) {
        if let Some(slot) = self.slots.get(vid) {
            *lock(slot) = Some(Bytes::from(frame.encode().to_vec()));
        } else {
            warn!(vid, "rc: no slot for engaged vehicle");
        }
    }

    fn frame_fields(&self) -> ([u16; 8], RcFlags) {
        let inp = lock(&self.input);
        (inp.channels, compose_flags(inp.arm, inp.estop, inp.mode))
    }

    fn t_ms(&self) -> u32 {
        self.t0.elapsed().as_millis() as u32
    }

    /// Run the 50 Hz publish loop until cancelled.
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
    use ndn_app::EngineAppExt;
    use ndn_engine::builder::{EngineBuilder, EngineConfig};
    use ndn_packet::encode::InterestBuilder;
    use uas_rc::{RcFrame, RC_FRAME_VERSION};

    fn roster(vids: &[&str]) -> Vec<String> {
        vids.iter().map(|v| v.to_string()).collect()
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
        let r = roster(&["iuas-01", "iuas-02", "wuas-01"]);
        assert_eq!(expand_targets(&RcTarget::One("iuas-02".into()), &r), vec!["iuas-02"]);
        assert!(
            expand_targets(&RcTarget::One("ghost".into()), &r).is_empty(),
            "an unreachable vehicle expands to nothing"
        );
        // Broadcast fans to every reachable vehicle, in stable order.
        assert_eq!(
            expand_targets(&RcTarget::Broadcast, &r),
            vec!["iuas-01", "iuas-02", "wuas-01"]
        );
        assert!(expand_targets(&RcTarget::Broadcast, &[]).is_empty());
    }

    // ---- default carriage: ndf-spark over the engine ----------------------

    /// The honesty test on the DEFAULT carriage, engine-loopback: a dashboard
    /// [`RcHost`] emits RC Sparks on `<vid>/rc/spark`, and a
    /// [`uas_rc::EngineSparkReceiver`] on the SAME engine (standing in for the
    /// agent) pulls + SP-3-judges them — a real Interest/Data exchange over the
    /// engine, no side socket — with the pilot's AETR channels intact; then a
    /// checkpoint Block cut over a full window anchor-verifies.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn spark_carriage_sticks_and_checkpoints_reach_a_consumer_over_the_engine() {
        let (engine, _shutdown) =
            EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
        let cancel = CancellationToken::new();
        let host = RcHost::new(&roster(&["iuas-02"])); // default = Spark
        host.serve(&engine, &cancel).await.expect("serve rc");

        assert_eq!(host.engage(RcTarget::One("iuas-02".into())), vec!["iuas-02"]);
        let mut sticks = center_channels();
        sticks[CH_ROLL] = MAX_US;
        sticks[CH_THROTTLE] = 1600;
        host.set_input(sticks, false, 0);

        let prefix: Name = names::vehicle_stream("iuas-02", "rc/spark").parse().unwrap();
        let mut rx = uas_rc::EngineSparkReceiver::new(&engine, prefix, &cancel);

        // Pace one Spark per virtual tick; pull each over the engine.
        let mut roll = None;
        for tick in 0..(ANCHOR_EVERY as u64 + 4) {
            host.tick(tick * 20);
            if let Some((frame, _hex)) = rx.poll().await {
                roll = Some(frame.channels[CH_ROLL]);
            }
        }
        assert_eq!(roll, Some(MAX_US), "roll deflection survived the spark-over-engine hop");

        // A full window was cut (ANCHOR_EVERY sticks) and anchor-verifies.
        assert!(host.checkpoints_cut() >= 1, "a checkpoint Block was cut");
        let verdict = rx.poll_checkpoint().await.expect("checkpoint fetched over the engine");
        assert!(
            matches!(verdict, uas_rc::AnchorVerdict::Anchored { .. }) || rx.anchored() >= 1,
            "the checkpoint window anchors (windowed merkle integrity): {verdict:?}"
        );
        cancel.cancel();
    }

    /// E-stop on the Spark carriage bypasses the pacer: the flagged Spark is
    /// carried IMMEDIATELY (fetchable) WITHOUT any 50 Hz tick having run.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn spark_estop_bypasses_the_pacer() {
        let (engine, _shutdown) =
            EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
        let cancel = CancellationToken::new();
        let host = RcHost::new(&roster(&["iuas-02"]));
        host.serve(&engine, &cancel).await.expect("serve rc");
        host.engage(RcTarget::One("iuas-02".into()));

        // No tick() — only the e-stop's own immediate carry can publish.
        host.estop(true);
        let prefix: Name = names::vehicle_stream("iuas-02", "rc/spark").parse().unwrap();
        let mut rx = uas_rc::EngineSparkReceiver::new(&engine, prefix, &cancel);
        let (frame, _hex) = rx.poll().await.expect("e-stop spark published");
        assert!(
            frame.flags.contains(RcFlags::EMERGENCY_STOP),
            "the un-paced Spark carries the e-stop flag"
        );
        cancel.cancel();
    }

    // ---- comparison bearer: frame-as-latest-wins-Data ---------------------

    /// Fetch `/muas/v3/<vid>/rc/frame` (the Data comparison bearer) over an
    /// engine until a frame arrives.
    async fn fetch_rc(consumer: &mut ndn_app::Consumer, vid: &str) -> Option<RcFrame> {
        let name: Name = names::vehicle_stream(vid, "rc/frame").parse().ok()?;
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            let interest = InterestBuilder::new(name.clone())
                .must_be_fresh()
                .lifetime(Duration::from_millis(50));
            if let Ok(data) = consumer.fetch_with(interest).await {
                if let Some(bytes) = data.content() {
                    if let Ok(frame) = RcFrame::decode(bytes) {
                        return Some(frame);
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        None
    }

    /// The Data comparison bearer, engine-loopback: a `--rc-data` host serves
    /// the pilot's frame as latest-wins Data on `<vid>/rc/frame`, fetched over
    /// the SAME engine with the AETR channels intact.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn data_bearer_frames_reach_a_consumer_over_the_engine() {
        let (engine, _shutdown) =
            EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
        let cancel = CancellationToken::new();
        let host = RcHost::with_carriage(&roster(&["iuas-02"]), RcCarriage::Data);
        host.serve(&engine, &cancel).await.expect("serve rc");

        assert_eq!(host.engage(RcTarget::One("iuas-02".into())), vec!["iuas-02"]);
        // Full right roll, a touch of up-throttle.
        let mut sticks = center_channels();
        sticks[CH_ROLL] = MAX_US;
        sticks[CH_THROTTLE] = 1600;
        host.set_input(sticks, false, 0);
        host.tick(0);

        let mut consumer = engine.app_consumer(cancel.child_token());
        let frame = fetch_rc(&mut consumer, "iuas-02")
            .await
            .expect("a published frame was fetched over the engine");
        assert_eq!(frame.channels[CH_ROLL], MAX_US, "roll deflection survived the named-data hop");
        assert_eq!(frame.channels[CH_THROTTLE], 1600);
        assert_eq!(frame.channels[CH_PITCH], CENTER_US);
        assert!(!frame.flags.contains(RcFlags::EMERGENCY_STOP));
        cancel.cancel();
    }

    /// E-stop bypasses the pacer on the Data bearer too: the flagged frame
    /// lands in the slot (fetchable) WITHOUT any 50 Hz tick having run.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn data_bearer_estop_bypasses_the_pacer() {
        let (engine, _shutdown) =
            EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
        let cancel = CancellationToken::new();
        let host = RcHost::with_carriage(&roster(&["iuas-02"]), RcCarriage::Data);
        host.serve(&engine, &cancel).await.expect("serve rc");
        host.engage(RcTarget::One("iuas-02".into()));

        host.estop(true);
        let mut consumer = engine.app_consumer(cancel.child_token());
        let frame = fetch_rc(&mut consumer, "iuas-02").await.expect("e-stop frame published");
        assert!(
            frame.flags.contains(RcFlags::EMERGENCY_STOP),
            "the un-paced frame carries the e-stop flag"
        );
        cancel.cancel();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn data_bearer_broadcast_reaches_every_reachable_lane() {
        let (engine, _shutdown) =
            EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
        let cancel = CancellationToken::new();
        let host = RcHost::with_carriage(&roster(&["iuas-01", "iuas-02"]), RcCarriage::Data);
        host.serve(&engine, &cancel).await.expect("serve rc");
        assert_eq!(host.engage(RcTarget::Broadcast).len(), 2);
        host.tick(0);
        let mut consumer = engine.app_consumer(cancel.child_token());
        assert!(fetch_rc(&mut consumer, "iuas-01").await.is_some(), "iuas-01 got the broadcast");
        assert!(fetch_rc(&mut consumer, "iuas-02").await.is_some(), "iuas-02 got the broadcast");
        cancel.cancel();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn data_bearer_disengage_reports_vehicles_and_clears_the_slot() {
        let (engine, _shutdown) =
            EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
        let cancel = CancellationToken::new();
        let host = RcHost::with_carriage(&roster(&["iuas-02"]), RcCarriage::Data);
        host.serve(&engine, &cancel).await.expect("serve rc");
        host.engage(RcTarget::One("iuas-02".into()));
        host.tick(0);
        assert_eq!(host.disengage(), vec!["iuas-02"], "reports who was engaged");
        assert!(!host.engaged());
        host.tick(20); // no session: nothing synthesized

        // A disengaged host publishes nothing: the slot is cleared and the
        // fetch finds no Data.
        let mut consumer = engine.app_consumer(cancel.child_token());
        let name: Name = names::vehicle_stream("iuas-02", "rc/frame").parse().unwrap();
        let interest = InterestBuilder::new(name)
            .must_be_fresh()
            .lifetime(Duration::from_millis(100));
        assert!(consumer.fetch_with(interest).await.is_err(), "no frame after disengage");
        cancel.cancel();
    }
}
