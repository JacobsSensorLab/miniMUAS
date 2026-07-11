//! The RC-over-NDN receiver task (RC-CONTROL R1): frames from a uas-rc
//! binding stream to the flight backend as channel overrides; the silence
//! ladder's typed intents map onto the agent's flight surface; every
//! engage / refusal / failsafe transition / release is journaled; and the
//! session publishes an `rc/status` latest-wins stream for the dashboard
//! R2 pilot strip.
//!
//! # Engage arbitration (RC vs the task queue)
//!
//! The vehicle is claimed race-free under the queue + busy locks
//! ([`crate::queue::rc_seize`]):
//!
//! - **Idle vehicle** — first admitted frames engage immediately: busy
//!   becomes [`RC_MANUAL`], `rc.engaged` is journaled.
//! - **Active mission task** — engage requires the frame's ARM_GESTURE
//!   flag (the deliberate held gesture, P11 D2) AND the configured
//!   `--rc-preempt`:
//!   - `deny` (default): journaled refusal (`rc.refused`), the mission
//!     keeps flying, frames are discarded;
//!   - `pause-mission`: the active task is suspended through the queue's
//!     split machinery exactly like a reorder displacement — its exact
//!     remainder re-enters the queue at the FRONT (`origin=split`) and
//!     resumes via [`crate::queue::kick`] when the RC session releases.
//! - **Non-queue owners** (`takeoff`, `rtl`, legacy `--no-queue` tasks,
//!   an override detour): nothing to split — refused. Work is never taken
//!   from a return-to-launch (the v2 rule).
//!
//! The ladder commands stay sovereign: rtl/land/hold re-label or clear the
//! busy claim, and the RC task observes the theft within one tick,
//! releases the override, and journals `rc.released{reason:"superseded"}`.
//!
//! # Failsafe intent mapping
//!
//! | intent | action |
//! |---|---|
//! | `Hold` | `backend.hold()` — position hold, session stays engaged. |
//! | `Rtl` | release the override, flush pending queue entries (an RTL is a ladder stop — nothing queued survives it), then the agent's own RTL path: slot-layered smart RTL when a fleet is configured, native RTL otherwise. The session RELEASES (`rc.released{reason:"silence-rtl"}`). |
//! | `EmergencyStop` | release the override + `backend.land()`, session stays engaged so live flag-cleared frames re-engage manual (the operator's release gesture). **Documented choice**: on the sim/kinematic and SITL backends `land()` is the strongest energy-reducing command the backend surface offers; a REAL airframe e-stop means motor cut (disarm), which needs a force-disarm passthrough the backend seam does not expose yet — deferred with a P11 note below. |
//! | `ResumeManual` | journaled; stick frames resume flowing on their own. |
//!
//! # P11 note (e-stop tier)
//!
//! `EmergencyStop` is D3-classed actuation in the SAFING direction: no
//! confirmation, no rate limit, mapped within one poll tick. Mapping it to
//! `land()` (not disarm) is deliberate for R1: the sim and SITL benches
//! have no force-disarm seam, and an uncommanded mid-air disarm is the one
//! energy-REDUCING action that can still destroy the airframe — the real
//! motor-cut passthrough (MAV_CMD_COMPONENT_ARM_DISARM force) lands with
//! the R5 hardware increment, where the e-stop fob work sits.
//!
//! # Arming
//!
//! ARM_GESTURE is an engage-arbitration token here, never an arm command:
//! R1 does not arm the vehicle (P11: the flag is a request; the vehicle's
//! own arm ladder — dashboard takeoff, prearm checks — stays the only path
//! to spinning motors). On the sim backend overrides move the kinematic
//! model regardless, which is exactly what the SITL-less tests observe.

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};
use uas_rc::{
    FailsafeConfig, RcAdmission, RcEvent, RcFlags, RcIntent, RcReceiverTask, RcSource,
    SparkRcReceiver, UdpRcReceiver,
};

use crate::config::{RcConfig, RcPreempt, RcTransport};
use crate::{lock, queue, AgentCommand, AgentShared, BackendExt};

/// The busy label an engaged RC session holds (the RC analogue of a task
/// kind; `task_abort("rc-manual")` therefore also works as a disengage).
pub(crate) const RC_MANUAL: &str = "rc-manual";

/// Receiver poll cadence (fast enough for 50 Hz frame streams and the
/// sub-frame-period e-stop budget).
const RC_TICK: Duration = Duration::from_millis(20);

/// rc/status republish floor (transitions publish immediately).
const STATUS_PERIOD: Duration = Duration::from_millis(250);

/// Build the configured binding and run the receiver loop until cancel.
pub(crate) async fn run_rc_task(
    shared: Arc<AgentShared>,
    config: RcConfig,
    cancel: CancellationToken,
) {
    let source = match &config.transport {
        RcTransport::Listen(addr) => match UdpRcReceiver::bind(addr) {
            Ok(rx) => RcSource::Udp(rx),
            Err(err) => {
                warn!(%err, %addr, "rc: udp bind failed; rc receiver disabled");
                shared.journal.event(
                    "rc.error",
                    serde_json::json!({ "bind": addr.to_string(), "error": err.to_string() }),
                );
                return;
            }
        },
        RcTransport::Spark(addr) => match SparkRcReceiver::bind(addr) {
            Ok(rx) => RcSource::Spark(rx),
            Err(err) => {
                warn!(%err, %addr, "rc: spark bind failed; rc receiver disabled");
                shared.journal.event(
                    "rc.error",
                    serde_json::json!({ "bind": addr.to_string(), "error": err.to_string() }),
                );
                return;
            }
        },
    };
    let task = RcReceiverTask::new(
        source,
        FailsafeConfig {
            hold_after_ms: config.hold_after_ms,
            rtl_after_ms: config.rtl_after_ms,
        },
        RcAdmission {
            allowed_instances: config.admission.clone(),
            allowed_senders: Vec::new(),
        },
    );
    let label = config.transport.label();
    info!(source = %label, preempt = config.preempt.as_str(), "rc receiver up");
    run_rc_loop(shared, config, task, label, cancel).await;
}

/// One engaged-session ledger (loop-local state).
struct Session {
    engaged: bool,
    /// Last journaled refusal reason — refusals journal on CHANGE, not per
    /// frame (a denied 50 Hz stream is one line, not a flood).
    refusal: Option<&'static str>,
    /// Set on every release: the CONTINUING stream may not instantly
    /// re-seize the vehicle (an operator disengage or a ladder theft must
    /// stick). Cleared once the stream pauses past the hold threshold —
    /// re-engaging then takes a fresh stream, a deliberate act.
    lockout: bool,
}

/// The receiver loop, split from the binding so tests can inject a task
/// around an OS-assigned port (or a scripted source).
pub(crate) async fn run_rc_loop(
    shared: Arc<AgentShared>,
    config: RcConfig,
    mut task: RcReceiverTask,
    source_label: String,
    cancel: CancellationToken,
) {
    let t0 = std::time::Instant::now();
    let mut interval = tokio::time::interval(RC_TICK);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut session = Session { engaged: false, refusal: None, lockout: false };
    let mut events: Vec<RcEvent> = Vec::new();
    let mut last_status: Option<std::time::Instant> = None;

    loop {
        tokio::select! {
            _ = cancel.cancelled() => break,
            _ = interval.tick() => {}
        }
        let now_ms = t0.elapsed().as_millis() as u64;

        // A post-release lockout lifts once the stream has actually paused
        // (measured BEFORE this tick's frames fold in, so a fresh stream
        // arriving right now can engage in the same tick).
        if session.lockout
            && task
                .status(now_ms)
                .silence_ms
                .is_none_or(|silence| silence > config.hold_after_ms)
        {
            session.lockout = false;
        }

        // Explicit operator disengage (`rc_disengage` service op).
        if shared.rc_disengage.swap(false, Ordering::Relaxed) && session.engaged {
            release(&shared, &mut session, "operator");
            queue::kick(&shared); // a paused mission resumes
        }
        // Superseded: a ladder command (rtl/land/hold) re-labelled or
        // cleared the busy claim — the interrupting command owns the
        // vehicle; give channel authority back without touching the queue.
        if session.engaged && *lock(&shared.busy) != RC_MANUAL {
            release(&shared, &mut session, "superseded");
        }

        events.clear();
        if let Err(err) = task.poll(now_ms, &mut events) {
            warn!(%err, "rc: receiver poll failed; rc receiver stopping");
            shared
                .journal
                .event("rc.error", serde_json::json!({ "error": err.to_string() }));
            break;
        }
        let transition = events
            .iter()
            .any(|e| matches!(e, RcEvent::Intent(_)));
        for event in events.drain(..) {
            match event {
                RcEvent::Frame(frame) => {
                    if !session.engaged {
                        try_engage(&shared, &config, &mut session, frame.flags, &source_label, &task);
                    }
                    if session.engaged {
                        lock(&shared.backend).rc_override(frame.channels);
                    }
                }
                RcEvent::Intent(intent) => {
                    if session.engaged {
                        apply_intent(&shared, &mut session, intent);
                    }
                }
            }
        }

        let status_due = last_status.is_none_or(|t| t.elapsed() >= STATUS_PERIOD);
        if transition || status_due {
            publish_status(&shared, &task, now_ms, session.engaged, &source_label);
            last_status = Some(std::time::Instant::now());
        }
    }
    if session.engaged {
        release(&shared, &mut session, "shutdown");
    }
}

/// First admitted frames while unengaged: arbitrate for the vehicle.
fn try_engage(
    shared: &Arc<AgentShared>,
    config: &RcConfig,
    session: &mut Session,
    flags: RcFlags,
    source_label: &str,
    task: &RcReceiverTask,
) {
    if session.lockout {
        // A released session's stream keeps flowing: it may not re-seize
        // until it pauses (see `Session::lockout`).
        if session.refusal != Some("released-lockout") {
            session.refusal = Some("released-lockout");
            shared.journal.event(
                "rc.refused",
                serde_json::json!({ "reason": "released-lockout" }),
            );
        }
        return;
    }
    let arm_gesture = flags.contains(RcFlags::ARM_GESTURE);
    let pause_allowed = config.preempt == RcPreempt::PauseMission;
    match queue::rc_seize(shared, arm_gesture, pause_allowed) {
        queue::RcSeize::Engaged => {
            engage(shared, session, source_label, task, None);
        }
        queue::RcSeize::EngagedPaused { paused_task_id } => {
            engage(shared, session, source_label, task, Some(paused_task_id));
        }
        queue::RcSeize::Refused { busy, reason } => {
            if session.refusal != Some(reason) {
                session.refusal = Some(reason);
                info!(reason, busy, "rc engage refused");
                shared.journal.event(
                    "rc.refused",
                    serde_json::json!({
                        "reason": reason,
                        "busy": busy,
                        "preempt": config.preempt.as_str(),
                        "arm_gesture": arm_gesture,
                    }),
                );
            }
        }
    }
}

fn engage(
    shared: &Arc<AgentShared>,
    session: &mut Session,
    source_label: &str,
    task: &RcReceiverTask,
    paused_task_id: Option<String>,
) {
    session.engaged = true;
    session.refusal = None;
    shared.rc_engaged.store(true, Ordering::Relaxed);
    let instance = task.status(0).instance;
    info!(source = source_label, ?instance, ?paused_task_id, "rc engaged");
    shared.journal.event(
        "rc.engaged",
        serde_json::json!({
            "source": source_label,
            "instance": instance,
            "paused_task_id": paused_task_id,
        }),
    );
    // Engaging IS the manual transition (the ladder's ResumeManual fires
    // one event ahead of the engaging frame, while nothing was subsumed
    // yet) — journal it here so `rc.failsafe` tells the whole story.
    shared.journal.event(
        "rc.failsafe",
        serde_json::json!({ "state": "manual", "intent": RcIntent::ResumeManual.as_str() }),
    );
}

/// Release channel authority and the busy claim. The CALLER decides what
/// happens to the queue (kick on operator release, flush on silence-RTL,
/// nothing when superseded — the interrupting command owns the vehicle).
fn release(shared: &Arc<AgentShared>, session: &mut Session, reason: &str) {
    lock(&shared.backend).rc_release();
    {
        let mut busy = lock(&shared.busy);
        if *busy == RC_MANUAL {
            busy.clear();
        }
    }
    session.engaged = false;
    session.lockout = true; // the continuing stream may not re-seize
    shared.rc_engaged.store(false, Ordering::Relaxed);
    info!(reason, "rc released");
    shared
        .journal
        .event("rc.released", serde_json::json!({ "reason": reason }));
}

/// Map one failsafe transition onto the flight surface (module docs table).
fn apply_intent(shared: &Arc<AgentShared>, session: &mut Session, intent: RcIntent) {
    let state = match intent {
        RcIntent::ResumeManual => "manual",
        RcIntent::Hold => "hold",
        RcIntent::Rtl => "rtl",
        RcIntent::EmergencyStop => "emergency-stop",
    };
    shared.journal.event(
        "rc.failsafe",
        serde_json::json!({ "state": state, "intent": intent.as_str() }),
    );
    match intent {
        RcIntent::ResumeManual => {
            // Sticks resume via the frame events; nothing to command.
            info!("rc failsafe: manual resumed");
        }
        RcIntent::Hold => {
            lock(&shared.backend).as_dyn().hold();
        }
        RcIntent::Rtl => {
            // Silence past the RTL rung: the session is over. Release the
            // override, then the agent's own RTL path — and like every
            // ladder RTL, nothing queued survives it (a paused mission is
            // NOT resumed under a vehicle that lost its pilot link).
            release(shared, session, "silence-rtl");
            queue::flush_pending(shared, "rc-failsafe");
            if shared.smart_rtl {
                *lock(&shared.busy) = "rtl".to_string();
                let _ = shared.commands.send(AgentCommand::SmartRtl);
            } else {
                lock(&shared.backend).as_dyn().rtl();
            }
        }
        RcIntent::EmergencyStop => {
            // Safing NOW: channel authority back, vehicle down. Session
            // stays engaged — live flag-cleared frames are the operator's
            // release gesture (ResumeManual). See the module P11 note for
            // why this is land(), not disarm, in R1.
            let mut backend = lock(&shared.backend);
            backend.rc_release();
            backend.as_dyn().land();
        }
    }
}

/// Publish one rc/status sample into the latest-wins buffer.
fn publish_status(
    shared: &AgentShared,
    task: &RcReceiverTask,
    now_ms: u64,
    engaged: bool,
    source_label: &str,
) {
    let snapshot = task.status(now_ms);
    let status = muas_contracts::rc::RcStatus {
        vehicle_id: shared.vehicle_id.clone(),
        gps_time_ns: crate::telemetry::gps_time_ns(),
        engaged,
        source: source_label.to_string(),
        seq_gap_pct: snapshot.seq_gap_pct,
        age_ms: snapshot.silence_ms,
        failsafe_state: snapshot.state.as_str().to_string(),
    };
    match serde_json::to_vec(&status) {
        Ok(bytes) => *lock(&shared.latest_rc) = Some(Bytes::from(bytes)),
        Err(err) => warn!(%err, "rc status failed to encode"),
    }
}

// ---------------------------------------------------------------------------
// tests — bench agent over the sim backend, REAL time (UDP frame streams +
// wall-clock silence ladders; thresholds shortened via RcConfig)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use muas_contracts::services::{InvestigateRequest, VehicleService};
    use muas_contracts::tasks::{task_origin, task_state, TaskQueueStatus};
    use std::sync::Mutex;
    use uas_fleet_node::flight_backend::{SimFlightBackend, SIM_TICK_S};
    use uas_rc::{RcFrame, UdpRcSender};

    const ORIGIN: (f64, f64) = (35.0, -90.0);
    /// Centered sticks (AETR at 1500 µs), channels 5-8 untouched (65535).
    const CENTERED: [u16; 8] = [1500, 1500, 1500, 1500, 65535, 65535, 65535, 65535];

    fn bench(
        vehicle_id: &str,
        log_dir: Option<std::path::PathBuf>,
    ) -> (Arc<AgentShared>, crate::SharedBackend) {
        let (journal, _task) = crate::journal::spawn(vehicle_id, log_dir, None, None);
        let (cmd_tx, _cmd_rx) = tokio::sync::mpsc::unbounded_channel();
        let sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        let backend: crate::SharedBackend =
            Arc::new(Mutex::new(Box::new(sim) as Box<dyn crate::TickableBackend>));
        let shared = Arc::new(AgentShared::bench(vehicle_id, backend.clone(), journal, cmd_tx));
        {
            // Real-time sim motion tick (these tests run on the wall clock).
            let backend = backend.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_secs_f64(SIM_TICK_S));
                loop {
                    interval.tick().await;
                    lock(&backend).advance(SIM_TICK_S);
                }
            });
        }
        (shared, backend)
    }

    /// Short-ladder RC config around a test-owned UDP receiver.
    fn rc_config(preempt: RcPreempt) -> RcConfig {
        RcConfig {
            preempt,
            hold_after_ms: 200,
            rtl_after_ms: 600,
            ..RcConfig::new(RcTransport::Listen("127.0.0.1:0".parse().unwrap()))
        }
    }

    /// Spawn the receiver loop on an OS-assigned port; returns the frame
    /// destination address and the loop's cancel token.
    fn start_rc(
        shared: &Arc<AgentShared>,
        config: RcConfig,
    ) -> (std::net::SocketAddr, CancellationToken) {
        let rx = UdpRcReceiver::bind("127.0.0.1:0").expect("bind rc receiver");
        let addr = rx.local_addr().expect("local addr");
        let task = RcReceiverTask::new(
            RcSource::Udp(rx),
            FailsafeConfig {
                hold_after_ms: config.hold_after_ms,
                rtl_after_ms: config.rtl_after_ms,
            },
            RcAdmission {
                allowed_instances: config.admission.clone(),
                allowed_senders: Vec::new(),
            },
        );
        let cancel = CancellationToken::new();
        tokio::spawn(run_rc_loop(
            shared.clone(),
            config,
            task,
            format!("listen:{addr}"),
            cancel.clone(),
        ));
        (addr, cancel)
    }

    /// Stream frames at ~50 Hz until the token cancels.
    fn stream_frames(
        dest: std::net::SocketAddr,
        channels: [u16; 8],
        flags: RcFlags,
        seq_from: u32,
    ) -> CancellationToken {
        let stop = CancellationToken::new();
        let token = stop.clone();
        tokio::spawn(async move {
            let tx = UdpRcSender::connect(dest).expect("connect rc sender");
            let mut seq = seq_from;
            let mut interval = tokio::time::interval(Duration::from_millis(20));
            loop {
                tokio::select! {
                    _ = token.cancelled() => break,
                    _ = interval.tick() => {}
                }
                let _ = tx.send(&RcFrame { seq, t_ms: seq * 20, channels, flags });
                seq += 1;
            }
        });
        stop
    }

    async fn wait_until(budget_s: f64, mut predicate: impl FnMut() -> bool) -> bool {
        let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(budget_s);
        loop {
            if predicate() {
                return true;
            }
            if tokio::time::Instant::now() > deadline {
                return false;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
    }

    fn journal_lines(dir: &std::path::Path) -> Vec<serde_json::Value> {
        let mut lines = Vec::new();
        for entry in std::fs::read_dir(dir).into_iter().flatten().flatten() {
            let text = std::fs::read_to_string(entry.path()).unwrap_or_default();
            lines.extend(text.lines().filter_map(|l| serde_json::from_str(l).ok()));
        }
        lines
    }

    fn temp_log_dir(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "muas-rc-test-{tag}-{}-{}",
            std::process::id(),
            crate::telemetry::gps_time_ns()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    fn kinds(lines: &[serde_json::Value], kind: &str) -> Vec<serde_json::Value> {
        lines.iter().filter(|l| l["kind"] == kind).cloned().collect()
    }

    fn service(shared: &Arc<AgentShared>) -> crate::service_impl::VehicleServiceImpl {
        crate::service_impl::VehicleServiceImpl::new(shared.clone())
    }

    fn investigate_req(north_m: f64, turns: f64) -> InvestigateRequest {
        InvestigateRequest {
            lat_deg: ORIGIN.0 + north_m / uas_flight::geo::EARTH_M_PER_DEG_LAT,
            lon_deg: ORIGIN.1,
            agl_m: 8.0,
            radius_m: 6.0,
            turns,
            sensors: vec!["camera".into()],
            ..InvestigateRequest::default()
        }
    }

    fn queue_status(shared: &AgentShared) -> Option<TaskQueueStatus> {
        lock(&shared.latest_tasks)
            .as_ref()
            .and_then(|bytes| serde_json::from_slice(bytes).ok())
    }

    fn rc_status(shared: &AgentShared) -> Option<muas_contracts::rc::RcStatus> {
        lock(&shared.latest_rc)
            .as_ref()
            .and_then(|bytes| serde_json::from_slice(bytes).ok())
    }

    /// Engage on an idle vehicle, observe kinematic motion from stick
    /// deflection, then silence walks the ladder to a released RTL — the
    /// full engage → fly → lose-link → release arc, with the journal and
    /// rc/status stream asserted along the way.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn engage_fly_east_silence_ladder_releases_to_rtl() {
        let dir = temp_log_dir("ladder");
        let (shared, backend) = bench("iuas-60", Some(dir.clone()));
        lock(&backend).as_dyn().ensure_airborne(8.0);
        let (dest, rc_cancel) = start_rc(&shared, rc_config(RcPreempt::Deny));

        // Full right roll: ch1 = 2000 → the sim flies EAST.
        let mut sticks = CENTERED;
        sticks[0] = 2000;
        let sender = stream_frames(dest, sticks, RcFlags::EMPTY, 0);

        assert!(
            wait_until(5.0, || *lock(&shared.busy) == RC_MANUAL
                && shared.rc_engaged.load(Ordering::Relaxed))
            .await,
            "rc never engaged the idle vehicle"
        );
        let lon0 = ORIGIN.1;
        assert!(
            wait_until(10.0, || {
                lock(&backend)
                    .as_dyn_ref()
                    .position()
                    .is_some_and(|(_, lon, _)| (lon - lon0)
                        * uas_flight::geo::m_per_deg_lon(ORIGIN.0)
                        > 1.0)
            })
            .await,
            "stick deflection produced no eastward motion"
        );
        assert!(
            wait_until(2.0, || rc_status(&shared)
                .is_some_and(|s| s.engaged && s.failsafe_state == "manual"))
            .await,
            "rc/status never showed an engaged manual session: {:?}",
            rc_status(&shared)
        );

        // Kill the link: hold at 200 ms, RTL + release at 600 ms.
        sender.cancel();
        assert!(
            wait_until(10.0, || {
                !shared.rc_engaged.load(Ordering::Relaxed)
                    && lock(&shared.busy).is_empty()
                    && lock(&backend).as_dyn_ref().telemetry().mode == "RTL"
            })
            .await,
            "silence never released to RTL (mode: {})",
            lock(&backend).as_dyn_ref().telemetry().mode
        );
        assert!(
            wait_until(2.0, || rc_status(&shared)
                .is_some_and(|s| !s.engaged && s.failsafe_state == "rtl"))
            .await,
            "rc/status never reported the released rtl state"
        );

        rc_cancel.cancel();
        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert_eq!(kinds(&lines, "rc.engaged").len(), 1);
        let failsafe: Vec<String> = kinds(&lines, "rc.failsafe")
            .iter()
            .map(|l| l["intent"].as_str().unwrap_or_default().to_string())
            .collect();
        assert_eq!(
            failsafe,
            vec!["resume-manual", "hold", "rtl"],
            "ladder journals every transition in order"
        );
        let released = kinds(&lines, "rc.released");
        assert_eq!(released.len(), 1);
        assert_eq!(released[0]["reason"], "silence-rtl");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// RC vs an active mission: no ARM_GESTURE refuses; ARM_GESTURE under
    /// the default deny policy still refuses — both journaled once, the
    /// mission keeps the vehicle.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn engage_is_denied_against_an_active_mission() {
        let dir = temp_log_dir("deny");
        let (shared, _backend) = bench("iuas-61", Some(dir.clone()));
        let svc = service(&shared);
        assert!(svc.investigate(investigate_req(40.0, 3.0)).await.accepted);
        let (dest, rc_cancel) = start_rc(&shared, rc_config(RcPreempt::Deny));

        // No arm gesture: refused for the deliberate-gesture rule.
        let sender = stream_frames(dest, CENTERED, RcFlags::EMPTY, 0);
        assert!(
            wait_until(5.0, || {
                journal_lines(&dir).iter().any(|l| {
                    l["kind"] == "rc.refused" && l["reason"] == "arm-gesture-required"
                })
            })
            .await,
            "no journaled arm-gesture refusal"
        );
        sender.cancel();
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Arm gesture held, but preempt policy = deny.
        let sender = stream_frames(dest, CENTERED, RcFlags::ARM_GESTURE, 1000);
        assert!(
            wait_until(5.0, || {
                journal_lines(&dir)
                    .iter()
                    .any(|l| l["kind"] == "rc.refused" && l["reason"] == "preempt-denied")
            })
            .await,
            "no journaled preempt-denied refusal"
        );
        sender.cancel();

        assert_eq!(*lock(&shared.busy), "investigate", "mission kept the vehicle");
        assert!(!shared.rc_engaged.load(Ordering::Relaxed));
        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert!(kinds(&lines, "rc.engaged").is_empty());
        // Refusals journal per reason change, not per 50 Hz frame.
        assert!(kinds(&lines, "rc.refused").len() <= 3, "refusal flood");
        rc_cancel.cancel();
        shared.abort.store(true, Ordering::Relaxed); // wind the bench down
    }

    /// pause-mission: ARM_GESTURE frames suspend the active task through
    /// the split machinery (remainder pending at the front), RC flies, and
    /// the explicit `rc_disengage` op hands the vehicle back to the queue.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn pause_mission_splits_the_active_task_and_resumes_on_disengage() {
        let dir = temp_log_dir("pause");
        let (shared, _backend) = bench("iuas-62", Some(dir.clone()));
        let svc = service(&shared);
        assert!(svc.investigate(investigate_req(40.0, 5.0)).await.accepted);
        let (dest, rc_cancel) = start_rc(&shared, rc_config(RcPreempt::PauseMission));

        let sender = stream_frames(dest, CENTERED, RcFlags::ARM_GESTURE, 0);
        assert!(
            wait_until(10.0, || *lock(&shared.busy) == RC_MANUAL
                && shared.rc_engaged.load(Ordering::Relaxed))
            .await,
            "rc never seized the vehicle under pause-mission"
        );
        // The suspended task became a front-of-queue split continuation.
        assert!(
            wait_until(10.0, || {
                queue_status(&shared).is_some_and(|status| {
                    let done = status
                        .tasks
                        .iter()
                        .any(|t| t.task_id == "tsk-1" && t.state == task_state::DONE);
                    let cont = status.tasks.iter().any(|t| {
                        t.state == task_state::PENDING
                            && t.origin == task_origin::SPLIT
                            && t.parent.as_deref() == Some("tsk-1")
                    });
                    done && cont
                })
            })
            .await,
            "no split continuation appeared: {:?}",
            queue_status(&shared)
        );

        // While RC flies, the disengage op is the way back. The pilot's
        // stream keeps flowing for a beat after the ack — the post-release
        // lockout keeps it from instantly re-seizing the vehicle.
        let ack = svc.rc_disengage().await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        sender.cancel();
        assert!(
            wait_until(10.0, || *lock(&shared.busy) == "investigate"
                && !shared.rc_engaged.load(Ordering::Relaxed))
            .await,
            "queue never resumed the paused mission (busy: {})",
            lock(&shared.busy)
        );
        rc_cancel.cancel();

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        let engaged = kinds(&lines, "rc.engaged");
        assert_eq!(engaged.len(), 1);
        assert_eq!(engaged[0]["paused_task_id"], "tsk-1");
        let released = kinds(&lines, "rc.released");
        assert_eq!(released.len(), 1);
        assert_eq!(released[0]["reason"], "operator");
        assert!(
            lines.iter().any(|l| l["kind"] == "task.split" && l["task_id"] == "tsk-1"),
            "queue journaled the split"
        );
        shared.abort.store(true, Ordering::Relaxed); // wind the bench down
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// E-stop: the flag maps to release + land within the stream, the
    /// session stays engaged, and clearing the flag resumes manual sticks.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn estop_lands_and_flag_clear_resumes_manual() {
        let dir = temp_log_dir("estop");
        let (shared, backend) = bench("iuas-63", Some(dir.clone()));
        lock(&backend).as_dyn().ensure_airborne(8.0);
        let (dest, rc_cancel) = start_rc(&shared, rc_config(RcPreempt::Deny));

        let sender = stream_frames(dest, CENTERED, RcFlags::EMPTY, 0);
        assert!(wait_until(5.0, || shared.rc_engaged.load(Ordering::Relaxed)).await);
        sender.cancel();

        // Operator e-stop: same stream, flag raised.
        let sender = stream_frames(dest, CENTERED, RcFlags::EMERGENCY_STOP, 1000);
        assert!(
            wait_until(5.0, || lock(&backend).as_dyn_ref().telemetry().mode == "LAND").await,
            "e-stop never commanded LAND (mode: {})",
            lock(&backend).as_dyn_ref().telemetry().mode
        );
        assert!(
            shared.rc_engaged.load(Ordering::Relaxed),
            "e-stop keeps the session engaged (flag-clear is the release gesture)"
        );
        sender.cancel();

        // Clearing the flag on live frames resumes manual control.
        let sender = stream_frames(dest, CENTERED, RcFlags::EMPTY, 2000);
        assert!(
            wait_until(5.0, || lock(&backend).as_dyn_ref().telemetry().mode == "GUIDED").await,
            "flag-clear frames never resumed manual overrides"
        );
        sender.cancel();
        rc_cancel.cancel();

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        let intents: Vec<String> = kinds(&lines, "rc.failsafe")
            .iter()
            .map(|l| l["intent"].as_str().unwrap_or_default().to_string())
            .collect();
        assert!(
            intents.contains(&"emergency-stop".to_string()),
            "e-stop transition journaled: {intents:?}"
        );
        assert!(
            intents.iter().filter(|i| *i == "emergency-stop").count() == 1,
            "e-stop journals once (transitions only): {intents:?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The rc_disengage service op refuses when nothing is engaged.
    #[tokio::test]
    async fn rc_disengage_refuses_when_not_engaged() {
        let (shared, _backend) = bench("iuas-64", None);
        let ack = service(&shared).rc_disengage().await;
        assert!(!ack.accepted);
        assert_eq!(ack.code, "rc-not-engaged");
    }
}
