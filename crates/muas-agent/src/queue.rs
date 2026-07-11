//! The per-vehicle task queue engine (v3 queue increment).
//!
//! Replaces the single busy-slot dispatch for the long-running mission tasks
//! (`raster-search`, `investigate`, `sensor-override`) with an ordered queue
//! owned by the vehicle:
//!
//! - **One active task at a time** — flight is exclusive; the dashboard's
//!   busy string stays the ACTIVE task kind, unchanged from v2 (the
//!   busy→idle completion signal fires when the whole queue drains).
//! - **Accept-and-queue** — requests that used to refuse `busy` now ack
//!   `accepted=true, code="queued"` with the task id + position +
//!   ETA-to-start in `detail`, up to [`DEFAULT_QUEUE_DEPTH`] pending entries
//!   (`queue-full` beyond it).
//! - **Split/resume** — a reorder that displaces the active task suspends it
//!   through the runners' existing abort machinery, snapshots the remaining
//!   work ([`ResumeSnapshot`], saved by the runner at the interruption
//!   point, so raster splits are capture-exact), and enqueues a
//!   continuation (`origin=split`, `parent=<id>`) at the displaced
//!   position.
//! - **Stream** — the queue publishes as latest-wins JSON
//!   ([`muas_contracts::tasks::TaskQueueStatus`]) under
//!   [`muas_contracts::names::TASK_QUEUE_STREAM`], on every mutation and at
//!   most ~1 Hz for progress-only changes.
//!
//! Interaction with the v2 command surface (all preserved):
//! - the abort **ladder** (rtl/land/hold) aborts the active task within one
//!   control cycle AND flushes every pending entry — the interrupting
//!   command owns the vehicle;
//! - `task_abort(<label>)` scoped-cancels the ACTIVE task; the queue then
//!   continues with the next entry (idle policy only once empty);
//! - `task_abort("tsk-<n>")` removes ONE pending entry without touching the
//!   flight (an active id is treated like its label);
//! - `takeoff` / watchpoints stay outside the queue; a takeoff that
//!   finishes [`kick`]s the queue so entries queued behind it run.
//!
//! POLICY HOOK (ROUND-3 §2): the pending-depth limit and the
//! accept-vs-refuse rules below are strategy-record material — they are
//! plain named constants/branches here on purpose, so the strategy-record
//! increment can lift them without re-plumbing.

use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;
use muas_contracts::services::{InvestigateRequest, RasterRequest, SensorRequest};
use muas_contracts::tasks::{
    task_kind, task_origin, task_state, QueuedTaskInfo, TaskProgress, TaskQueueStatus,
};
use tracing::{info, warn};

use crate::mission::FlightOutcome;
use crate::{lock, AgentShared, BackendExt};

/// Default pending-depth limit (POLICY HOOK, ROUND-3 §2: becomes
/// strategy-record-driven; a named constant until then).
pub const DEFAULT_QUEUE_DEPTH: usize = 4;

/// How many finished entries stay visible on the stream.
const FINISHED_TAIL: usize = 8;

/// Progress-only republish floor (mutations always publish immediately).
const PROGRESS_PUBLISH_PERIOD: Duration = Duration::from_secs(1);

// ---------------------------------------------------------------------------
// task parameters & resume snapshots
// ---------------------------------------------------------------------------

/// What a queue entry runs. Raster entries carry their remainder coordinates
/// (`start_leg` / `skip_captures` into the deterministically re-planned
/// [`crate::mission::RasterPlan`]) so a split continuation is just another
/// raster entry.
#[derive(Debug, Clone)]
pub enum TaskParams {
    Raster {
        req: RasterRequest,
        /// First plan leg still to fly (0 = the whole raster).
        start_leg: usize,
        /// Capture points already fired on `start_leg` (skipped on resume —
        /// this is what makes split captures exact, no duplicates).
        skip_captures: usize,
    },
    Investigate {
        req: InvestigateRequest,
    },
    SensorOverride {
        req: SensorRequest,
    },
}

impl TaskParams {
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Raster { .. } => task_kind::RASTER_SEARCH,
            Self::Investigate { .. } => task_kind::INVESTIGATE,
            Self::SensorOverride { .. } => task_kind::SENSOR_OVERRIDE,
        }
    }

    /// Human digest + stable 8-hex content hash for the stream.
    fn digest(&self) -> String {
        use std::hash::{Hash, Hasher};
        let (summary, canonical) = match self {
            Self::Raster { req, start_leg, skip_captures } => {
                let resumed = if *start_leg > 0 || *skip_captures > 0 {
                    format!(" (resume leg {start_leg}+{skip_captures})")
                } else {
                    String::new()
                };
                (
                    format!(
                        "area {} corners, {:.0} m spacing @ {:.0} m agl{resumed}",
                        req.corners.len(),
                        req.spacing_m,
                        req.agl_m
                    ),
                    serde_json::to_string(req).unwrap_or_default()
                        + &format!("/{start_leg}/{skip_captures}"),
                )
            }
            Self::Investigate { req } => (
                format!(
                    "({:.5}, {:.5}) r={:.0} m, {:.2} turns @ {:.0} m agl",
                    req.lat_deg, req.lon_deg, req.radius_m, req.turns, req.agl_m
                ),
                serde_json::to_string(req).unwrap_or_default(),
            ),
            Self::SensorOverride { req } => (
                format!("{} @ ({:.5}, {:.5})", req.sensor, req.lat_deg, req.lon_deg),
                serde_json::to_string(req).unwrap_or_default(),
            ),
        };
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        canonical.hash(&mut hasher);
        format!("{summary} [{:08x}]", hasher.finish() as u32)
    }

    /// Rough still-air duration estimate, seconds (queue ETA arithmetic —
    /// deliberately coarse; live progress replaces it once the task flies).
    fn estimate_s(&self, shared: &AgentShared) -> f64 {
        let here = lock(&shared.backend)
            .as_dyn_ref()
            .position()
            .map(|(lat, lon, _)| (lat, lon));
        let transit = |to: (f64, f64), speed: f64| {
            here.map_or(0.0, |h| muas_contracts::policy::dist_m(h, to) / speed)
        };
        match self {
            Self::Raster { req, start_leg, skip_captures } => {
                match crate::mission::plan_raster(req)
                    .ok()
                    .and_then(|p| p.remainder(*start_leg, *skip_captures))
                {
                    Some(plan) => {
                        transit(plan.legs[0][0], plan.speed_m_s)
                            + plan.path_len_m / plan.speed_m_s
                            + 20.0
                    }
                    None => 0.0,
                }
            }
            Self::Investigate { req } => {
                let speed = 3.0; // mission::INVESTIGATE_SPEED_M_S
                let turns = if req.turns > 0.0 { req.turns } else { 1.0 };
                transit((req.lat_deg, req.lon_deg), speed)
                    + turns * std::f64::consts::TAU * req.radius_m.max(2.0) / speed
                    + 15.0
            }
            Self::SensorOverride { req } => {
                transit((req.lat_deg, req.lon_deg), crate::sensor::OVERRIDE_SPEED_M_S) + 10.0
            }
        }
    }
}

/// Remaining-work snapshot, written by the runner (exactly at the
/// interruption point, plus refreshed with every progress report).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResumeSnapshot {
    /// Raster: first leg still to fly (index into the CURRENT plan) and
    /// captures already fired on it.
    Raster { leg: usize, fired_in_leg: usize },
    /// Investigate: turns (orbit) / passes (flyover) still to fly.
    Investigate { remaining_turns: f64 },
}

// ---------------------------------------------------------------------------
// queue state
// ---------------------------------------------------------------------------

/// One queue entry.
#[derive(Debug, Clone)]
pub struct TaskEntry {
    pub task_id: String,
    pub kind: &'static str,
    pub params: TaskParams,
    pub state: &'static str,
    pub origin: &'static str,
    pub parent: Option<String>,
    pub enqueued_ns: u64,
    pub started_ns: Option<u64>,
    pub progress: Option<TaskProgress>,
    pub resume: Option<ResumeSnapshot>,
    pub est_duration_s: f64,
}

impl TaskEntry {
    fn info(&self, eta_to_start_s: Option<f64>) -> QueuedTaskInfo {
        QueuedTaskInfo {
            task_id: self.task_id.clone(),
            kind: self.kind.to_string(),
            params_digest: self.params.digest(),
            state: self.state.to_string(),
            origin: self.origin.to_string(),
            parent: self.parent.clone(),
            enqueued_ns: self.enqueued_ns,
            started_ns: self.started_ns,
            progress: self.progress.clone(),
            eta_to_start_s,
            est_duration_s: self.est_duration_s,
        }
    }
}

/// The vehicle's queue (behind `AgentShared::tasks`).
#[derive(Default)]
pub struct QueueState {
    seq: u64,
    active: Option<TaskEntry>,
    pending: VecDeque<TaskEntry>,
    /// Recent terminal entries kept visible on the stream.
    finished: VecDeque<TaskEntry>,
    driver_running: bool,
    /// Set by a displacing reorder: the pending index where the split
    /// continuation lands; consumed by [`on_task_end`].
    preempt_insert_at: Option<usize>,
    last_publish: Option<tokio::time::Instant>,
}

impl QueueState {
    fn next_id(&mut self) -> String {
        self.seq += 1;
        format!("tsk-{}", self.seq)
    }

    fn retire(&mut self, entry: TaskEntry) {
        self.finished.push_back(entry);
        while self.finished.len() > FINISHED_TAIL {
            self.finished.pop_front();
        }
    }

    /// Ids in run order: active first, then pending.
    fn queue_ids(&self) -> Vec<String> {
        self.active
            .iter()
            .chain(self.pending.iter())
            .map(|e| e.task_id.clone())
            .collect()
    }

    /// Seconds until the entry at pending index `upto` starts.
    fn eta_to_start_s(&self, upto: usize) -> f64 {
        let active_remaining = self.active.as_ref().map_or(0.0, |a| {
            a.progress.as_ref().map_or(a.est_duration_s, |p| p.eta_s)
        });
        active_remaining
            + self
                .pending
                .iter()
                .take(upto)
                .map(|e| e.est_duration_s)
                .sum::<f64>()
    }

    /// Pending queue length (tests / handlers).
    pub fn pending_len(&self) -> usize {
        self.pending.len()
    }

    /// The active entry's `(task_id, kind)` if any (tests / handlers).
    pub fn active_brief(&self) -> Option<(String, &'static str)> {
        self.active.as_ref().map(|e| (e.task_id.clone(), e.kind))
    }
}

// ---------------------------------------------------------------------------
// publishing
// ---------------------------------------------------------------------------

fn publish_locked(shared: &AgentShared, q: &mut QueueState) {
    let mut tasks = Vec::with_capacity(1 + q.pending.len() + q.finished.len());
    if let Some(active) = &q.active {
        tasks.push(active.info(None));
    }
    for (i, entry) in q.pending.iter().enumerate() {
        tasks.push(entry.info(Some(q.eta_to_start_s(i))));
    }
    for entry in &q.finished {
        tasks.push(entry.info(None));
    }
    let status = TaskQueueStatus {
        vehicle_id: shared.vehicle_id.clone(),
        gps_time_ns: crate::telemetry::gps_time_ns(),
        depth_limit: shared.queue_depth as u32,
        tasks,
    };
    match serde_json::to_vec(&status) {
        Ok(bytes) => *lock(&shared.latest_tasks) = Some(Bytes::from(bytes)),
        Err(err) => warn!(%err, "task queue status failed to encode"),
    }
    q.last_publish = Some(tokio::time::Instant::now());
}

// ---------------------------------------------------------------------------
// submission
// ---------------------------------------------------------------------------

/// Outcome of [`submit`].
pub enum Submit {
    /// Vehicle was idle: the task is ACTIVE and flying (busy label set).
    Started { task_id: String },
    /// Accepted into the queue behind `ahead` task(s) (active included).
    Queued {
        task_id: String,
        ahead: usize,
        eta_to_start_s: f64,
    },
    /// Pending depth limit reached.
    Full { depth: usize },
    /// The vehicle is flying the RTL ladder: work is never queued behind a
    /// return-to-launch (POLICY HOOK, ROUND-3 §2: acceptance rules become
    /// strategy-record-driven) — callers refuse busy, exactly v2.
    RtlOwned,
    /// The queue engine is disabled (`queue_enabled = false`): callers keep
    /// the legacy busy-refusal + direct-spawn semantics.
    Disabled,
}

/// Submit a task. When the vehicle is idle this OCCUPIES it (busy label +
/// abort flags cleared, exactly the v2 `occupy`) and starts the queue
/// driver, so the accept ack has the same race-free claim as v2.
pub fn submit(shared: &Arc<AgentShared>, params: TaskParams, origin: &'static str) -> Submit {
    if !shared.queue_enabled {
        return Submit::Disabled;
    }
    let kind = params.kind();
    let est = params.estimate_s(shared);
    let now = crate::telemetry::gps_time_ns();
    let mut q = lock(&shared.tasks);
    let mut busy = lock(&shared.busy);
    if *busy == "rtl" {
        return Submit::RtlOwned;
    }
    let idle = q.active.is_none() && busy.is_empty();
    if !idle && q.pending.len() >= shared.queue_depth {
        drop(busy);
        shared.journal.event(
            "task.refused",
            serde_json::json!({ "kind": kind, "code": "queue-full",
                                "depth": shared.queue_depth }),
        );
        return Submit::Full { depth: shared.queue_depth };
    }
    let task_id = q.next_id();
    let entry = TaskEntry {
        task_id: task_id.clone(),
        kind,
        params,
        state: if idle { task_state::ACTIVE } else { task_state::PENDING },
        origin,
        parent: None,
        enqueued_ns: now,
        started_ns: idle.then_some(now),
        progress: None,
        resume: None,
        est_duration_s: est,
    };
    let position = q.pending.len();
    shared.journal.event(
        "task.queued",
        serde_json::json!({
            "task_id": task_id, "kind": kind, "origin": origin,
            "position": if idle { 0 } else { position + 1 },
            "digest": entry.params.digest(),
        }),
    );
    if idle {
        *busy = kind.to_string();
        drop(busy);
        shared.abort.store(false, Ordering::Relaxed);
        shared.operator_abort.store(false, Ordering::Relaxed);
        shared
            .journal
            .event("task.started", serde_json::json!({ "task_id": task_id, "kind": kind }));
        q.active = Some(entry);
        ensure_driver(shared, &mut q);
        publish_locked(shared, &mut q);
        Submit::Started { task_id }
    } else {
        drop(busy);
        q.pending.push_back(entry);
        let ahead = q.pending.len(); // active + pending ahead (1-based slot)
        let eta = q.eta_to_start_s(position);
        publish_locked(shared, &mut q);
        Submit::Queued {
            task_id,
            ahead,
            eta_to_start_s: eta,
        }
    }
}

/// Start the driver task if it is not already running (call with the queue
/// lock held).
fn ensure_driver(shared: &Arc<AgentShared>, q: &mut QueueState) {
    if !q.driver_running {
        q.driver_running = true;
        let shared = shared.clone();
        tokio::spawn(async move { drive(shared).await });
    }
}

/// Promote the queue after a NON-queue owner released the vehicle (takeoff
/// finished, idle-vehicle override wrapper ended): if entries are pending
/// and the vehicle is idle, start the front one.
pub(crate) fn kick(shared: &Arc<AgentShared>) {
    let mut q = lock(&shared.tasks);
    if q.active.is_some() || q.pending.is_empty() || shared.abort.load(Ordering::Relaxed) {
        return;
    }
    let mut busy = lock(&shared.busy);
    if !busy.is_empty() {
        return;
    }
    let mut next = q.pending.pop_front().expect("checked non-empty");
    *busy = next.kind.to_string();
    drop(busy);
    shared.operator_abort.store(false, Ordering::Relaxed);
    next.state = task_state::ACTIVE;
    next.started_ns = Some(crate::telemetry::gps_time_ns());
    shared.journal.event(
        "task.started",
        serde_json::json!({ "task_id": next.task_id, "kind": next.kind }),
    );
    q.active = Some(next);
    ensure_driver(shared, &mut q);
    publish_locked(shared, &mut q);
}

// ---------------------------------------------------------------------------
// runner feedback (progress + resume)
// ---------------------------------------------------------------------------

/// Update the active entry's live progress (+ optional resume refresh) and
/// republish at most every [`PROGRESS_PUBLISH_PERIOD`].
pub(crate) fn note_progress(
    shared: &AgentShared,
    progress: TaskProgress,
    resume: Option<ResumeSnapshot>,
) {
    let mut q = lock(&shared.tasks);
    let Some(active) = q.active.as_mut() else { return };
    active.progress = Some(progress);
    if let Some(resume) = resume {
        active.resume = Some(resume);
    }
    let due = q
        .last_publish
        .is_none_or(|t| t.elapsed() >= PROGRESS_PUBLISH_PERIOD);
    if due {
        publish_locked(shared, &mut q);
    }
}

/// Exact remaining-work snapshot, written by the runner the moment it
/// observes an interruption (split fidelity depends on this being the last
/// word).
pub(crate) fn save_resume(shared: &AgentShared, resume: ResumeSnapshot) {
    let mut q = lock(&shared.tasks);
    if let Some(active) = q.active.as_mut() {
        active.resume = Some(resume);
    }
}

// ---------------------------------------------------------------------------
// aborts / flush / reorder
// ---------------------------------------------------------------------------

/// `task_abort("tsk-<n>")` resolution.
pub enum ById {
    /// A pending entry was removed (state `aborted`, journaled).
    Pending,
    /// The id names the ACTIVE entry: treat like a label abort of `kind`.
    Active(&'static str),
    None,
}

/// Remove a PENDING entry by id, or identify the active one.
pub fn abort_by_id(shared: &AgentShared, task_id: &str) -> ById {
    let mut q = lock(&shared.tasks);
    if let Some(active) = &q.active {
        if active.task_id == task_id {
            return ById::Active(active.kind);
        }
    }
    let Some(pos) = q.pending.iter().position(|e| e.task_id == task_id) else {
        return ById::None;
    };
    let mut entry = q.pending.remove(pos).expect("position found above");
    entry.state = task_state::ABORTED;
    shared.journal.event(
        "task.aborted",
        serde_json::json!({ "task_id": entry.task_id, "kind": entry.kind, "by": "operator" }),
    );
    q.retire(entry);
    publish_locked(shared, &mut q);
    ById::Pending
}

/// Blanket flush: every pending entry aborts (the ladder commands and the
/// driver's ladder-abort path both call this; idempotent).
pub fn flush_pending(shared: &AgentShared, by: &str) {
    let mut q = lock(&shared.tasks);
    flush_pending_locked(shared, &mut q, by);
    publish_locked(shared, &mut q);
}

fn flush_pending_locked(shared: &AgentShared, q: &mut QueueState, by: &str) {
    q.preempt_insert_at = None;
    while let Some(mut entry) = q.pending.pop_front() {
        entry.state = task_state::ABORTED;
        shared.journal.event(
            "task.aborted",
            serde_json::json!({ "task_id": entry.task_id, "kind": entry.kind, "by": by }),
        );
        q.retire(entry);
    }
}

/// Apply a full-queue reorder. `ordered_ids` must be exactly the current
/// queue's ids (active + pending, any order). Displacing the active id from
/// position 0 raises the preempt: the runner is aborted (existing abort
/// machinery) and [`on_task_end`] enqueues the split continuation at the
/// active id's new position. Returns whether a split was triggered.
pub fn reorder(shared: &AgentShared, ordered_ids: &[String]) -> Result<bool, String> {
    let mut q = lock(&shared.tasks);
    let current = q.queue_ids();
    if current.is_empty() {
        return Err("queue is empty".to_string());
    }
    {
        let mut want: Vec<&String> = ordered_ids.iter().collect();
        let mut have: Vec<&String> = current.iter().collect();
        want.sort_unstable();
        have.sort_unstable();
        if want != have {
            return Err(format!(
                "ordered_task_ids must be a permutation of the current queue {current:?}"
            ));
        }
    }
    let active_id = q.active.as_ref().map(|e| e.task_id.clone());
    // New pending order = ordered ids minus the active id.
    let mut new_pending = VecDeque::with_capacity(q.pending.len());
    for id in ordered_ids {
        if Some(id) == active_id.as_ref() {
            continue;
        }
        let pos = q
            .pending
            .iter()
            .position(|e| &e.task_id == id)
            .expect("validated as permutation above");
        new_pending.push_back(q.pending.remove(pos).expect("position found above"));
    }
    q.pending = new_pending;
    let split = match (&active_id, ordered_ids.first()) {
        (Some(active), Some(first)) => active != first,
        _ => false,
    };
    if split {
        let insert_at = ordered_ids
            .iter()
            .position(|id| Some(id) == active_id.as_ref())
            .expect("active id validated present");
        q.preempt_insert_at = Some(insert_at);
        // Suspend through the runners' existing abort machinery; the runner
        // saves its exact resume snapshot on the way out and the driver
        // (on_task_end) builds + inserts the continuation.
        shared.abort.store(true, Ordering::Relaxed);
    }
    shared.journal.event(
        "queue.reordered",
        serde_json::json!({ "order": ordered_ids, "split": split }),
    );
    publish_locked(shared, &mut q);
    Ok(split)
}

// ---------------------------------------------------------------------------
// RC-over-NDN arbitration (RC-CONTROL R1)
// ---------------------------------------------------------------------------

/// Outcome of [`rc_seize`].
pub(crate) enum RcSeize {
    /// Idle vehicle: the `rc-manual` busy claim is taken.
    Engaged,
    /// The active queue task was suspended through the split machinery —
    /// its remainder sits at the FRONT of pending and resumes via [`kick`]
    /// when the RC session releases.
    EngagedPaused { paused_task_id: String },
    /// Something owns the vehicle and policy refuses the engage.
    Refused { busy: String, reason: &'static str },
}

/// RC engage arbitration, race-free under the queue + busy locks (same
/// lock order as [`submit`]).
///
/// - Idle vehicle: claim `rc-manual` (the RC analogue of `occupy`).
/// - Active mission task: only with the frame's ARM_GESTURE held AND
///   `--rc-preempt pause-mission` — the active task is suspended exactly
///   like a reorder displacement (`preempt_insert_at = 0`, abort raised;
///   the runner snapshots its remainder on the way out and [`on_task_end`]
///   enqueues the `origin=split` continuation at the front). The busy
///   label hands to RC in the same critical section, which is what makes
///   [`on_task_end`] KEEP the pending queue instead of flushing it (see
///   the re-label branch there).
/// - Non-queue owners (`takeoff`, `rtl`, a `--no-queue` legacy task, an
///   override detour): nothing to split — refused.
pub(crate) fn rc_seize(shared: &AgentShared, arm_gesture: bool, pause_allowed: bool) -> RcSeize {
    let mut q = lock(&shared.tasks);
    let mut busy = lock(&shared.busy);
    if busy.is_empty() {
        *busy = crate::rc::RC_MANUAL.to_string();
        shared.abort.store(false, Ordering::Relaxed);
        shared.operator_abort.store(false, Ordering::Relaxed);
        return RcSeize::Engaged;
    }
    if !arm_gesture {
        return RcSeize::Refused { busy: busy.clone(), reason: "arm-gesture-required" };
    }
    if *busy == "rtl" {
        // Work is never taken from a return-to-launch (the v2 rule).
        return RcSeize::Refused { busy: busy.clone(), reason: "rtl-owns-vehicle" };
    }
    if !pause_allowed {
        return RcSeize::Refused { busy: busy.clone(), reason: "preempt-denied" };
    }
    let pausable = q.active.as_ref().filter(|active| active.kind == *busy);
    let Some(active) = pausable else {
        return RcSeize::Refused { busy: busy.clone(), reason: "unpausable-owner" };
    };
    let paused_task_id = active.task_id.clone();
    q.preempt_insert_at = Some(0);
    shared.operator_abort.store(false, Ordering::Relaxed);
    shared.abort.store(true, Ordering::Relaxed);
    *busy = crate::rc::RC_MANUAL.to_string();
    RcSeize::EngagedPaused { paused_task_id }
}

// ---------------------------------------------------------------------------
// the driver
// ---------------------------------------------------------------------------

async fn drive(shared: Arc<AgentShared>) {
    loop {
        let runnable = {
            let mut q = lock(&shared.tasks);
            match q.active.as_ref() {
                Some(entry) => Some((entry.task_id.clone(), entry.params.clone())),
                None => {
                    q.driver_running = false;
                    None
                }
            }
        };
        let Some((task_id, params)) = runnable else { return };
        let outcome = run_task(&shared, &params).await;
        match on_task_end(&shared, &task_id, outcome) {
            AfterTask::Next => {}
            AfterTask::Idle(after) => {
                // Queue drained on a natural/operator end: the post-task
                // idle policy takes over (it re-checks idleness itself).
                crate::mission::apply_idle_policy(&shared, after);
            }
            AfterTask::Stop => {
                lock(&shared.tasks).driver_running = false;
                return;
            }
        }
    }
}

async fn run_task(shared: &Arc<AgentShared>, params: &TaskParams) -> FlightOutcome {
    match params {
        TaskParams::Raster { req, start_leg, skip_captures } => {
            match crate::mission::plan_raster(req) {
                Err(err) => {
                    // Should have been caught at ack; journal + fail.
                    shared
                        .journal
                        .event("task.plan_failed", serde_json::json!({ "error": err }));
                    FlightOutcome::TimedOut
                }
                Ok(plan) => match plan.remainder(*start_leg, *skip_captures) {
                    // Empty remainder: the parent finished everything.
                    None => FlightOutcome::Completed,
                    Some(remainder) => {
                        crate::mission::raster_flight_loop(shared, req, remainder).await
                    }
                },
            }
        }
        TaskParams::Investigate { req } => {
            crate::mission::investigate_flight_loop(shared, req).await
        }
        TaskParams::SensorOverride { req } => {
            crate::sensor::override_capture_core(shared, req, task_kind::SENSOR_OVERRIDE, true)
                .await
        }
    }
}

enum AfterTask {
    /// Another entry is ACTIVE (busy label already swapped); keep driving.
    Next,
    /// Queue empty, vehicle released: apply the idle policy for this kind.
    Idle(&'static str),
    /// Another command owns the vehicle (ladder/shutdown): stand down.
    Stop,
}

/// Terminal bookkeeping for the active entry + hand-off to the next one.
fn on_task_end(shared: &Arc<AgentShared>, task_id: &str, outcome: FlightOutcome) -> AfterTask {
    // A scoped operator abort (task_abort) is consumed here — the queue
    // continues; the ladder's blanket abort is NOT consumed (the
    // interrupting command owns the vehicle).
    let operator = outcome == FlightOutcome::Aborted
        && shared.operator_abort.swap(false, Ordering::Relaxed);
    let mut q = lock(&shared.tasks);
    let Some(mut entry) = q.active.take() else { return AfterTask::Stop };
    debug_assert_eq!(entry.task_id, task_id);
    let kind = entry.kind;
    let preempt = if outcome == FlightOutcome::Aborted && !operator {
        q.preempt_insert_at.take()
    } else {
        q.preempt_insert_at = None;
        None
    };

    let (state, continue_queue) = match outcome {
        FlightOutcome::Completed => (task_state::DONE, true),
        FlightOutcome::TimedOut => (task_state::FAILED, true),
        FlightOutcome::Aborted if operator => (task_state::ABORTED, true),
        // Preempted by a reorder: the parent side of a split counts done.
        FlightOutcome::Aborted if preempt.is_some() => (task_state::DONE, true),
        FlightOutcome::Aborted => (task_state::ABORTED, false), // ladder
    };
    entry.state = state;
    if state == task_state::ABORTED {
        shared.journal.event(
            "task.aborted",
            serde_json::json!({
                "task_id": entry.task_id, "kind": kind,
                "by": if operator { "operator" } else { "ladder" },
            }),
        );
    } else {
        shared.journal.event(
            "task.completed",
            serde_json::json!({
                "task_id": entry.task_id, "kind": kind, "outcome": outcome.as_str(),
                "split": preempt.is_some(),
            }),
        );
    }

    // Split continuation: the remainder (from the runner's exact resume
    // snapshot) re-enters the queue at the displaced position.
    if let Some(insert_at) = preempt {
        shared.abort.store(false, Ordering::Relaxed); // reorder raised it
        if let Some(params) = continuation_params(&entry) {
            let cont_id = q.next_id();
            let est = params.estimate_s(shared);
            shared.journal.event(
                "task.split",
                serde_json::json!({
                    "task_id": entry.task_id, "continuation": cont_id,
                    "insert_at": insert_at, "digest": params.digest(),
                }),
            );
            let cont = TaskEntry {
                task_id: cont_id,
                kind,
                params,
                state: task_state::PENDING,
                origin: task_origin::SPLIT,
                parent: Some(entry.task_id.clone()),
                enqueued_ns: crate::telemetry::gps_time_ns(),
                started_ns: None,
                progress: None,
                resume: None,
                est_duration_s: est,
            };
            let at = insert_at.min(q.pending.len());
            q.pending.insert(at, cont);
        }
    } else if operator {
        shared.abort.store(false, Ordering::Relaxed); // legacy handoff rule
    }
    q.retire(entry);

    if !continue_queue || shared.cancel.is_cancelled() {
        flush_pending_locked(shared, &mut q, "ladder");
        publish_locked(shared, &mut q);
        return AfterTask::Stop;
    }
    // A fresh interrupting command may have landed during the epilogue.
    if shared.abort.load(Ordering::Relaxed) {
        flush_pending_locked(shared, &mut q, "ladder");
        publish_locked(shared, &mut q);
        return AfterTask::Stop;
    }
    let mut busy = lock(&shared.busy);
    if !busy.is_empty() && *busy != kind {
        // Re-labelled: another command owns the vehicle. `rc-manual`
        // (pause-mission preempt) SUSPENDS the queue — pending entries,
        // the split continuation included, stay put and the RC release
        // path restarts them via `kick`. Anything else (rtl) is the
        // ladder: nothing queued survives it.
        let rc_pause = *busy == crate::rc::RC_MANUAL;
        drop(busy);
        if !rc_pause {
            flush_pending_locked(shared, &mut q, "ladder");
        }
        publish_locked(shared, &mut q);
        return AfterTask::Stop;
    }
    match q.pending.pop_front() {
        Some(mut next) => {
            // No observable idle gap between queue items: the busy label
            // swaps straight to the next kind (dashboard busy→idle
            // completion fires when the WHOLE queue drains).
            *busy = next.kind.to_string();
            drop(busy);
            shared.operator_abort.store(false, Ordering::Relaxed);
            next.state = task_state::ACTIVE;
            next.started_ns = Some(crate::telemetry::gps_time_ns());
            shared.journal.event(
                "task.started",
                serde_json::json!({ "task_id": next.task_id, "kind": next.kind }),
            );
            info!(task_id = %next.task_id, kind = next.kind, "queue: next task starts");
            q.active = Some(next);
            publish_locked(shared, &mut q);
            AfterTask::Next
        }
        None => {
            if *busy == kind {
                busy.clear();
            }
            drop(busy);
            publish_locked(shared, &mut q);
            AfterTask::Idle(kind)
        }
    }
}

/// The remainder of a preempted task. `None` = nothing left to resume.
fn continuation_params(entry: &TaskEntry) -> Option<TaskParams> {
    match (&entry.params, entry.resume) {
        (
            TaskParams::Raster { req, start_leg, skip_captures },
            Some(ResumeSnapshot::Raster { leg, fired_in_leg }),
        ) => Some(TaskParams::Raster {
            req: req.clone(),
            start_leg: start_leg + leg,
            skip_captures: if leg == 0 { skip_captures + fired_in_leg } else { fired_in_leg },
        }),
        (TaskParams::Investigate { req }, Some(ResumeSnapshot::Investigate { remaining_turns })) => {
            if remaining_turns <= 0.02 {
                return None; // effectively finished
            }
            let mut req = req.clone();
            req.turns = remaining_turns;
            Some(TaskParams::Investigate { req })
        }
        // No snapshot yet (preempted before the flight got going), or an
        // atomic override (nothing to split): the remainder is the whole
        // request.
        (params, _) => Some(params.clone()),
    }
}

// ---------------------------------------------------------------------------
// tests — bench agent shared state over a sim backend, paused tokio time
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use muas_contracts::services::{
        InvestigateRequest, QueueReorderRequest, RasterRequest, VehicleService,
    };
    use std::sync::Mutex;
    use uas_flight::geo::{m_per_deg_lon, EARTH_M_PER_DEG_LAT};
    use uas_fleet_node::flight_backend::{SimFlightBackend, SIM_TICK_S};

    const ORIGIN: (f64, f64) = (35.0, -90.0);

    fn bench(
        vehicle_id: &str,
        log_dir: Option<std::path::PathBuf>,
        customize: impl FnOnce(&mut AgentShared),
    ) -> (Arc<AgentShared>, crate::SharedBackend) {
        let (journal, _task) = crate::journal::spawn(vehicle_id, log_dir, None, None);
        let (cmd_tx, _cmd_rx) = tokio::sync::mpsc::unbounded_channel();
        let sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        let backend: crate::SharedBackend =
            Arc::new(Mutex::new(Box::new(sim) as Box<dyn crate::TickableBackend>));
        let mut shared = AgentShared::bench(vehicle_id, backend.clone(), journal, cmd_tx);
        customize(&mut shared);
        let shared = Arc::new(shared);
        {
            let backend = backend.clone();
            tokio::spawn(async move {
                let mut interval =
                    tokio::time::interval(Duration::from_secs_f64(SIM_TICK_S));
                loop {
                    interval.tick().await;
                    lock(&backend).advance(SIM_TICK_S);
                }
            });
        }
        (shared, backend)
    }

    fn service(shared: &Arc<AgentShared>) -> crate::service_impl::VehicleServiceImpl {
        crate::service_impl::VehicleServiceImpl::new(shared.clone())
    }

    fn investigate_req(north_m: f64, turns: f64) -> InvestigateRequest {
        InvestigateRequest {
            lat_deg: ORIGIN.0 + north_m / EARTH_M_PER_DEG_LAT,
            lon_deg: ORIGIN.1,
            agl_m: 8.0,
            radius_m: 6.0,
            turns,
            sensors: vec!["camera".into()],
            mission_id: String::new(), // operator origin
            ..InvestigateRequest::default()
        }
    }

    fn raster_req(width_m: f64, height_m: f64, spacing: f64, step: f64) -> RasterRequest {
        let dlat = (height_m / 2.0) / EARTH_M_PER_DEG_LAT;
        let dlon = (width_m / 2.0) / m_per_deg_lon(ORIGIN.0);
        RasterRequest {
            agl_m: 8.0,
            spacing_m: spacing,
            capture_every_m: step,
            speed_m_s: 5.0,
            corners: vec![
                (ORIGIN.0 + dlat, ORIGIN.1 - dlon),
                (ORIGIN.0 + dlat, ORIGIN.1 + dlon),
                (ORIGIN.0 - dlat, ORIGIN.1 + dlon),
                (ORIGIN.0 - dlat, ORIGIN.1 - dlon),
            ],
            ..RasterRequest::default()
        }
    }

    fn status_of(shared: &AgentShared) -> TaskQueueStatus {
        serde_json::from_slice(
            lock(&shared.latest_tasks).as_ref().expect("queue stream published"),
        )
        .expect("queue stream decodes")
    }

    fn state_of(shared: &AgentShared, task_id: &str) -> Option<String> {
        status_of(shared)
            .tasks
            .iter()
            .find(|t| t.task_id == task_id)
            .map(|t| t.state.clone())
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
            tokio::time::sleep(Duration::from_millis(100)).await;
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
            "muas-queue-test-{tag}-{}-{}",
            std::process::id(),
            crate::telemetry::gps_time_ns()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    /// task.started journal order (the actual execution order).
    fn started_order(lines: &[serde_json::Value]) -> Vec<String> {
        lines
            .iter()
            .filter(|l| l["kind"] == "task.started")
            .map(|l| l["task_id"].as_str().unwrap_or_default().to_string())
            .collect()
    }

    /// Enqueue-while-active + queued ack shape + in-order drain: the
    /// service-level accept-and-queue contract end to end.
    #[tokio::test(start_paused = true)]
    async fn enqueue_while_active_acks_queued_and_drains_in_order() {
        let dir = temp_log_dir("drain");
        let (shared, _backend) = bench("iuas-50", Some(dir.clone()), |_| {});
        let svc = service(&shared);

        let first = svc.investigate(investigate_req(40.0, 1.0)).await;
        assert!(first.accepted, "idle vehicle starts immediately: {}", first.detail);
        assert!(first.code.is_empty(), "immediate start carries no advisory code");
        assert_eq!(*lock(&shared.busy), "investigate", "busy string = active task kind");

        // Second request: ACCEPTED with the queued advisory code; detail
        // carries task id + position + ETA-to-start.
        let second = svc.investigate(investigate_req(-40.0, 1.0)).await;
        assert!(second.accepted, "busy no longer refuses: {}", second.detail);
        assert_eq!(second.code, "queued");
        assert!(
            second.detail.contains("tsk-2") && second.detail.contains("position 1"),
            "detail names the task + position: '{}'",
            second.detail
        );
        assert!(second.detail.contains("starts in ~"), "detail: '{}'", second.detail);

        // The stream shows the ordered queue with an ETA for the pending entry.
        let status = status_of(&shared);
        assert_eq!(status.depth_limit, DEFAULT_QUEUE_DEPTH as u32);
        assert_eq!(status.tasks[0].task_id, "tsk-1");
        assert_eq!(status.tasks[0].state, task_state::ACTIVE);
        assert_eq!(status.tasks[1].task_id, "tsk-2");
        assert_eq!(status.tasks[1].state, task_state::PENDING);
        assert!(status.tasks[1].eta_to_start_s.is_some());
        assert!(status.tasks[1].est_duration_s > 0.0);

        // Both drain, in order, with NO idle gap needed between them.
        assert!(
            wait_until(600.0, || {
                lock(&shared.busy).is_empty()
                    && state_of(&shared, "tsk-1").as_deref() == Some(task_state::DONE)
                    && state_of(&shared, "tsk-2").as_deref() == Some(task_state::DONE)
            })
            .await,
            "queue never drained: {:?}",
            status_of(&shared)
        );

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert_eq!(started_order(&lines), vec!["tsk-1", "tsk-2"], "in-order execution");
        assert_eq!(
            lines.iter().filter(|l| l["kind"] == "task.completed").count(),
            2,
            "both tasks journal completion"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test(start_paused = true)]
    async fn depth_limit_refuses_queue_full() {
        let (shared, _backend) = bench("iuas-51", None, |s| s.queue_depth = 1);
        let svc = service(&shared);
        assert!(svc.investigate(investigate_req(40.0, 3.0)).await.accepted);
        assert_eq!(svc.investigate(investigate_req(50.0, 1.0)).await.code, "queued");
        let third = svc.investigate(investigate_req(60.0, 1.0)).await;
        assert!(!third.accepted);
        assert_eq!(third.code, "queue-full");
        // The refused request never entered the queue.
        assert_eq!(lock(&shared.tasks).pending_len(), 1);
        shared.abort.store(true, Ordering::Relaxed); // wind the bench down
    }

    #[tokio::test(start_paused = true)]
    async fn task_abort_removes_a_pending_entry_by_id() {
        let dir = temp_log_dir("abort-pending");
        let (shared, _backend) = bench("iuas-52", Some(dir.clone()), |_| {});
        let svc = service(&shared);
        assert!(svc.investigate(investigate_req(40.0, 3.0)).await.accepted);
        assert_eq!(svc.investigate(investigate_req(50.0, 1.0)).await.code, "queued");
        assert_eq!(svc.investigate(investigate_req(60.0, 1.0)).await.code, "queued");

        // Unknown ids refuse; nothing changes.
        let miss = svc.task_abort("tsk-99".into()).await;
        assert!(!miss.accepted);
        assert_eq!(miss.code, "no-such-task");
        assert_eq!(lock(&shared.tasks).pending_len(), 2);

        // Remove ONE pending entry by id: the flight is untouched.
        let ack = svc.task_abort("tsk-2".into()).await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        assert_eq!(lock(&shared.tasks).pending_len(), 1);
        assert_eq!(state_of(&shared, "tsk-2").as_deref(), Some(task_state::ABORTED));
        assert_eq!(*lock(&shared.busy), "investigate", "active task untouched");

        // The remaining pending entry still runs (tsk-1 then tsk-3).
        let ack = svc.task_abort("tsk-1".into()).await; // active id = its label
        assert!(ack.accepted, "active id aborts like its label: {}", ack.detail);
        assert!(
            wait_until(600.0, || {
                state_of(&shared, "tsk-3").as_deref() == Some(task_state::DONE)
            })
            .await,
            "tsk-3 never ran after the aborts"
        );

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "task.aborted" && l["task_id"] == "tsk-2" && l["by"] == "operator"));
        assert_eq!(started_order(&lines), vec!["tsk-1", "tsk-3"]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test(start_paused = true)]
    async fn reorder_swaps_pending_order_without_split() {
        let dir = temp_log_dir("reorder");
        let (shared, _backend) = bench("iuas-53", Some(dir.clone()), |_| {});
        let svc = service(&shared);
        assert!(svc.investigate(investigate_req(40.0, 3.0)).await.accepted);
        assert_eq!(svc.investigate(investigate_req(50.0, 1.0)).await.code, "queued");
        assert_eq!(svc.investigate(investigate_req(60.0, 1.0)).await.code, "queued");

        // Mismatched id sets refuse.
        let bad = svc
            .queue_reorder(QueueReorderRequest {
                ordered_task_ids: vec!["tsk-1".into(), "tsk-3".into()],
            })
            .await;
        assert!(!bad.accepted);
        assert_eq!(bad.code, "bad-reorder");

        // Active stays at position 0: pending swap, NO split.
        let ack = svc
            .queue_reorder(QueueReorderRequest {
                ordered_task_ids: vec!["tsk-1".into(), "tsk-3".into(), "tsk-2".into()],
            })
            .await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        assert!(!ack.detail.contains("split"), "no split: '{}'", ack.detail);
        let status = status_of(&shared);
        let order: Vec<&str> = status.tasks.iter().map(|t| t.task_id.as_str()).collect();
        assert_eq!(&order[..3], &["tsk-1", "tsk-3", "tsk-2"]);

        // Drain: execution follows the new order.
        assert!(
            wait_until(900.0, || lock(&shared.busy).is_empty()
                && state_of(&shared, "tsk-2").as_deref() == Some(task_state::DONE))
            .await,
            "queue never drained"
        );
        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert_eq!(started_order(&lines), vec!["tsk-1", "tsk-3", "tsk-2"]);
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "queue.reordered" && l["split"] == false));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The headline feature: a reorder that displaces the ACTIVE raster
    /// splits it — the prioritized investigate flies, then the continuation
    /// (origin=split, parent) finishes the raster with EVERY capture fired
    /// exactly once (no duplicates, none lost).
    #[tokio::test(start_paused = true)]
    async fn reorder_displacing_active_raster_splits_and_resumes_all_captures() {
        let dir = temp_log_dir("split");
        let (shared, _backend) = bench("wuas-54", Some(dir.clone()), |_| {});
        let svc = service(&shared);

        let raster = raster_req(120.0, 60.0, 20.0, 15.0);
        let capture_total = crate::mission::plan_raster(&raster).unwrap().capture_count();
        let ack = svc.raster_search(raster).await;
        assert!(ack.accepted, "detail: {}", ack.detail);

        // Let it fire a few captures mid-sweep.
        assert!(
            wait_until(300.0, || {
                lock(&shared.latest_search)
                    .as_ref()
                    .and_then(|b| {
                        serde_json::from_slice::<uas_fleet_data::kinds::SearchStatus>(b).ok()
                    })
                    .is_some_and(|s| s.frames_captured >= 3 && s.state == "searching")
            })
            .await,
            "raster never got going"
        );

        // Queue an investigate, then move it AHEAD of the running raster.
        assert_eq!(svc.investigate(investigate_req(50.0, 1.0)).await.code, "queued");
        let ack = svc
            .queue_reorder(QueueReorderRequest {
                ordered_task_ids: vec!["tsk-2".into(), "tsk-1".into()],
            })
            .await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        assert!(ack.detail.contains("split"), "ack names the split: '{}'", ack.detail);

        // The investigate takes the vehicle, then the continuation (split
        // child of tsk-1) resumes and the raster completes fully.
        assert!(
            wait_until(900.0, || lock(&shared.busy).is_empty()
                && state_of(&shared, "tsk-2").as_deref() == Some(task_state::DONE)
                && state_of(&shared, "tsk-3").as_deref() == Some(task_state::DONE))
            .await,
            "split-resume never completed: {:?}",
            status_of(&shared)
        );
        let status = status_of(&shared);
        let continuation = status
            .tasks
            .iter()
            .find(|t| t.task_id == "tsk-3")
            .expect("continuation on the stream");
        assert_eq!(continuation.origin, task_origin::SPLIT);
        assert_eq!(continuation.parent.as_deref(), Some("tsk-1"));
        assert_eq!(continuation.kind, task_kind::RASTER_SEARCH);

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        // Execution order: raster, prioritized investigate, continuation.
        assert_eq!(started_order(&lines), vec!["tsk-1", "tsk-2", "tsk-3"]);
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "task.split"
                && l["task_id"] == "tsk-1"
                && l["continuation"] == "tsk-3"));
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "queue.reordered" && l["split"] == true));

        // Split fidelity: every planned capture fired EXACTLY once across
        // parent + continuation (planned points are bit-identical between
        // the plan and its remainder).
        let mut planned: Vec<(u64, u64)> = lines
            .iter()
            .filter(|l| l["kind"] == "search.capture")
            .map(|l| {
                (
                    l["planned"]["lat_deg"].as_f64().unwrap().to_bits(),
                    l["planned"]["lon_deg"].as_f64().unwrap().to_bits(),
                )
            })
            .collect();
        assert_eq!(planned.len(), capture_total, "total captures = plan total");
        planned.sort_unstable();
        planned.dedup();
        assert_eq!(planned.len(), capture_total, "no capture point fired twice");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The active entry publishes live progress (pct / detail / eta) on the
    /// `tasks/queue` stream while a raster flies.
    #[tokio::test(start_paused = true)]
    async fn active_task_publishes_progress_snapshots() {
        let (shared, _backend) = bench("wuas-55", None, |_| {});
        let svc = service(&shared);
        assert!(svc.raster_search(raster_req(200.0, 100.0, 25.0, 20.0)).await.accepted);
        assert!(
            wait_until(300.0, || {
                status_of(&shared).tasks.first().and_then(|t| t.progress.clone()).is_some_and(
                    |p| {
                        p.detail.contains("leg")
                            && (0.0..=100.0).contains(&p.pct)
                            && p.eta_s > 0.0
                    },
                )
            })
            .await,
            "no progress snapshot ever published: {:?}",
            status_of(&shared)
        );
        shared.abort.store(true, Ordering::Relaxed); // wind the bench down
    }

    /// The ladder (rtl/land/hold) flushes every pending entry: the blanket
    /// stop owns the vehicle, nothing queued survives it.
    #[tokio::test(start_paused = true)]
    async fn ladder_abort_flushes_pending_entries() {
        let dir = temp_log_dir("ladder");
        let (shared, _backend) = bench("iuas-56", Some(dir.clone()), |_| {});
        let svc = service(&shared);
        assert!(svc.investigate(investigate_req(40.0, 3.0)).await.accepted);
        assert_eq!(svc.investigate(investigate_req(50.0, 1.0)).await.code, "queued");

        let ack = svc.flight_hold().await;
        assert!(ack.accepted);
        assert!(
            wait_until(30.0, || {
                state_of(&shared, "tsk-1").as_deref() == Some(task_state::ABORTED)
                    && state_of(&shared, "tsk-2").as_deref() == Some(task_state::ABORTED)
            })
            .await,
            "ladder must abort active AND pending: {:?}",
            status_of(&shared)
        );
        shared.journal.sync().await;
        assert!(journal_lines(&dir)
            .iter()
            .any(|l| l["kind"] == "task.aborted" && l["task_id"] == "tsk-2" && l["by"] == "ladder"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
