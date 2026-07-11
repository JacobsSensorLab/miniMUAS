//! The detect→confirm→queue→dispatch mission state machine — the brain the
//! agents deliberately don't have, ported faithfully from v2
//! `run_dashboard.py` (survey §Confirm-then-queue & multi-target dispatch).
//!
//! Pure and synchronous: every input returns a list of [`Action`]s (events
//! to broadcast, detections to fan out, jobs to dispatch) that the async
//! layer executes. Tests drive the machine directly with scripted
//! detections — no NDN, no sockets, no clocks it doesn't own.
//!
//! Semantics pinned from v2:
//! - **Confirm-then-queue**: a hit first reinforces a *candidate*; only a
//!   candidate seen on `confirm_count` separate frames is promoted to a
//!   dispatched target (the guard against the field failure where one 99%
//!   texture false-positive launched the IUAS).
//! - **Best-localized wins**: candidate/target position comes from the
//!   sighting with the smallest `offset_m` (object nearest the frame center
//!   ⇒ least AGL/heading lever-arm error), NOT the highest confidence.
//! - **Multi-target multi-sensor**: one job per requested sensor per
//!   target, each dispatched to any idle, enabled, capability-matching
//!   IUAS; the raster search continues while the queue drains.
//! - **Completion**: raster done + nothing in flight + no queued job any
//!   currently-enabled vehicle could serve.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::Arc;

use muas_contracts::strategy::{
    rank_candidates, reask_schedule, CandidateSnapshot, DispatchStrategy, RequesterStrategy,
};
use serde::Serialize;
use serde_json::{json, Value};

/// Flat-earth metres per degree of latitude (the v2 constant).
const M_PER_DEG_LAT: f64 = 111_111.0;

fn dist_m(lat_a: f64, lon_a: f64, lat_b: f64, lon_b: f64) -> f64 {
    let dn = (lat_a - lat_b) * M_PER_DEG_LAT;
    let de = (lon_a - lon_b)
        * M_PER_DEG_LAT
        * f64::max(((lat_a + lat_b) / 2.0).to_radians().cos(), 1e-6);
    dn.hypot(de)
}

fn round4(v: f64) -> f64 {
    (v * 1e4).round() / 1e4
}

fn round2(v: f64) -> f64 {
    (v * 1e2).round() / 1e2
}

// ───────────────────────────── inputs ───────────────────────────────────────

/// One localized detection (the v2 `DetectionResponse` facts the machine
/// consumes).
#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    /// Detected object class/id (e.g. `"tennis racket"`).
    pub object_id: String,
    /// Detector confidence, 0..1.
    pub confidence: f64,
    /// Ground-projected latitude, degrees.
    pub lat_deg: f64,
    /// Ground-projected longitude, degrees.
    pub lon_deg: f64,
    /// Nadir offset of the object in the frame, metres (localization
    /// quality: smaller = better).
    pub offset_m: f64,
}

/// Outcome of one detection request.
#[derive(Debug, Clone, PartialEq)]
pub enum DetectOutcome {
    /// The provider found the queried object.
    Hit(Detection),
    /// The provider answered without a find (carries the provider error /
    /// miss note, empty for a clean miss).
    Miss(String),
    /// The provider never answered.
    Timeout,
}

/// A completed investigation job result fed back into the machine.
#[derive(Debug, Clone, PartialEq)]
pub struct JobResult {
    /// Which target the job belonged to.
    pub target_index: usize,
    /// Which sensor job it was.
    pub sensor: String,
    /// Completed vs failed.
    pub ok: bool,
    /// Artifact data names delivered by the job.
    pub artifacts: Vec<String>,
    /// Free-form completion note (rejection detail, `"timeout"`, ...).
    pub note: String,
    /// Artifact items for the sensor-data map layer:
    /// `(kind, data_name, lat_deg, lon_deg)` from the artifact's own
    /// capture pose.
    pub artifact_items: Vec<(String, String, f64, f64)>,
}

// ───────────────────────────── outputs ──────────────────────────────────────

/// The raster order handed to the WUAS (v3 `RasterRequest` facts plus the
/// dashboard-side service deadline).
#[derive(Debug, Clone, PartialEq)]
pub struct RasterOrder {
    pub mission_id: String,
    pub corners: Vec<(f64, f64)>,
    pub agl_m: f64,
    pub spacing_m: f64,
    pub capture_every_m: f64,
    pub speed_m_s: f64,
    pub object_query: String,
    pub min_confidence: f64,
    pub target_separation_m: f64,
    pub max_duration_s: f64,
    /// Search deadline: `max_duration_s` + the search margin (v2 sized the
    /// blocking service timeout this way; v3 arms a timer since the raster
    /// ack returns immediately and completion rides the status stream).
    pub timeout_s: f64,
}

/// The investigate order handed to an IUAS (v3 `InvestigateRequest` facts).
#[derive(Debug, Clone, PartialEq)]
pub struct InvestigateOrder {
    pub mission_id: String,
    pub source_detection_id: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub agl_m: f64,
    pub radius_m: f64,
    pub turns: f64,
    pub sensors: Vec<String>,
}

/// What the machine wants the async layer to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Broadcast this message to every dashboard client (and the recorder).
    Emit(Value),
    /// Fan out one detection request for a newly seen frame.
    Detect {
        mission_id: String,
        frame: String,
        seq: i64,
        object_query: String,
    },
    /// Send the raster-search order to the WUAS.
    StartSearch {
        vehicle: String,
        order: RasterOrder,
    },
    /// Send one investigation job to an IUAS.
    Dispatch {
        target_index: usize,
        sensor: String,
        vehicle: String,
        order: InvestigateOrder,
    },
    /// Register + broadcast a sensor-data item (map layer / playback modal).
    SensorData(Value),
}

// ───────────────────────────── state ────────────────────────────────────────

/// Pre-confirmation hit cluster (v2 `self.candidates` entries).
#[derive(Debug, Clone)]
struct Candidate {
    object_id: String,
    confidence: f64,
    lat: f64,
    lon: f64,
    frame: String,
    best_offset: f64,
    frames: BTreeSet<String>,
}

/// One investigation job (one sensor of one target).
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Job {
    pub sensor: String,
    pub vehicle: String,
    /// `queued | investigating | done | failed | cancelled`.
    pub status: String,
    pub artifacts: Vec<String>,
}

/// Job statuses that will never change again (nothing in flight, nothing
/// serviceable).
fn job_terminal(status: &str) -> bool {
    matches!(status, "done" | "failed" | "cancelled")
}

/// Target status once every job is terminal: a failure taints the target,
/// otherwise any completed job counts, otherwise everything was cancelled.
fn terminal_target_status(jobs: &[Job]) -> String {
    if jobs.iter().any(|j| j.status == "failed") {
        "failed".into()
    } else if jobs.iter().any(|j| j.status == "done") {
        "done".into()
    } else {
        "cancelled".into()
    }
}

/// A candidate left under-confirmed when the raster ended (hits <
/// confirm_count — the geometric trap: a camera footprint narrower than
/// the leg spacing can only ever see a real object on ONE pass, so
/// confirm-count 2 is unsatisfiable for it). Surfaced to the operator at
/// search end; never auto-dispatched, never blocking completion. The
/// operator either promotes it ("Investigate anyway" → the normal
/// queue/dispatch path) or dismisses it (terminal).
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Unconfirmed {
    pub index: usize,
    pub object_id: String,
    pub confidence: f64,
    pub lat: f64,
    pub lon: f64,
    pub frame: String,
    pub best_offset: f64,
    /// Distinct frames that saw it (all < `need`, or it would be a target).
    pub hits: usize,
    /// The confirm_count it fell short of.
    pub need: usize,
    /// `unconfirmed | promoted | dismissed`.
    pub status: String,
}

/// A confirmed target and its per-sensor job queue.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Target {
    pub index: usize,
    pub object_id: String,
    pub confidence: f64,
    pub lat: f64,
    pub lon: f64,
    pub frame: String,
    pub best_offset: f64,
    /// `queued | investigating | done | failed`.
    pub status: String,
    pub artifacts: Vec<String>,
    pub jobs: Vec<Job>,
}

/// Mission machine configuration.
#[derive(Clone)]
pub struct MissionConfig {
    pub wuas_id: String,
    pub iuas_ids: Vec<String>,
    /// Independent frames required before a candidate becomes a target
    /// (v2 `--confirm-count`, default 2).
    pub confirm_count: u32,
    /// Extra deadline margin over `max_duration_s` (v2 `--search-margin-s`).
    pub search_margin_s: f64,
    /// Wall clock, seconds since the Unix epoch (injected for tests).
    pub clock: Arc<dyn Fn() -> f64 + Send + Sync>,
    /// How the dispatcher ORDERS capable candidates (ROUND-3 §2). The default
    /// reproduces `pick_vehicle` exactly (idle-first, config order); the
    /// deployment folds a non-default record from `--strategy-chain` /
    /// `--strategy` and injects it via [`MissionConfig::with_strategies`].
    pub dispatch_strategy: DispatchStrategy,
    /// The re-ask backoff after an agent busy-refusal (ROUND-3 §2). The
    /// default never re-asks on a timer (today's event-driven pump only).
    pub requester_strategy: RequesterStrategy,
}

impl MissionConfig {
    /// Config with the v2 defaults, the system clock, and behavior-neutral
    /// (crate-default) service strategies.
    pub fn new(wuas_id: impl Into<String>, iuas_ids: Vec<String>) -> Self {
        Self {
            wuas_id: wuas_id.into(),
            iuas_ids,
            confirm_count: 2,
            search_margin_s: 60.0,
            clock: Arc::new(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0)
            }),
            dispatch_strategy: DispatchStrategy::default(),
            requester_strategy: RequesterStrategy::default(),
        }
    }

    /// Inject non-default dispatch/requester strategies — the strategy-load
    /// seam. The deployment resolves these from `--strategy-chain`/`--strategy`
    /// (`muas_contracts::strategy::load_active` → `dispatch()`/`requester()`)
    /// and hands them in; the crate defaults are behavior-neutral.
    pub fn with_strategies(
        mut self,
        dispatch: DispatchStrategy,
        requester: RequesterStrategy,
    ) -> Self {
        self.dispatch_strategy = dispatch;
        self.requester_strategy = requester;
        self
    }
}

/// The mission state machine. Wrap it in a mutex; every method is sync.
pub struct Mission {
    cfg: MissionConfig,
    /// `idle | searching | investigating | done | aborted`.
    pub state: String,
    pub mission_id: String,
    pub params: Value,
    pub search_done: bool,
    pub targets: Vec<Target>,
    candidates: Vec<Candidate>,
    /// End-of-raster leftovers (hits < confirm_count), operator-disposed.
    /// Indexed in its OWN namespace (`u#N` in the UI) — a promotion mints a
    /// fresh target index through the normal path.
    pub unconfirmed: Vec<Unconfirmed>,
    /// Per-vehicle enable gate: disabled = no auto-dispatch, no takeoff;
    /// RTL/Land/Hold always allowed (enforced by the command layer).
    pub enabled: HashMap<String, bool>,
    /// Investigation sensors each vehicle advertises (from its
    /// CapabilityProfile extras); absent ⇒ legacy assumption `camera`.
    pub capabilities: HashMap<String, BTreeSet<String>>,
    /// Externally observed per-vehicle busy state (from the `busy` field
    /// the telemetry poller already fetches). `pump_dispatch` skips
    /// hinted-busy vehicles; a busy→idle transition
    /// ([`Mission::set_vehicle_busy`]) completes that vehicle's in-flight
    /// jobs (the real completion signal — the investigate ACCEPT ack only
    /// marks the job in flight) and pumps the queue.
    pub vehicle_busy: HashMap<String, bool>,
    /// The additive `sensor_meta` object each vehicle advertises (hfov /
    /// DRI / audio reach — the map sensor layer renders from it); `Null`
    /// for legacy vehicles.
    pub sensor_meta: HashMap<String, Value>,
    /// Vehicles whose in-flight job was operator-aborted (`task_abort`
    /// acked); consumed at the busy→idle completion so the job's outcome
    /// note says "aborted" instead of "completed".
    aborted_vehicles: HashSet<String>,
    seen_frames: HashSet<String>,
    pub detects_pending: u64,
    pub detects_done: u64,
    /// Per-`(target_index, sensor)` count of agent busy-refusal re-asks so
    /// far — indexes the requester strategy's backoff
    /// ([`RequesterStrategy::reask`]). Empty under the default (never re-ask).
    reask_attempts: HashMap<(usize, String), u32>,
    /// Telemetry-poller hint: per-vehicle estimated remaining flight time,
    /// seconds, for the dispatch strategy's flight-time floor. Empty ⇒ the
    /// floor term is inert (behavior-neutral). Wired by the sibling poller
    /// via [`Mission::note_vehicle_flight_time`].
    vehicle_ft_est_s: HashMap<String, f64>,
}

impl Mission {
    /// A fresh idle machine; every vehicle starts enabled.
    pub fn new(cfg: MissionConfig) -> Self {
        let mut enabled = HashMap::new();
        enabled.insert(cfg.wuas_id.clone(), true);
        for vid in &cfg.iuas_ids {
            enabled.insert(vid.clone(), true);
        }
        Self {
            cfg,
            state: "idle".into(),
            mission_id: String::new(),
            params: json!({}),
            search_done: false,
            targets: Vec::new(),
            candidates: Vec::new(),
            unconfirmed: Vec::new(),
            enabled,
            capabilities: HashMap::new(),
            vehicle_busy: HashMap::new(),
            sensor_meta: HashMap::new(),
            aborted_vehicles: HashSet::new(),
            seen_frames: HashSet::new(),
            detects_pending: 0,
            detects_done: 0,
            reask_attempts: HashMap::new(),
            vehicle_ft_est_s: HashMap::new(),
        }
    }

    /// Telemetry-poller hook: record a vehicle's estimated remaining flight
    /// time, seconds — a hint the dispatch strategy's flight-time floor ranks
    /// on. Absent ⇒ unknown ⇒ inert, so this is behavior-neutral until a
    /// non-default dispatch strategy consults it. (Seam: the sibling-owned
    /// telemetry poller in `lib.rs` wires this from vehicle telemetry.)
    pub fn note_vehicle_flight_time(&mut self, vehicle: &str, flight_time_est_s: f64) {
        self.vehicle_ft_est_s.insert(vehicle.to_string(), flight_time_est_s);
    }

    /// WUAS then the IUAS list — the v2 vehicle ordering (index 0 is the
    /// searcher; binary video frames are prefixed with this index).
    pub fn vehicles(&self) -> Vec<String> {
        let mut v = vec![self.cfg.wuas_id.clone()];
        v.extend(self.cfg.iuas_ids.iter().cloned());
        v
    }

    /// The configured searcher.
    pub fn wuas_id(&self) -> &str {
        &self.cfg.wuas_id
    }

    fn now(&self) -> f64 {
        (self.cfg.clock)()
    }

    /// Build a v2-shaped event message (`{"type":"event","kind",...,"t"}`).
    fn event(&self, kind: &str, fields: Value) -> Value {
        let mut m = json!({ "type": "event", "kind": kind, "t": self.now() });
        if let (Some(dst), Some(src)) = (m.as_object_mut(), fields.as_object()) {
            for (k, v) in src {
                dst.insert(k.clone(), v.clone());
            }
        }
        m
    }

    fn param_f64(&self, key: &str, default: f64) -> f64 {
        self.params.get(key).and_then(Value::as_f64).unwrap_or(default)
    }

    fn param_str(&self, key: &str, default: &str) -> String {
        self.params
            .get(key)
            .and_then(Value::as_str)
            .unwrap_or(default)
            .to_string()
    }

    /// v2 `_mission_sensors`: requested investigation sensors, filtered to
    /// the known set, defaulting to camera.
    fn mission_sensors(&self) -> Vec<String> {
        let wanted: Vec<String> = self
            .params
            .get("investigate_sensors")
            .and_then(Value::as_array)
            .map(|a| {
                a.iter()
                    .filter_map(Value::as_str)
                    .filter(|s| *s == "camera" || *s == "audio")
                    .map(str::to_string)
                    .collect()
            })
            .unwrap_or_default();
        if wanted.is_empty() {
            vec!["camera".into()]
        } else {
            wanted
        }
    }

    /// Targets as the v2 JSON dicts (hello payload / tests).
    pub fn targets_json(&self) -> Value {
        serde_json::to_value(&self.targets).unwrap_or_else(|_| json!([]))
    }

    /// The `mission` object of the hello message.
    pub fn hello_mission(&self) -> Value {
        json!({
            "state": self.state,
            "mission_id": self.mission_id,
            "targets": self.targets_json(),
            "unconfirmed": serde_json::to_value(&self.unconfirmed)
                .unwrap_or_else(|_| json!([])),
        })
    }

    // ── operator inputs ─────────────────────────────────────────────────────

    /// v2 `start_mission`: reset, announce, fan the raster out.
    pub fn start_mission(&mut self, params: Value) -> Vec<Action> {
        if self.state == "searching" || self.state == "investigating" {
            let reason = format!("state={}", self.state);
            return vec![Action::Emit(
                self.event("mission.rejected", json!({ "reason": reason })),
            )];
        }
        let mission_id = format!("mission-{}", self.now() as i64);
        self.state = "searching".into();
        self.mission_id = mission_id.clone();
        self.params = params;
        self.search_done = false;
        self.targets.clear();
        self.candidates.clear();
        self.unconfirmed.clear();
        self.seen_frames.clear();
        self.detects_pending = 0;
        self.detects_done = 0;

        let corners =
            crate::raster::corners_for_area(self.params.get("area").unwrap_or(&Value::Null));
        let max_duration_s = self.param_f64("max_duration_s", 600.0);
        let order = RasterOrder {
            mission_id: mission_id.clone(),
            corners,
            agl_m: self.param_f64("agl_m", 6.0),
            spacing_m: self.param_f64("leg_spacing_m", 5.0),
            capture_every_m: self.param_f64("capture_every_m", 4.0),
            speed_m_s: self.param_f64("speed_m_s", 2.0),
            object_query: self.param_str("object_query", "tennis racket"),
            min_confidence: self.param_f64("min_confidence", 0.3),
            target_separation_m: self.param_f64("target_separation_m", 5.0),
            max_duration_s,
            timeout_s: max_duration_s + self.cfg.search_margin_s,
        };
        vec![
            Action::Emit(self.event(
                "mission.started",
                json!({
                    "mission_id": mission_id,
                    "vehicle": self.cfg.wuas_id,
                    "agl_m": order.agl_m,
                }),
            )),
            Action::StartSearch {
                vehicle: self.cfg.wuas_id.clone(),
                order,
            },
        ]
    }

    /// v2 `set_enabled` (dashboard-side gate). Re-enabling pumps the queue
    /// so targets that queued while the vehicle was disabled launch now.
    pub fn set_enabled(&mut self, vehicle: &str, enabled: bool) -> Vec<Action> {
        if !self.enabled.contains_key(vehicle) {
            return Vec::new();
        }
        self.enabled.insert(vehicle.to_string(), enabled);
        let kind = if enabled { "vehicle.enabled" } else { "vehicle.disabled" };
        let mut actions = vec![Action::Emit(self.event(kind, json!({ "vehicle": vehicle })))];
        if enabled {
            actions.extend(self.pump_dispatch());
        }
        actions
    }

    /// Capability advertisement changed (poller). A new capability may
    /// unblock a queued job. `sensor_meta` is the vehicle's additive
    /// sensor-metadata object (`Null` for legacy profiles); it rides the
    /// broadcast so the map sensor layer stays current.
    pub fn set_capabilities(
        &mut self,
        vehicle: &str,
        sensors: BTreeSet<String>,
        sensor_meta: Value,
    ) -> Vec<Action> {
        if self.capabilities.get(vehicle) == Some(&sensors)
            && self.sensor_meta.get(vehicle) == Some(&sensor_meta)
        {
            return Vec::new();
        }
        let msg = json!({
            "type": "capabilities",
            "vehicle": vehicle,
            "sensors": sensors.iter().cloned().collect::<Vec<_>>(),
            "sensor_meta": sensor_meta,
        });
        self.capabilities.insert(vehicle.to_string(), sensors);
        self.sensor_meta.insert(vehicle.to_string(), sensor_meta);
        let mut actions = vec![Action::Emit(msg)];
        actions.extend(self.pump_dispatch());
        actions
    }

    /// The vehicle's live busy state, as read off the telemetry the
    /// dashboard already polls (`busy` non-empty ⇒ occupied).
    ///
    /// The busy→idle transition is BOTH the queue pump ("job 2 dispatches
    /// when job 1 completes") and, since the completion fix, the job
    /// completion signal itself: the investigate ACCEPT ack only says the
    /// vehicle took the job (status stays `investigating`); the job is
    /// done when its assigned vehicle's telemetry goes idle again. The
    /// transition requires a PRIOR busy observation, so telemetry lag
    /// right after dispatch can never complete a job instantly.
    pub fn set_vehicle_busy(&mut self, vehicle: &str, busy: bool) -> Vec<Action> {
        if !self.enabled.contains_key(vehicle) {
            return Vec::new();
        }
        let previous = self.vehicle_busy.insert(vehicle.to_string(), busy);
        if previous == Some(busy) || busy {
            return Vec::new();
        }
        // Busy → idle: complete this vehicle's in-flight jobs (outcome
        // noted from the operator-abort ledger when task_abort landed
        // here, else the honest "completed (vehicle idle)")…
        let mut actions = Vec::new();
        if previous == Some(true) {
            let aborted = self.aborted_vehicles.remove(vehicle);
            let flying: Vec<(usize, String)> = self
                .targets
                .iter()
                .flat_map(|t| {
                    t.jobs
                        .iter()
                        .filter(|j| j.status == "investigating" && j.vehicle == vehicle)
                        .map(move |j| (t.index, j.sensor.clone()))
                })
                .collect();
            for (target_index, sensor) in flying {
                actions.extend(self.on_job_result(JobResult {
                    target_index,
                    sensor,
                    ok: true,
                    artifacts: Vec::new(),
                    note: if aborted {
                        "aborted (operator task_abort; vehicle idle)".into()
                    } else {
                        "completed (vehicle idle)".into()
                    },
                    artifact_items: Vec::new(),
                }));
            }
        }
        // …then a finishing IUAS takes the next serviceable job
        // (on_job_result pumps too; a second pump is a cheap no-op).
        actions.extend(self.pump_dispatch());
        actions
    }

    /// A `task_abort` for `vehicle`'s active investigation was ACCEPTED by
    /// the agent: remember it so the busy→idle completion notes the job
    /// as aborted rather than completed.
    pub fn note_task_abort(&mut self, vehicle: &str) {
        self.aborted_vehicles.insert(vehicle.to_string());
    }

    /// The investigate ACCEPT ack arrived (v3 typed-Ack: intent accepted,
    /// flight just starting). The job was already `investigating` since
    /// dispatch — record nothing terminal, just surface the acceptance;
    /// completion rides the vehicle's busy→idle transition
    /// ([`Self::set_vehicle_busy`]), with the dispatch deadline as the
    /// timeout backstop.
    pub fn on_job_accepted(&mut self, target_index: usize, sensor: &str, note: &str) -> Vec<Action> {
        let Some(target) = self.targets.iter().find(|t| t.index == target_index) else {
            return Vec::new();
        };
        let Some(job) = target.jobs.iter().find(|j| j.sensor == sensor) else {
            return Vec::new();
        };
        let payload = json!({
            "index": target_index,
            "sensor": sensor,
            "vehicle": job.vehicle,
            "note": note,
        });
        vec![Action::Emit(self.event("target.job_accepted", payload))]
    }

    /// Operator removed a QUEUED (not yet dispatched) job from the
    /// detection panel: pure state-machine cancellation — the job goes to
    /// `cancelled`, `target.job_cancelled` is emitted, and the completion
    /// predicate re-runs (a cancelled job neither flies nor blocks the
    /// mission). Non-queued jobs are ignored here: an in-flight job is
    /// cancelled by `task_abort` at its vehicle, and completion then rides
    /// the busy→idle transition.
    pub fn cancel_job(&mut self, target_index: usize, sensor: &str) -> Vec<Action> {
        let Some(ti) = self.targets.iter().position(|t| t.index == target_index) else {
            return Vec::new();
        };
        {
            let target = &mut self.targets[ti];
            let Some(job) = target
                .jobs
                .iter_mut()
                .find(|j| j.sensor == sensor && j.status == "queued")
            else {
                return Vec::new();
            };
            job.status = "cancelled".into();
            if target.jobs.iter().all(|j| job_terminal(&j.status)) {
                target.status = terminal_target_status(&target.jobs);
            }
        }
        let payload = json!({
            "index": target_index,
            "sensor": sensor,
            "lat": self.targets[ti].lat,
            "lon": self.targets[ti].lon,
        });
        let mut actions = vec![Action::Emit(self.event("target.job_cancelled", payload))];
        actions.extend(self.pump_dispatch());
        actions
    }

    /// v2 `handle_command` abort semantics: RTL/Land of the searcher during
    /// a search aborts the mission.
    pub fn note_flight_command(&mut self, vehicle: &str, command: &str) {
        if self.state == "searching"
            && vehicle == self.cfg.wuas_id
            && (command == "rtl" || command == "land")
        {
            self.state = "aborted".into();
        }
    }

    /// RTL/Land/Hold ALL aborts a live mission.
    pub fn note_all_command(&mut self) {
        if self.state == "searching" || self.state == "investigating" {
            self.state = "aborted".into();
        }
    }

    // ── search progress ─────────────────────────────────────────────────────

    /// A frame name observed in the WUAS search status. New frames fan out
    /// one detection request each; oldest-first ordering is the caller's
    /// job (v2 iterated `last_frames` reversed).
    pub fn on_new_frame(&mut self, frame: &str) -> Vec<Action> {
        if !self.seen_frames.insert(frame.to_string()) {
            return Vec::new();
        }
        let seq = frame_seq(frame);
        self.detects_pending += 1;
        vec![
            Action::Emit(self.event("detect.sent", json!({ "frame": frame, "seq": seq }))),
            Action::Detect {
                mission_id: self.mission_id.clone(),
                frame: frame.to_string(),
                seq,
                object_query: self.param_str("object_query", "tennis racket"),
            },
        ]
    }

    /// The raster finished (service response in v2; terminal search-status
    /// state or ack rejection in v3). Idempotent.
    pub fn on_search_response(
        &mut self,
        ok: bool,
        status: &str,
        frames: u64,
        error: &str,
    ) -> Vec<Action> {
        if self.search_done {
            return Vec::new();
        }
        let evt = if ok {
            self.event(
                "mission.search_finished",
                json!({ "status": status, "frames": frames }),
            )
        } else {
            self.event("mission.search_failed", json!({ "error": error }))
        };
        let mut actions = vec![Action::Emit(evt)];
        actions.extend(self.finish_search());
        actions
    }

    /// The raster never answered within its deadline. Idempotent.
    pub fn on_search_timeout(&mut self) -> Vec<Action> {
        if self.search_done {
            return Vec::new();
        }
        let mut actions = vec![Action::Emit(self.event("mission.search_timeout", json!({})))];
        actions.extend(self.finish_search());
        actions
    }

    fn finish_search(&mut self) -> Vec<Action> {
        self.search_done = true;
        let mut actions = Vec::new();
        // End-of-raster disposition: leftover candidates (hits <
        // confirm_count) become operator-facing `unconfirmed` — surfaced,
        // NOT auto-dispatched, NOT blocking completion. Mid-raster
        // candidate behavior is untouched; only a LIVE mission surfaces
        // them (an aborted one drops its candidates silently).
        if self.state == "searching" || self.state == "investigating" {
            let need = self.cfg.confirm_count.max(1) as usize;
            for cand in std::mem::take(&mut self.candidates) {
                let u = Unconfirmed {
                    index: self.unconfirmed.len(),
                    object_id: cand.object_id,
                    confidence: cand.confidence,
                    lat: cand.lat,
                    lon: cand.lon,
                    frame: cand.frame,
                    best_offset: cand.best_offset,
                    hits: cand.frames.len(),
                    need,
                    status: "unconfirmed".into(),
                };
                actions.push(Action::Emit(self.event(
                    "target.unconfirmed",
                    json!({
                        "index": u.index,
                        "hits": u.hits,
                        "need": u.need,
                        "object_id": u.object_id,
                        "confidence": round4(u.confidence),
                        "lat": u.lat,
                        "lon": u.lon,
                        "frame": u.frame,
                    }),
                )));
                self.unconfirmed.push(u);
            }
        }
        if self.state == "searching"
            && self
                .targets
                .iter()
                .any(|t| t.status == "queued" || t.status == "investigating")
        {
            self.state = "investigating".into();
        }
        // Drain (or immediately complete) the target queue.
        actions.extend(self.pump_dispatch());
        actions
    }

    // ── unconfirmed disposition (operator inputs) ───────────────────────────

    /// Operator "Investigate anyway" on an end-of-raster unconfirmed
    /// candidate: promote it through the NORMAL target path — one queued
    /// job per requested sensor, dispatched by the same pump. A completed
    /// mission reopens (`done` → `investigating`) so the completion
    /// predicate re-runs against the re-armed jobs and closes the mission
    /// again when they land. Idempotent: only an `unconfirmed` entry in a
    /// non-aborted mission promotes.
    pub fn promote_unconfirmed(&mut self, index: usize) -> Vec<Action> {
        if !matches!(self.state.as_str(), "searching" | "investigating" | "done") {
            return Vec::new();
        }
        let Some(u) = self
            .unconfirmed
            .iter_mut()
            .find(|u| u.index == index && u.status == "unconfirmed")
        else {
            return Vec::new();
        };
        u.status = "promoted".into();
        let u = u.clone();
        if self.state == "done" {
            self.state = "investigating".into();
        }
        let sensors = self.mission_sensors();
        let target = Target {
            index: self.targets.len(),
            object_id: u.object_id.clone(),
            confidence: u.confidence,
            lat: u.lat,
            lon: u.lon,
            frame: u.frame.clone(),
            best_offset: u.best_offset,
            status: "queued".into(),
            artifacts: Vec::new(),
            jobs: sensors
                .iter()
                .map(|s| Job {
                    sensor: s.clone(),
                    vehicle: String::new(),
                    status: "queued".into(),
                    artifacts: Vec::new(),
                })
                .collect(),
        };
        let mut actions = vec![
            Action::Emit(self.event(
                "target.promoted",
                json!({
                    "index": u.index,
                    "target_index": target.index,
                    "lat": u.lat,
                    "lon": u.lon,
                }),
            )),
            // The same wire shape as a confirm-count promotion, plus the
            // provenance key (`hits` is the honest count, below `need`).
            Action::Emit(self.event(
                "mission.target_found",
                json!({
                    "index": target.index,
                    "object_id": target.object_id,
                    "confidence": round4(target.confidence),
                    "lat": target.lat,
                    "lon": target.lon,
                    "frame": target.frame,
                    "hits": u.hits,
                    "sensors": sensors,
                    "promoted_from": u.index,
                }),
            )),
        ];
        self.targets.push(target);
        actions.extend(self.pump_dispatch());
        actions
    }

    /// Operator "Dismiss" on an unconfirmed candidate: terminal — it can
    /// no longer be promoted, and nothing else ever touches it.
    pub fn dismiss_unconfirmed(&mut self, index: usize) -> Vec<Action> {
        let Some(u) = self
            .unconfirmed
            .iter_mut()
            .find(|u| u.index == index && u.status == "unconfirmed")
        else {
            return Vec::new();
        };
        u.status = "dismissed".into();
        let payload = json!({ "index": u.index, "lat": u.lat, "lon": u.lon });
        vec![Action::Emit(self.event("target.dismissed", payload))]
    }

    // ── detection fan-in ────────────────────────────────────────────────────

    /// A detection request completed (hit / miss / timeout).
    pub fn on_detect_outcome(&mut self, frame: &str, outcome: DetectOutcome) -> Vec<Action> {
        self.detects_pending = self.detects_pending.saturating_sub(1);
        self.detects_done += 1;
        let seq = frame_seq(frame);
        match outcome {
            DetectOutcome::Miss(error) => vec![Action::Emit(self.event(
                "detect.miss",
                json!({ "frame": frame, "seq": seq, "error": error }),
            ))],
            DetectOutcome::Timeout => vec![Action::Emit(
                self.event("detect.timeout", json!({ "frame": frame, "seq": seq })),
            )],
            DetectOutcome::Hit(d) => {
                let mut actions = vec![Action::Emit(self.event(
                    "detect.hit",
                    json!({
                        "frame": frame,
                        "seq": seq,
                        "object_id": d.object_id,
                        "confidence": round4(d.confidence),
                        "lat": d.lat_deg,
                        "lon": d.lon_deg,
                        "offset_m": round2(d.offset_m),
                    }),
                ))];
                let min_conf = self.param_f64("min_confidence", 0.3);
                if d.confidence >= min_conf
                    && (self.state == "searching" || self.state == "investigating")
                {
                    actions.extend(self.on_detect_hit(&d, frame));
                }
                actions
            }
        }
    }

    /// v2 `_on_detect_hit`: confirm-then-queue with ground-distance dedup.
    fn on_detect_hit(&mut self, d: &Detection, frame: &str) -> Vec<Action> {
        let sep = self.param_f64("target_separation_m", 5.0);
        let need = self.cfg.confirm_count.max(1) as usize;
        let (lat, lon) = (d.lat_deg, d.lon_deg);

        // Already a confirmed target nearby? Absorb + maybe refine.
        for i in 0..self.targets.len() {
            if dist_m(self.targets[i].lat, self.targets[i].lon, lat, lon) <= sep {
                self.targets[i].confidence = self.targets[i].confidence.max(d.confidence);
                // Refine position only from a BETTER-localized sighting,
                // and only while not yet flown.
                if self.targets[i].status == "queued"
                    && d.offset_m < self.targets[i].best_offset
                {
                    self.targets[i].best_offset = d.offset_m;
                    self.targets[i].lat = lat;
                    self.targets[i].lon = lon;
                    self.targets[i].frame = frame.to_string();
                    let payload = json!({
                        "index": self.targets[i].index,
                        "confidence": round4(self.targets[i].confidence),
                        "lat": self.targets[i].lat,
                        "lon": self.targets[i].lon,
                        "frame": frame,
                        "best_offset_m": round2(self.targets[i].best_offset),
                    });
                    return vec![Action::Emit(self.event("target.updated", payload))];
                }
                return Vec::new();
            }
        }

        // Otherwise reinforce / create a candidate.
        let ci = match self
            .candidates
            .iter()
            .position(|c| dist_m(c.lat, c.lon, lat, lon) <= sep)
        {
            Some(i) => i,
            None => {
                self.candidates.push(Candidate {
                    object_id: d.object_id.clone(),
                    confidence: d.confidence,
                    lat,
                    lon,
                    frame: frame.to_string(),
                    best_offset: d.offset_m,
                    frames: BTreeSet::new(),
                });
                self.candidates.len() - 1
            }
        };
        let cand = &mut self.candidates[ci];
        cand.frames.insert(frame.to_string());
        let hits = cand.frames.len();
        cand.confidence = cand.confidence.max(d.confidence);
        // POSITION comes from the best-localized sighting, not the highest
        // confidence (the v2 field fix: the fix snaps to the pass where the
        // object sat directly underneath).
        if d.offset_m < cand.best_offset {
            cand.best_offset = d.offset_m;
            cand.lat = lat;
            cand.lon = lon;
            cand.frame = frame.to_string();
        }
        let cand_snapshot = json!({
            "object_id": cand.object_id,
            "hits": hits,
            "need": need,
            "confidence": round4(cand.confidence),
            "lat": cand.lat,
            "lon": cand.lon,
            "best_offset_m": round2(cand.best_offset),
        });
        let mut actions = vec![Action::Emit(self.event("detect.candidate", cand_snapshot))];
        if hits < need {
            return actions;
        }

        // Promote candidate → target: one investigation job per requested
        // sensor, each dispatched to an IUAS advertising that sensor.
        let cand = self.candidates.remove(ci);
        let sensors = self.mission_sensors();
        let target = Target {
            index: self.targets.len(),
            object_id: cand.object_id,
            confidence: cand.confidence,
            lat: cand.lat,
            lon: cand.lon,
            frame: cand.frame,
            best_offset: cand.best_offset,
            status: "queued".into(),
            artifacts: Vec::new(),
            jobs: sensors
                .iter()
                .map(|s| Job {
                    sensor: s.clone(),
                    vehicle: String::new(),
                    status: "queued".into(),
                    artifacts: Vec::new(),
                })
                .collect(),
        };
        let found = json!({
            "index": target.index,
            "object_id": target.object_id,
            "confidence": round4(target.confidence),
            "lat": target.lat,
            "lon": target.lon,
            "frame": target.frame,
            "hits": need,
            "sensors": sensors,
        });
        self.targets.push(target);
        actions.push(Action::Emit(self.event("mission.target_found", found)));
        actions.extend(self.pump_dispatch());
        actions
    }

    // ── dispatch machinery ──────────────────────────────────────────────────

    /// Pick a vehicle for one `sensor` job. Hard eligibility (enabled +
    /// capability match + not-busy) is the caller's filter — never strategy
    /// material; ORDER among the eligible is the dispatch strategy's
    /// ([`rank_candidates`], ROUND-3 §2), and we take the first.
    ///
    /// The default strategy (idle-first, config order) reproduces the v2
    /// `_pick_vehicle_locked` first-match scan exactly: every eligible
    /// candidate here is idle (busy vehicles stay filtered out — the
    /// dashboard does not queue jobs on remote vehicles, so a job with no
    /// idle taker waits for a busy→idle transition), so idle-first is a no-op
    /// and the input (config) order carries through. Lifting the busy
    /// exclusion into the ranking's idle-first term is a follow that needs the
    /// pump/completion path to handle remote queueing — flagged in the round
    /// report.
    fn pick_vehicle(&self, sensor: &str, busy: &HashSet<String>) -> Option<String> {
        let default_caps: BTreeSet<String> = ["camera".to_string()].into();
        let mut eligible: Vec<String> = Vec::new();
        let mut snapshots: Vec<CandidateSnapshot> = Vec::new();
        for vid in &self.cfg.iuas_ids {
            if busy.contains(vid) || !self.enabled.get(vid).copied().unwrap_or(true) {
                continue;
            }
            let caps = self.capabilities.get(vid).unwrap_or(&default_caps);
            if !caps.contains(sensor) {
                continue;
            }
            eligible.push(vid.clone());
            snapshots.push(CandidateSnapshot {
                // Every eligible candidate cleared the busy gate above.
                idle: true,
                // Idle vehicles carry no dashboard-side pending queue.
                queued: 0,
                // Telemetry-fed hint (empty ⇒ None ⇒ the floor term is inert).
                flight_time_est_s: self.vehicle_ft_est_s.get(vid).copied(),
                // Per-vehicle distance needs position telemetry the pure
                // machine does not carry yet ⇒ the `nearest` term is inert.
                distance_m: None,
            });
        }
        rank_candidates(&self.cfg.dispatch_strategy, &snapshots)
            .first()
            .map(|&i| eligible[i].clone())
    }

    /// v2 `_pump_dispatch`: assign queued jobs to idle, enabled,
    /// capability-matching IUAS; when nothing dispatches, check completion.
    fn pump_dispatch(&mut self) -> Vec<Action> {
        if self.state != "searching" && self.state != "investigating" {
            return Vec::new(); // operator aborted: stop draining the queue
        }
        let mut busy: HashSet<String> = self
            .targets
            .iter()
            .flat_map(|t| t.jobs.iter())
            .filter(|j| j.status == "investigating")
            .map(|j| j.vehicle.clone())
            .collect();
        // ROUND-3: a vehicle whose TELEMETRY says it is occupied is busy no
        // matter what the job table thinks (the accept-ack completes jobs
        // early; see `vehicle_busy`). This is what routes a second target
        // to the idle capable IUAS instead of the one still flying.
        busy.extend(
            self.vehicle_busy
                .iter()
                .filter(|(_, is_busy)| **is_busy)
                .map(|(vid, _)| vid.clone()),
        );
        let mut to_dispatch: Vec<(usize, usize, String)> = Vec::new();
        for ti in 0..self.targets.len() {
            for ji in 0..self.targets[ti].jobs.len() {
                if self.targets[ti].jobs[ji].status != "queued" {
                    continue;
                }
                let Some(vid) = self.pick_vehicle(&self.targets[ti].jobs[ji].sensor, &busy)
                else {
                    continue;
                };
                self.targets[ti].jobs[ji].status = "investigating".into();
                self.targets[ti].jobs[ji].vehicle = vid.clone();
                busy.insert(vid.clone());
                self.targets[ti].status = "investigating".into();
                to_dispatch.push((ti, ji, vid));
            }
        }
        if to_dispatch.is_empty() {
            return self.maybe_complete();
        }
        let mut actions = Vec::new();
        for (ti, ji, vid) in to_dispatch {
            let (index, object_id, lat, lon) = {
                let t = &self.targets[ti];
                (t.index, t.object_id.clone(), t.lat, t.lon)
            };
            let sensor = self.targets[ti].jobs[ji].sensor.clone();
            let order = InvestigateOrder {
                mission_id: self.mission_id.clone(),
                source_detection_id: format!("{object_id}-{index}-{sensor}"),
                lat_deg: lat,
                lon_deg: lon,
                agl_m: self.param_f64("orbit_agl_m", 8.0),
                radius_m: self.param_f64("orbit_radius_m", 6.0),
                turns: self.param_f64("orbit_count", 1.0),
                sensors: vec![sensor.clone()],
            };
            let dispatch_evt = json!({
                "index": index,
                "sensor": sensor,
                "vehicle": vid,
                "lat": order.lat_deg,
                "lon": order.lon_deg,
                "radius_m": order.radius_m,
                "agl_m": order.agl_m,
            });
            actions.push(Action::Emit(self.event("target.dispatch", dispatch_evt)));
            actions.push(Action::Dispatch {
                target_index: index,
                sensor,
                vehicle: vid,
                order,
            });
        }
        actions
    }

    /// v2 `_maybe_complete_locked`: the completion predicate. The mission
    /// ends when the raster is done, nothing is in flight, and no queued
    /// job could ever be served by a currently-enabled vehicle.
    fn maybe_complete(&mut self) -> Vec<Action> {
        if !self.search_done {
            return Vec::new();
        }
        if self.state != "searching" && self.state != "investigating" {
            return Vec::new();
        }
        let jobs: Vec<(String, String)> = self
            .targets
            .iter()
            .flat_map(|t| t.jobs.iter())
            .map(|j| (j.status.clone(), j.sensor.clone()))
            .collect();
        if jobs.iter().any(|(s, _)| s == "investigating") {
            self.state = "investigating".into();
            return Vec::new();
        }
        let serviceable = jobs
            .iter()
            .filter(|(s, sensor)| {
                s == "queued" && self.pick_vehicle(sensor, &HashSet::new()).is_some()
            })
            .count();
        if serviceable > 0 {
            self.state = "investigating".into();
            return Vec::new();
        }
        let unserved = jobs.iter().filter(|(s, _)| s == "queued").count();
        let note = if unserved > 0 {
            format!("unserviceable-jobs:{unserved}")
        } else {
            String::new()
        };
        self.complete(&note)
    }

    /// v2 `_complete_locked`.
    fn complete(&mut self, note: &str) -> Vec<Action> {
        if self.state != "searching" && self.state != "investigating" {
            return Vec::new();
        }
        self.state = "done".into();
        let investigated = self.targets.iter().filter(|t| t.status == "done").count();
        vec![Action::Emit(self.event(
            "mission.completed",
            json!({
                "targets": self.targets.len(),
                "investigated": investigated,
                "note": note,
            }),
        ))]
    }

    /// An investigation job finished (v2 `_dispatch_iuas`'s `finish`).
    pub fn on_job_result(&mut self, result: JobResult) -> Vec<Action> {
        let Some(ti) = self
            .targets
            .iter()
            .position(|t| t.index == result.target_index)
        else {
            return Vec::new();
        };
        // ROUND-3 (SURGICAL): an agent busy-refusal is a scheduling race,
        // not a job failure — the 2026-07-10 eval journals show target 2's
        // jobs dispatched at the same vehicles still flying target 1
        // ("vehicle busy with task 'investigate'") and then terminally
        // marked failed, so only the first target was ever investigated.
        // Requeue the job, remember the vehicle as busy, and pump so an
        // idle capable vehicle can take it now (or the busy one when its
        // telemetry goes idle). Detection keys on the pinned
        // `PolicyRejection::Busy` detail text because `JobResult` carries
        // no ack code (adding one means touching `lib.rs`, which is the
        // dashboard owner's file — flagged in the round report).
        if !result.ok && result.note.starts_with("vehicle busy") {
            let refused_by = self.targets[ti]
                .jobs
                .iter_mut()
                .find(|j| j.sensor == result.sensor)
                .map(|job| {
                    job.status = "queued".into();
                    std::mem::take(&mut job.vehicle)
                });
            let Some(refused_by) = refused_by else {
                return Vec::new();
            };
            self.vehicle_busy.insert(refused_by.clone(), true);
            // Re-ask backoff (ROUND-3 §2 requester strategy). The default
            // requester never re-asks on a timer (`max_attempts` 0 ⇒ `None`),
            // so this stays exactly today's behavior: requeue + an immediate
            // event-driven pump. A non-default requester's delay rides the
            // event; running the TIMED re-pump is the async executor's follow
            // (an Action it can schedule) — the pure machine only computes the
            // schedule and surfaces it. Flagged in the round report.
            let attempt = {
                let counter = self
                    .reask_attempts
                    .entry((result.target_index, result.sensor.clone()))
                    .or_insert(0);
                *counter += 1;
                *counter
            };
            let reask_delay_s = reask_schedule(&self.cfg.requester_strategy, attempt);
            let payload = json!({
                "index": result.target_index,
                "sensor": result.sensor,
                "vehicle": refused_by,
                "note": result.note,
                "attempt": attempt,
                "reask_delay_s": reask_delay_s,
            });
            let mut actions = vec![Action::Emit(self.event("target.job_requeued", payload))];
            actions.extend(self.pump_dispatch());
            return actions;
        }
        let status = if result.ok { "done" } else { "failed" };
        {
            let target = &mut self.targets[ti];
            if let Some(job) = target.jobs.iter_mut().find(|j| j.sensor == result.sensor) {
                job.status = status.into();
                job.artifacts = result.artifacts.clone();
            }
            target.artifacts = target
                .jobs
                .iter()
                .flat_map(|j| j.artifacts.iter().cloned())
                .collect();
            if target.jobs.iter().all(|j| job_terminal(&j.status)) {
                target.status = terminal_target_status(&target.jobs);
            }
        }
        let target = self.targets[ti].clone();
        let terminal = target.status == "done" || target.status == "failed";
        let vehicle = target
            .jobs
            .iter()
            .find(|j| j.sensor == result.sensor)
            .map(|j| j.vehicle.clone())
            .unwrap_or_default();

        let mut actions = vec![Action::Emit(self.event(
            if result.ok {
                "target.job_completed"
            } else {
                "target.job_failed"
            },
            json!({
                "index": target.index,
                "sensor": result.sensor,
                "vehicle": vehicle,
                "artifacts": result.artifacts,
                "note": result.note,
                "lat": target.lat,
                "lon": target.lon,
            }),
        ))];
        if terminal {
            actions.push(Action::Emit(self.event(
                if target.status == "done" {
                    "target.completed"
                } else {
                    "target.failed"
                },
                json!({
                    "index": target.index,
                    "artifacts": target.artifacts,
                    "note": result.note,
                    "lat": target.lat,
                    "lon": target.lon,
                }),
            )));
        }
        // Mission evidence joins the sensor-data layer, pinned at the
        // capture pose the artifact itself carries.
        for (kind, name, lat, lon) in &result.artifact_items {
            actions.push(Action::SensorData(json!({
                "vehicle": vehicle,
                "sensor": result.sensor,
                "kind": kind,
                "name": name,
                "lat": lat,
                "lon": lon,
                "t": self.now(),
                "source": "mission",
                "label": format!(
                    "target #{} {} ({:.0}%)",
                    target.index,
                    target.object_id,
                    target.confidence * 100.0
                ),
            })));
        }
        actions.extend(self.pump_dispatch());
        actions
    }
}

/// Capture sequence number from a frame name (`.../frame/<ts>/<seq>`),
/// `-1` when unparseable (v2 `_frame_seq`).
pub fn frame_seq(frame: &str) -> i64 {
    frame
        .rsplit('/')
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(-1)
}

// ───────────────────────── strategy-driven dispatch (ROUND-3 §2) ─────────────

#[cfg(test)]
mod strategy_tests {
    use super::*;
    use muas_contracts::strategy::{RankTerm, ReaskPolicy};

    fn cfg(iuas: &[&str]) -> MissionConfig {
        let mut c = MissionConfig::new("wuas-01", iuas.iter().map(|s| s.to_string()).collect());
        c.confirm_count = 1;
        c.clock = Arc::new(|| 1000.0);
        c
    }

    /// Default dispatch strategy = today's `pick_vehicle`: idle-first with the
    /// config vehicle-id order as the tie break.
    #[test]
    fn default_dispatch_strategy_reproduces_config_order() {
        let m = Mission::new(cfg(&["iuas-01", "iuas-02"]));
        assert_eq!(m.pick_vehicle("camera", &HashSet::new()), Some("iuas-01".into()));
    }

    /// A flight-time-floor + idle-first strategy swaps the pick when the
    /// telemetry hints say the config-first vehicle is below the floor.
    #[test]
    fn flight_time_floor_strategy_swaps_dispatch_order() {
        let dispatch = DispatchStrategy {
            ranking: vec![RankTerm::flight_time_floor_s(300.0), RankTerm::idle_first()],
            ..DispatchStrategy::default()
        };
        let mut m = Mission::new(
            cfg(&["iuas-01", "iuas-02"]).with_strategies(dispatch, RequesterStrategy::default()),
        );
        m.note_vehicle_flight_time("iuas-01", 120.0); // below the 300 s floor
        m.note_vehicle_flight_time("iuas-02", 600.0); // clears it
        assert_eq!(
            m.pick_vehicle("camera", &HashSet::new()),
            Some("iuas-02".into()),
            "flight-time floor ranks the healthy IUAS ahead of config order"
        );

        // The SAME hints under the default strategy keep config order.
        let mut d = Mission::new(cfg(&["iuas-01", "iuas-02"]));
        d.note_vehicle_flight_time("iuas-01", 120.0);
        d.note_vehicle_flight_time("iuas-02", 600.0);
        assert_eq!(d.pick_vehicle("camera", &HashSet::new()), Some("iuas-01".into()));
    }

    fn investigating_target(m: &mut Mission) {
        m.state = "investigating".into();
        m.targets.push(Target {
            index: 0,
            object_id: "x".into(),
            confidence: 0.9,
            lat: 35.0,
            lon: -90.0,
            frame: "f".into(),
            best_offset: 1.0,
            status: "investigating".into(),
            artifacts: vec![],
            jobs: vec![Job {
                sensor: "camera".into(),
                vehicle: "iuas-01".into(),
                status: "investigating".into(),
                artifacts: vec![],
            }],
        });
    }

    fn busy_refusal(m: &mut Mission) -> Vec<Action> {
        m.on_job_result(JobResult {
            target_index: 0,
            sensor: "camera".into(),
            ok: false,
            artifacts: vec![],
            note: "vehicle busy with task 'investigate'".into(),
            artifact_items: vec![],
        })
    }

    fn requeued(actions: &[Action]) -> Value {
        actions
            .iter()
            .find_map(|a| match a {
                Action::Emit(v) if v.get("kind") == Some(&json!("target.job_requeued")) => {
                    Some(v.clone())
                }
                _ => None,
            })
            .expect("target.job_requeued emitted")
    }

    /// The requester strategy's exact backoff rides the requeue events on
    /// successive agent busy-refusals (5 s, then 10 s).
    #[test]
    fn reask_schedule_drives_requeue_backoff() {
        let requester = RequesterStrategy {
            reask: ReaskPolicy {
                backoff_initial_s: 5.0,
                give_up_horizon_s: 120.0,
                max_attempts: 5,
                multiplier: 2.0,
            },
            ..RequesterStrategy::default()
        };
        let mut m = Mission::new(
            cfg(&["iuas-01"]).with_strategies(DispatchStrategy::default(), requester),
        );
        investigating_target(&mut m);
        let evt = requeued(&busy_refusal(&mut m));
        assert_eq!(evt["attempt"], json!(1));
        assert_eq!(evt["reask_delay_s"], json!(5.0));

        // Re-arm the same job and refuse again → attempt 2, 10 s (× multiplier).
        m.targets[0].jobs[0].status = "investigating".into();
        m.targets[0].jobs[0].vehicle = "iuas-01".into();
        m.vehicle_busy.clear();
        let evt = requeued(&busy_refusal(&mut m));
        assert_eq!(evt["attempt"], json!(2));
        assert_eq!(evt["reask_delay_s"], json!(10.0));
    }

    /// Behavior-neutral: the default requester never re-asks on a timer, so
    /// the requeue carries a null delay and the job requeues exactly as today.
    #[test]
    fn default_requester_reask_is_none_behavior_neutral() {
        let mut m = Mission::new(cfg(&["iuas-01"]));
        investigating_target(&mut m);
        let evt = requeued(&busy_refusal(&mut m));
        assert_eq!(evt["reask_delay_s"], Value::Null);
        assert_eq!(evt["attempt"], json!(1));
        assert_eq!(m.targets[0].jobs[0].status, "queued", "requeued as before");
    }
}
