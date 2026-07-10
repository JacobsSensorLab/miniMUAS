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
    /// `queued | investigating | done | failed`.
    pub status: String,
    pub artifacts: Vec<String>,
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
}

impl MissionConfig {
    /// Config with the v2 defaults and the system clock.
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
        }
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
    /// Per-vehicle enable gate: disabled = no auto-dispatch, no takeoff;
    /// RTL/Land/Hold always allowed (enforced by the command layer).
    pub enabled: HashMap<String, bool>,
    /// Investigation sensors each vehicle advertises (from its
    /// CapabilityProfile extras); absent ⇒ legacy assumption `camera`.
    pub capabilities: HashMap<String, BTreeSet<String>>,
    /// The additive `sensor_meta` object each vehicle advertises (hfov /
    /// DRI / audio reach — the map sensor layer renders from it); `Null`
    /// for legacy vehicles.
    pub sensor_meta: HashMap<String, Value>,
    seen_frames: HashSet<String>,
    pub detects_pending: u64,
    pub detects_done: u64,
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
            enabled,
            capabilities: HashMap::new(),
            sensor_meta: HashMap::new(),
            seen_frames: HashSet::new(),
            detects_pending: 0,
            detects_done: 0,
        }
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
        if self.state == "searching"
            && self
                .targets
                .iter()
                .any(|t| t.status == "queued" || t.status == "investigating")
        {
            self.state = "investigating".into();
        }
        // Drain (or immediately complete) the target queue.
        self.pump_dispatch()
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

    /// v2 `_pick_vehicle_locked`: first idle, enabled IUAS advertising
    /// `sensor`; `None` if none.
    fn pick_vehicle(&self, sensor: &str, busy: &HashSet<String>) -> Option<String> {
        let default_caps: BTreeSet<String> = ["camera".to_string()].into();
        for vid in &self.cfg.iuas_ids {
            if busy.contains(vid) || !self.enabled.get(vid).copied().unwrap_or(true) {
                continue;
            }
            let caps = self.capabilities.get(vid).unwrap_or(&default_caps);
            if caps.contains(sensor) {
                return Some(vid.clone());
            }
        }
        None
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
            let terminal = target
                .jobs
                .iter()
                .all(|j| j.status == "done" || j.status == "failed");
            if terminal {
                target.status = if target.jobs.iter().all(|j| j.status == "done") {
                    "done".into()
                } else {
                    "failed".into()
                };
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
