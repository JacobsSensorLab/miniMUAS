//! miniMUAS v3 GCS dashboard: web UI backend + mission orchestrator.
//!
//! One process, three jobs (the v2 shape, ported):
//!
//! 1. **Web server** (axum) at `http://0.0.0.0:8080` serving the embedded
//!    single-page canvas UI and one WebSocket carrying everything:
//!    telemetry, search status, events, detections, video frames (binary),
//!    and operator commands — the v2 JSON message schema, kept.
//! 2. **NDN consumer** (`/muas/v3` engine + UDP faces, the muas-agent
//!    bring-up pattern): latest-wins pollers with MustBeFresh, and every
//!    service request through the typed `VehicleService` client over
//!    `FaceRpcCarrier`.
//! 3. **Mission state machine** — detect→confirm→queue→dispatch, the brain
//!    the agents deliberately don't have ([`mission`]).
//!
//! Recording: session-scoped power-loss-safe JSONL via uas-console's
//! `Recorder` — armed at mission start (or the explicit Record button),
//! finalized at mission end / RTL-all / explicit stop, named
//! `<run>-<mission>-<t>.jsonl`; an idle dashboard records nothing.
//! Recordings are derived UI artifacts (the broadcast stream the operator
//! saw); the per-vehicle journal chains remain the durable truth.
//! `/replays` serves the index and the files, and replay is frontend-driven
//! through the same `dispatch()` handlers.

pub mod catalog;
pub mod config;
pub mod detect;
pub mod hub;
pub mod lens;
pub mod mission;
pub mod ndn;
pub mod providers;
pub mod raster;
pub mod rc;
pub mod server;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

use ndn_app::EngineAppExt;
use ndn_engine::{ForwarderEngine, ShutdownHandle};
use serde_json::{json, Value};
use tokio_util::sync::CancellationToken;

pub use config::{DashConfig, ParseOutcome, UdpLink, HELP};

/// CLI convenience: parsed config, `Ok(None)` for `--help`.
pub fn parse_outcome(args: &[String]) -> Result<Option<DashConfig>, String> {
    match config::parse_args(args)? {
        ParseOutcome::Run(config) => Ok(Some(*config)),
        ParseOutcome::Help => Ok(None),
    }
}
use mission::{Action, DetectOutcome, JobResult, Mission, MissionConfig};
use providers::{CmdResult, Commander, DetectionProvider, SimControl};

/// Lock a mutex, recovering from poisoning (v2's "failures never kill the
/// process" posture, same as muas-agent).
pub(crate) fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// Wall clock, seconds since the Unix epoch (v2 `time.time()`).
fn now_s() -> f64 {
    hub::now_ns() as f64 / 1e9
}

/// Parse the pilot surface's WS target selector: `"broadcast"` → every
/// RC-reachable vehicle; anything else → that single vehicle id.
fn ws_rc_target(message: &Value) -> uas_rc::RcTarget {
    match message.get("target").and_then(Value::as_str) {
        Some("broadcast") | Some("Broadcast") => uas_rc::RcTarget::Broadcast,
        Some(vid) => uas_rc::RcTarget::One(vid.to_string()),
        None => uas_rc::RcTarget::Broadcast,
    }
}

/// Everything the web layer, the pollers, and the action executor share.
pub struct Dashboard {
    pub config: DashConfig,
    pub hub: hub::Hub,
    mission: Mutex<Mission>,
    /// Last decoded telemetry per vehicle (armed guard for shutdown).
    last_sample: Mutex<HashMap<String, Value>>,
    /// Everything captured this session (map layer + playback modal).
    sensor_data: Mutex<Vec<Value>>,
    /// Latest task-queue snapshot per vehicle (tile queue strip; rides the
    /// hello message so a fresh client renders the strip immediately).
    task_queues: Mutex<HashMap<String, Value>>,
    /// Latest rc/status snapshot per vehicle (RC-CONTROL R2 pilot strip +
    /// map manual ring; rides hello so a fresh client renders immediately).
    rc_status: Mutex<HashMap<String, Value>>,
    /// The RC pilot-surface send host — present only when `--rc-target`
    /// wired at least one reachable vehicle (else the Pilot surface is inert).
    rc: OnceLock<Arc<rc::RcHost>>,
    /// Per-vehicle video relay enable flags.
    video_flags: Mutex<HashMap<String, Arc<AtomicBool>>>,
    pub detector: Arc<dyn DetectionProvider>,
    pub commander: Arc<dyn Commander>,
    /// uas-console binding dogfood (track + tile lenses, /instrument.json).
    pub lens: lens::LensHost,
    /// Engine handle for on-demand consumers (artifacts, video relays);
    /// absent in pure-logic tests.
    engine: OnceLock<(ForwarderEngine, CancellationToken)>,
    /// Simulation attachment (virtual deployments only): the capability
    /// info the hello message advertises + the control seam WS `sim`
    /// commands forward through. Never set in a real deployment.
    sim: OnceLock<(Value, Arc<dyn SimControl>)>,
}

impl Dashboard {
    /// Assemble the shared state (no sockets yet — see [`start`]).
    pub fn new(
        config: DashConfig,
        detector: Arc<dyn DetectionProvider>,
        commander: Arc<dyn Commander>,
    ) -> Self {
        let mut mission_cfg =
            MissionConfig::new(config.wuas_id.clone(), config.iuas_ids.clone());
        mission_cfg.confirm_count = config.confirm_count;
        mission_cfg.search_margin_s = config.search_margin_s;
        // Fold the active strategy (dispatch ranking + requester backoff)
        // when a source is configured; absent = behavior-neutral defaults.
        if let Some(source) = &config.strategy {
            match muas_contracts::strategy::load_active(Some(source)) {
                Ok(active) => {
                    mission_cfg = mission_cfg.with_strategies(active.dispatch(), active.requester());
                    tracing::info!("dashboard dispatch strategy loaded");
                }
                Err(e) => tracing::warn!(%e, "strategy load failed; using defaults"),
            }
        }
        let vehicles = config.vehicles();
        Self {
            hub: hub::Hub::new(config.record_dir.clone(), &config.run_name),
            mission: Mutex::new(Mission::new(mission_cfg)),
            last_sample: Mutex::new(HashMap::new()),
            sensor_data: Mutex::new(Vec::new()),
            task_queues: Mutex::new(HashMap::new()),
            rc_status: Mutex::new(HashMap::new()),
            rc: OnceLock::new(),
            video_flags: Mutex::new(HashMap::new()),
            detector,
            commander,
            lens: lens::LensHost::new(&vehicles),
            engine: OnceLock::new(),
            sim: OnceLock::new(),
            config,
        }
    }

    /// Attach the engine (enables artifact fetches + video relays).
    pub fn attach_engine(&self, engine: ForwarderEngine, cancel: CancellationToken) {
        let _ = self.engine.set((engine, cancel));
    }

    /// Attach a simulation deployment: `info` is advertised as the hello
    /// message's `sim` capability object (e.g. `{"anomalies": true, ...}` —
    /// gates the UI's anomaly-placement tool); `control` executes the
    /// forwarded `sim` WS commands against the deployment's control
    /// endpoint. Idempotent; first attach wins.
    pub fn attach_sim(&self, info: Value, control: Arc<dyn SimControl>) {
        let _ = self.sim.set((info, control));
    }

    /// Attach the RC pilot-surface send host (RC-CONTROL R2). Idempotent;
    /// first attach wins. Absent when no `--rc-target` was configured.
    pub fn attach_rc(&self, host: Arc<rc::RcHost>) {
        let _ = self.rc.set(host);
    }

    /// The RC pilot host, if the surface is configured.
    pub fn rc_host(&self) -> Option<&Arc<rc::RcHost>> {
        self.rc.get()
    }

    /// A fresh app consumer over the attached engine.
    pub fn consumer(&self) -> Option<ndn_app::Consumer> {
        self.engine
            .get()
            .map(|(engine, cancel)| engine.app_consumer(cancel.child_token()))
    }

    /// The vehicle list, WUAS first (wire ordering).
    pub fn vehicles(&self) -> Vec<String> {
        self.config.vehicles()
    }

    /// The searcher id.
    pub fn wuas_id(&self) -> String {
        self.config.wuas_id.clone()
    }

    /// Run a closure under the mission lock.
    pub fn with_mission<R>(&self, f: impl FnOnce(&mut Mission) -> R) -> R {
        f(&mut lock(&self.mission))
    }

    /// Current mission state string.
    pub fn mission_state(&self) -> String {
        lock(&self.mission).state.clone()
    }

    /// `(detects_pending, detects_done)` for the search-status banner.
    pub fn detect_counters(&self) -> (u64, u64) {
        let m = lock(&self.mission);
        (m.detects_pending, m.detects_done)
    }

    pub(crate) fn set_last_sample(&self, vehicle: &str, sample: Value) {
        lock(&self.last_sample).insert(vehicle.to_string(), sample);
    }

    /// The v2 hello message a new WS client receives.
    pub fn hello(&self) -> Value {
        let m = lock(&self.mission);
        let enabled: serde_json::Map<String, Value> = m
            .enabled
            .iter()
            .map(|(k, v)| (k.clone(), json!(v)))
            .collect();
        let capabilities: serde_json::Map<String, Value> = m
            .capabilities
            .iter()
            .map(|(k, v)| (k.clone(), json!(v.iter().cloned().collect::<Vec<_>>())))
            .collect();
        let sensor_meta: serde_json::Map<String, Value> =
            m.sensor_meta.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        let task_queues: serde_json::Map<String, Value> = lock(&self.task_queues)
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        let rc_status: serde_json::Map<String, Value> = lock(&self.rc_status)
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        // Which vehicles the Pilot surface can drive (RC-reachable targets);
        // empty = the surface renders read-only (status strip only).
        let rc_targets: Vec<String> = self.rc.get().map(|h| h.vehicles()).unwrap_or_default();
        let mut hello = json!({
            "type": "hello",
            "vehicles": self.vehicles(),
            "enabled": enabled,
            "capabilities": capabilities,
            "sensor_meta": sensor_meta,
            "sensor_data": *lock(&self.sensor_data),
            "task_queues": task_queues,
            "rc": rc_status,
            "rc_targets": rc_targets,
            "mission": m.hello_mission(),
            "recording": self.hub.is_recording(),
            // Where this surface describes ITSELF (ROUND-3 §3½): render
            // contracts, understood kinds, native widgets.
            "catalog_url": "/catalog.json",
        });
        // Surveyed GCS position (--gcs): the map's network layer prefers
        // this over the NET.gcs export and the first-fix heuristic.
        if let Some((lat, lon)) = self.config.gcs {
            hello["gcs"] = json!({ "lat": lat, "lon": lon, "source": "manual" });
        }
        // Sim capability flag: present ONLY when a virtual deployment
        // attached itself — gates the UI's anomaly-placement tool.
        if let Some((info, _)) = self.sim.get() {
            hello["sim"] = info.clone();
        }
        hello
    }

    /// Broadcast a v2-shaped event (`{"type":"event","kind",...,"t"}`).
    pub fn emit_event(&self, kind: &str, fields: Value) {
        let mut m = json!({ "type": "event", "kind": kind, "t": now_s() });
        if let (Some(dst), Some(src)) = (m.as_object_mut(), fields.as_object()) {
            for (k, v) in src {
                dst.insert(k.clone(), v.clone());
            }
        }
        tracing::info!(target: "dash", kind, "event");
        self.hub.broadcast(&m);
    }

    /// Register + broadcast a captured-data item (dedup by name, cap 500 —
    /// the v2 sensor-data registry).
    pub fn add_sensor_data(&self, item: Value) {
        {
            let mut data = lock(&self.sensor_data);
            let name = item.get("name").cloned().unwrap_or(Value::Null);
            if data.iter().any(|d| d.get("name") == Some(&name)) {
                return;
            }
            data.push(item.clone());
            let excess = data.len().saturating_sub(500);
            if excess > 0 {
                data.drain(..excess);
            }
        }
        self.hub.broadcast(&json!({ "type": "sensor_data", "item": item }));
    }

    /// Store + broadcast one vehicle's task-queue snapshot (the 1 Hz
    /// `tasks/queue` poller feeds this). Content dedup: an unchanged queue
    /// is silent — new WS clients get the stored copy via the hello
    /// message instead. Returns whether a broadcast went out.
    pub fn on_task_queue(&self, vehicle: &str, status: Value) -> bool {
        {
            let mut queues = lock(&self.task_queues);
            if queues.get(vehicle) == Some(&status) {
                return false;
            }
            queues.insert(vehicle.to_string(), status.clone());
        }
        self.hub.broadcast(&json!({
            "type": "task_queue",
            "vehicle": vehicle,
            "status": status,
        }));
        true
    }

    /// Store + broadcast one vehicle's rc/status snapshot (the ~4 Hz
    /// `rc/status` poller feeds this — RC-CONTROL R2). Content dedup: an
    /// unchanged status is silent (new WS clients get the stored copy via
    /// hello). Returns whether a broadcast went out. Mirrors
    /// [`on_task_queue`](Self::on_task_queue).
    pub fn on_rc_status(&self, vehicle: &str, status: Value) -> bool {
        {
            let mut all = lock(&self.rc_status);
            if all.get(vehicle) == Some(&status) {
                return false;
            }
            all.insert(vehicle.to_string(), status.clone());
        }
        self.hub.broadcast(&json!({
            "type": "rc",
            "vehicle": vehicle,
            "status": status,
        }));
        true
    }

    // ── recording sessions ──────────────────────────────────────────────────

    /// Arm a recording session (mission start / explicit Record). Emits
    /// `record.started` only on a fresh arm — re-arming an open session
    /// (e.g. a mission starting under a manual recording) is silent.
    ///
    /// Recordings are derived UI artifacts (the operator's broadcast
    /// stream); the per-vehicle journal chains stay the durable truth.
    pub fn arm_recording(&self, label: &str) {
        let was_recording = self.hub.is_recording();
        if let Some(name) = self.hub.arm(label) {
            if !was_recording {
                self.emit_event("record.started", json!({ "name": name }));
            }
        }
    }

    /// Finalize the recording session (mission end / RTL-all / explicit
    /// stop). The `record.stopped` event is broadcast first so it is the
    /// recording's last line. No-op while idle.
    pub fn finalize_recording(&self) {
        if !self.hub.is_recording() {
            return;
        }
        let name = self
            .hub
            .recording_path()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()));
        self.emit_event("record.stopped", json!({ "name": name }));
        self.hub.finalize();
    }

    /// Finalize when an operator command just aborted a live mission
    /// (searcher RTL/Land mid-search, or any RTL/Land/Hold-all).
    fn finalize_if_aborted(&self, was_live: bool) {
        if was_live && self.mission_state() == "aborted" {
            self.finalize_recording();
        }
    }

    fn mission_live(&self) -> bool {
        matches!(self.mission_state().as_str(), "searching" | "investigating")
    }

    // ── action executor ─────────────────────────────────────────────────────

    /// Execute the mission machine's actions: broadcast emissions, fan out
    /// detections, dispatch jobs, start searches.
    pub fn apply_actions(self: &Arc<Self>, actions: Vec<Action>) {
        for action in actions {
            match action {
                Action::Emit(message) => {
                    let kind = if message.get("type") == Some(&json!("event")) {
                        let kind = message.get("kind").and_then(Value::as_str).unwrap_or("");
                        tracing::info!(target: "dash", kind, "event");
                        kind.to_string()
                    } else {
                        String::new()
                    };
                    // Session-scoped recording: arm BEFORE broadcasting
                    // mission.started (it must be the first recorded line)…
                    if kind == "mission.started" {
                        let label = message
                            .get("mission_id")
                            .and_then(Value::as_str)
                            .unwrap_or("mission");
                        self.arm_recording(label);
                    }
                    self.hub.broadcast(&message);
                    // …and finalize AFTER mission.completed lands.
                    if kind == "mission.completed" {
                        self.finalize_recording();
                    }
                }
                Action::SensorData(item) => self.add_sensor_data(item),
                Action::Detect { mission_id, frame, seq: _, object_query } => {
                    let dash = self.clone();
                    let timeout = Duration::from_millis(dash.config.detect_timeout_ms);
                    tokio::spawn(async move {
                        let fut = dash.detector.detect(mission_id, frame.clone(), object_query);
                        let outcome = match tokio::time::timeout(timeout, fut).await {
                            Ok(outcome) => outcome,
                            Err(_) => DetectOutcome::Timeout,
                        };
                        let actions =
                            dash.with_mission(|m| m.on_detect_outcome(&frame, outcome));
                        dash.apply_actions(actions);
                    });
                }
                Action::StartSearch { vehicle, order } => {
                    let dash = self.clone();
                    tokio::spawn(async move {
                        let deadline = Duration::from_secs_f64(order.timeout_s.max(1.0));
                        let mission_id = order.mission_id.clone();
                        let result =
                            dash.commander.raster_search(vehicle.clone(), order).await;
                        let actions = match result {
                            CmdResult::Ack(ack) if ack.accepted => {
                                // The v3 raster ack returns immediately;
                                // completion rides the search/status stream.
                                // Arm the v2-sized deadline as the backstop.
                                let dash2 = dash.clone();
                                tokio::spawn(async move {
                                    tokio::time::sleep(deadline).await;
                                    let actions = dash2.with_mission(|m| {
                                        if m.mission_id == mission_id {
                                            m.on_search_timeout()
                                        } else {
                                            Vec::new()
                                        }
                                    });
                                    dash2.apply_actions(actions);
                                });
                                Vec::new()
                            }
                            CmdResult::Ack(ack) => dash.with_mission(|m| {
                                m.on_search_response(false, "", 0, &ack.detail)
                            }),
                            CmdResult::Timeout => {
                                dash.with_mission(mission::Mission::on_search_timeout)
                            }
                            CmdResult::Error(err) => dash
                                .with_mission(|m| m.on_search_response(false, "", 0, &err)),
                        };
                        dash.apply_actions(actions);
                    });
                }
                Action::Dispatch { target_index, sensor, vehicle, order } => {
                    let dash = self.clone();
                    tokio::spawn(async move {
                        let result = dash.commander.investigate(vehicle, order).await;
                        // The investigate ACK is the typed intent decision,
                        // not the outcome: an ACCEPT leaves the job
                        // `investigating` (in flight) — it completes on the
                        // vehicle's busy→idle transition, fed by the
                        // telemetry poller into Mission::set_vehicle_busy.
                        // Refusals/timeouts keep the terminal mapping
                        // (busy-refusal requeues inside on_job_result).
                        let failed = |note: String| JobResult {
                            target_index,
                            sensor: sensor.clone(),
                            ok: false,
                            artifacts: Vec::new(),
                            note,
                            artifact_items: Vec::new(),
                        };
                        let actions = match result {
                            CmdResult::Ack(ack) if ack.accepted => dash.with_mission(|m| {
                                m.on_job_accepted(target_index, &sensor, &ack.detail)
                            }),
                            CmdResult::Ack(ack) => {
                                dash.with_mission(|m| m.on_job_result(failed(ack.detail)))
                            }
                            CmdResult::Timeout => dash
                                .with_mission(|m| m.on_job_result(failed("timeout".into()))),
                            CmdResult::Error(err) => {
                                dash.with_mission(|m| m.on_job_result(failed(err)))
                            }
                        };
                        dash.apply_actions(actions);
                    });
                }
            }
        }
    }

    // ── operator commands (from the WS) ─────────────────────────────────────

    /// v2 `handle_command`: one parsed WS text message. A `Some` return is
    /// replied to the requesting client only (`raster_preview`).
    pub fn handle_command(self: &Arc<Self>, message: &Value) -> Option<Value> {
        let cmd = message.get("cmd").and_then(Value::as_str).unwrap_or("");
        match cmd {
            "preview_raster" => {
                let f = |k: &str, d: f64| message.get(k).and_then(Value::as_f64).unwrap_or(d);
                return Some(raster::preview_message(
                    message.get("area").unwrap_or(&Value::Null),
                    f("leg_spacing_m", 5.0),
                    f("capture_every_m", 4.0),
                    f("speed_m_s", 2.0),
                ));
            }
            "start_mission" => {
                let params = message.get("params").cloned().unwrap_or_else(|| json!({}));
                let actions = self.with_mission(|m| m.start_mission(params));
                self.apply_actions(actions);
            }
            "set_enabled" => {
                let vid = message.get("vehicle").and_then(Value::as_str).unwrap_or("");
                let enabled = message.get("enabled").and_then(Value::as_bool).unwrap_or(true);
                let actions = self.with_mission(|m| m.set_enabled(vid, enabled));
                self.apply_actions(actions);
            }
            "record" => {
                // Explicit Record button: arm/stop a manual session.
                match message.get("action").and_then(Value::as_str).unwrap_or("") {
                    "start" => self.arm_recording("manual"),
                    "stop" => self.finalize_recording(),
                    _ => {}
                }
            }
            "flight" => self.cmd_flight(message),
            "task_abort" => self.cmd_task_abort(message),
            "queue_reorder" => self.cmd_queue_reorder(message),
            "job_cancel" => {
                // Queued (not yet dispatched) job removal is pure mission-
                // machine state; in-flight jobs are cancelled by task_abort
                // at their vehicle instead (the UI picks the right path).
                let index = message.get("index").and_then(Value::as_u64).unwrap_or(0) as usize;
                let sensor = message.get("sensor").and_then(Value::as_str).unwrap_or("");
                let actions = self.with_mission(|m| m.cancel_job(index, sensor));
                self.apply_actions(actions);
            }
            "candidate_promote" => {
                // "Investigate anyway" on an end-of-raster unconfirmed
                // candidate: normal queue/dispatch path; reopens a
                // completed mission until the re-armed jobs land.
                let index = message.get("index").and_then(Value::as_u64).unwrap_or(0) as usize;
                let actions = self.with_mission(|m| m.promote_unconfirmed(index));
                self.apply_actions(actions);
            }
            "candidate_dismiss" => {
                let index = message.get("index").and_then(Value::as_u64).unwrap_or(0) as usize;
                let actions = self.with_mission(|m| m.dismiss_unconfirmed(index));
                self.apply_actions(actions);
            }
            "all" => self.cmd_all(message),
            "video" => self.cmd_video(message),
            "sensor" => self.cmd_sensor(message),
            "system" => self.cmd_system(message),
            "rc" => self.cmd_rc(message),
            "sim" => self.cmd_sim(message),
            _ => {}
        }
        None
    }

    fn vehicle_of(&self, message: &Value) -> Option<String> {
        let vid = message.get("vehicle").and_then(Value::as_str)?;
        self.vehicles().contains(&vid.to_string()).then(|| vid.to_string())
    }

    fn enabled(&self, vehicle: &str) -> bool {
        lock(&self.mission).enabled.get(vehicle).copied().unwrap_or(true)
    }

    fn cmd_flight(self: &Arc<Self>, message: &Value) {
        let Some(vid) = self.vehicle_of(message) else { return };
        let command = message.get("command").and_then(Value::as_str).unwrap_or("");
        if !matches!(command, "rtl" | "land" | "hold" | "takeoff") {
            return;
        }
        // Safety actions (rtl/land/hold) are ALWAYS allowed, even to a
        // disabled vehicle — disable must never trap an aircraft in the
        // air. Only takeoff is blocked.
        if !self.enabled(&vid) && command == "takeoff" {
            self.emit_event(
                "command.rejected",
                json!({ "vehicle": vid, "command": command, "reason": "vehicle disabled" }),
            );
            return;
        }
        let was_live = self.mission_live();
        self.with_mission(|m| m.note_flight_command(&vid, command));
        self.send_flight(vid, command.to_string(), message.get("params").cloned());
        // Searcher RTL/Land aborted the mission: the recording session ends
        // with the abort command it just captured.
        self.finalize_if_aborted(was_live);
    }

    fn send_flight(self: &Arc<Self>, vid: String, command: String, params: Option<Value>) {
        // Correlation id for the per-command lifecycle strip
        // (sent → acked/refused → outcome) in the UI.
        let id = format!("cmd-{}", hub::now_ns() / 1_000_000 % 100_000_000_000);
        self.emit_event(
            "command.sent",
            json!({ "id": id, "vehicle": vid, "command": command }),
        );
        let agl = params
            .as_ref()
            .and_then(|p| p.get("target_agl_m"))
            .and_then(Value::as_f64);
        let dash = self.clone();
        tokio::spawn(async move {
            // `detail` is the provider's free-form note and rides its own
            // field; `error` appears ONLY on a refusal/transport failure —
            // an accepted ack's note must never render as an error.
            match dash.commander.flight(vid.clone(), command.clone(), agl).await {
                CmdResult::Ack(ack) => {
                    let mut m = json!({
                        "id": id,
                        "vehicle": vid,
                        "command": command,
                        "ok": ack.accepted,
                        "detail": ack.detail,
                    });
                    if !ack.accepted {
                        m["error"] = json!(ack.detail);
                    }
                    dash.emit_event("command.result", m);
                }
                CmdResult::Timeout => dash.emit_event(
                    "command.timeout",
                    json!({ "id": id, "vehicle": vid, "command": command }),
                ),
                CmdResult::Error(err) => dash.emit_event(
                    "command.result",
                    json!({
                        "id": id,
                        "vehicle": vid,
                        "command": command,
                        "ok": false,
                        "detail": err,
                        "error": err,
                    }),
                ),
            }
        });
    }

    /// Scoped cancel of one named task on one vehicle (`task_abort`): the
    /// surgical alternative to the RTL/Land/Hold ladder. Full command
    /// lifecycle (sent → acked/refused → toast) like any flight command.
    fn cmd_task_abort(self: &Arc<Self>, message: &Value) {
        let Some(vid) = self.vehicle_of(message) else { return };
        let Some(label) = message
            .get("label")
            .and_then(Value::as_str)
            .filter(|l| !l.is_empty())
            .map(str::to_string)
        else {
            return;
        };
        let id = format!("cmd-{}", hub::now_ns() / 1_000_000 % 100_000_000_000);
        let command = format!("abort {label}");
        self.emit_event(
            "command.sent",
            json!({ "id": id, "vehicle": vid, "command": command }),
        );
        let dash = self.clone();
        tokio::spawn(async move {
            match dash.commander.task_abort(vid.clone(), label.clone()).await {
                CmdResult::Ack(ack) => {
                    if ack.accepted && label == "investigate" {
                        // The busy→idle transition will complete this
                        // vehicle's job; note the abort so the outcome
                        // reads "aborted", not "completed".
                        dash.with_mission(|m| m.note_task_abort(&vid));
                    }
                    let mut m = json!({
                        "id": id,
                        "vehicle": vid,
                        "command": command,
                        "ok": ack.accepted,
                        "detail": ack.detail,
                    });
                    if !ack.accepted {
                        m["error"] = json!(ack.detail);
                    }
                    dash.emit_event("command.result", m);
                }
                CmdResult::Timeout => dash.emit_event(
                    "command.timeout",
                    json!({ "id": id, "vehicle": vid, "command": command }),
                ),
                CmdResult::Error(err) => dash.emit_event(
                    "command.result",
                    json!({
                        "id": id,
                        "vehicle": vid,
                        "command": command,
                        "ok": false,
                        "detail": err,
                        "error": err,
                    }),
                ),
            }
        });
    }

    /// Reorder one vehicle's task queue (`queue_reorder`): the UI sends the
    /// FULL desired id order — active first unless deliberately displaced
    /// (displacement splits the active task agent-side; the strip warned
    /// before the drop committed). Full command lifecycle (sent →
    /// acked/refused → toast) so the strip can revert its optimistic order
    /// on a refusal (`bad-reorder` / `queue-disabled`).
    fn cmd_queue_reorder(self: &Arc<Self>, message: &Value) {
        let Some(vid) = self.vehicle_of(message) else { return };
        let ids: Vec<String> = message
            .get("ordered_task_ids")
            .and_then(Value::as_array)
            .map(|a| a.iter().filter_map(Value::as_str).map(str::to_string).collect())
            .unwrap_or_default();
        if ids.is_empty() {
            return;
        }
        let id = format!("cmd-{}", hub::now_ns() / 1_000_000 % 100_000_000_000);
        let command = "queue_reorder".to_string();
        self.emit_event(
            "command.sent",
            json!({ "id": id, "vehicle": vid, "command": command }),
        );
        let dash = self.clone();
        tokio::spawn(async move {
            match dash.commander.queue_reorder(vid.clone(), ids).await {
                CmdResult::Ack(ack) => {
                    let mut m = json!({
                        "id": id,
                        "vehicle": vid,
                        "command": command,
                        "ok": ack.accepted,
                        "detail": ack.detail,
                    });
                    if !ack.accepted {
                        m["error"] = json!(ack.detail);
                    }
                    dash.emit_event("command.result", m);
                }
                CmdResult::Timeout => dash.emit_event(
                    "command.timeout",
                    json!({ "id": id, "vehicle": vid, "command": command }),
                ),
                CmdResult::Error(err) => dash.emit_event(
                    "command.result",
                    json!({
                        "id": id,
                        "vehicle": vid,
                        "command": command,
                        "ok": false,
                        "detail": err,
                        "error": err,
                    }),
                ),
            }
        });
    }

    fn cmd_all(self: &Arc<Self>, message: &Value) {
        let command = message.get("command").and_then(Value::as_str).unwrap_or("");
        if !matches!(command, "rtl" | "land" | "hold") {
            return;
        }
        let was_live = self.mission_live();
        self.with_mission(mission::Mission::note_all_command);
        for vid in self.vehicles() {
            self.send_flight(vid, command.to_string(), None);
        }
        // RTL/Land/Hold-all ends the mission AND its recording session (the
        // command.sent lines above are its final captured moments).
        self.finalize_if_aborted(was_live);
    }

    fn cmd_video(self: &Arc<Self>, message: &Value) {
        let Some(vid) = self.vehicle_of(message) else { return };
        let params = message.get("params").cloned().unwrap_or_else(|| json!({}));
        let enable = params.get("enable").and_then(Value::as_bool).unwrap_or(false);
        let u = |k: &str, d: u64| params.get(k).and_then(Value::as_u64).unwrap_or(d);
        let request = muas_contracts::services::VideoRequest {
            enabled: enable,
            width: u("width", 320) as u32,
            height: u("height", 240) as u32,
            fps: params.get("fps").and_then(Value::as_f64).unwrap_or(5.0) as u32,
            quality: u("quality", 40) as u32,
        };
        // Flag first so a running relay stops promptly on disable.
        let flag = {
            let mut flags = lock(&self.video_flags);
            let flag = flags
                .entry(vid.clone())
                .or_insert_with(|| Arc::new(AtomicBool::new(false)))
                .clone();
            flag.store(enable, Ordering::Relaxed);
            flag
        };
        self.emit_event("video.control", json!({ "vehicle": vid, "enable": enable }));
        let dash = self.clone();
        tokio::spawn(async move {
            match dash.commander.video_control(vid.clone(), request).await {
                CmdResult::Ack(ack) if ack.accepted => {
                    if enable {
                        dash.spawn_video_relay(&vid, flag);
                    }
                }
                CmdResult::Ack(ack) => dash.emit_event(
                    "video.control_failed",
                    json!({ "vehicle": vid, "error": ack.detail }),
                ),
                CmdResult::Timeout => {
                    dash.emit_event("video.control_timeout", json!({ "vehicle": vid }));
                }
                CmdResult::Error(err) => dash.emit_event(
                    "video.control_failed",
                    json!({ "vehicle": vid, "error": err }),
                ),
            }
        });
    }

    fn spawn_video_relay(self: &Arc<Self>, vehicle: &str, flag: Arc<AtomicBool>) {
        let Some((engine, cancel)) = self.engine.get() else { return };
        let Some(index) = self.vehicles().iter().position(|v| v == vehicle) else { return };
        let consumer = engine.app_consumer(cancel.child_token());
        tokio::spawn(ndn::video_relay(
            self.clone(),
            consumer,
            vehicle.to_string(),
            index as u8,
            flag,
            cancel.clone(),
        ));
    }

    fn cmd_sensor(self: &Arc<Self>, message: &Value) {
        let Some(vid) = self.vehicle_of(message) else { return };
        if !self.enabled(&vid) {
            self.emit_event(
                "sensor.rejected",
                json!({ "vehicle": vid, "reason": "vehicle disabled" }),
            );
            return;
        }
        let params = message.get("params").cloned().unwrap_or_else(|| json!({}));
        let s = |k: &str, d: &str| {
            params.get(k).and_then(Value::as_str).unwrap_or(d).to_string()
        };
        let f = |k: &str, d: f64| params.get(k).and_then(Value::as_f64).unwrap_or(d);
        let target = params.get("target");
        let tf = |k: &str| target.and_then(|t| t.get(k)).and_then(Value::as_f64);
        let request_id = format!("cap-{}", (hub::now_ns() / 1_000_000) % 100_000_000);
        let request = muas_contracts::services::SensorRequest {
            sensor: s("sensor", "camera"),
            mode: s("mode", "now"),
            lat_deg: tf("lat").unwrap_or(0.0),
            lon_deg: tf("lon").unwrap_or(0.0),
            radius_m: f("radius_m", 6.0),
            expiry_s: f("expires_s", 600.0),
            duration_s: f("duration_s", 6.0),
            mission_id: String::new(),
        };
        let mut fields = json!({
            "vehicle": vid,
            "request": request_id,
            "sensor": request.sensor,
            "mode": request.mode,
        });
        if let (Some(lat), Some(lon)) = (tf("lat"), tf("lon")) {
            fields["lat"] = json!(lat);
            fields["lon"] = json!(lon);
        }
        self.emit_event("sensor.request", fields);
        let dash = self.clone();
        let sensor = request.sensor.clone();
        tokio::spawn(async move {
            match dash.commander.sensor_capture(vid.clone(), request).await {
                // v3 deviation (documented): the sensor ack is the typed
                // intent decision; capture results + artifacts arrive via
                // the data plane once agent-side capture execution lands.
                CmdResult::Ack(ack) => dash.emit_event(
                    "sensor.result",
                    json!({
                        "vehicle": vid,
                        "request": request_id,
                        "sensor": sensor,
                        "status": if ack.accepted { "accepted" } else { "rejected" },
                        "message": ack.detail,
                    }),
                ),
                CmdResult::Timeout => dash.emit_event(
                    "sensor.timeout",
                    json!({ "vehicle": vid, "request": request_id }),
                ),
                CmdResult::Error(err) => dash.emit_event(
                    "sensor.failed",
                    json!({ "vehicle": vid, "request": request_id, "error": err }),
                ),
            }
        });
    }

    /// Forward one simulation-control op (anomaly placement tool) through
    /// the attached [`SimControl`] — WS → deployment control endpoint →
    /// AnomalyField. Silently ignored when no deployment attached (real
    /// deployments must not honor sim commands).
    fn cmd_sim(self: &Arc<Self>, message: &Value) {
        let Some((_, control)) = self.sim.get() else { return };
        let op = message.get("op").and_then(Value::as_str).unwrap_or("").to_string();
        if op.is_empty() {
            return;
        }
        let params = message.get("params").cloned().unwrap_or_else(|| json!({}));
        let control = control.clone();
        let dash = self.clone();
        tokio::spawn(async move {
            match control.call(op.clone(), params).await {
                Ok(result) => dash.emit_event("sim.result", json!({ "op": op, "ok": true, "result": result })),
                Err(err) => dash.emit_event("sim.result", json!({ "op": op, "ok": false, "error": err })),
            }
        });
    }

    /// RC-CONTROL R2 pilot-surface ops (`{"cmd":"rc","op":…}`). Silently
    /// inert when no `--rc-target` was configured. Frame carriage runs on
    /// the host's own 50 Hz loop; these ops only steer it.
    fn cmd_rc(self: &Arc<Self>, message: &Value) {
        let Some(host) = self.rc.get().cloned() else { return };
        let op = message.get("op").and_then(Value::as_str).unwrap_or("");
        match op {
            "engage" => {
                let target = ws_rc_target(message);
                let vids = host.engage(target);
                if vids.is_empty() {
                    self.emit_event(
                        "rc.rejected",
                        json!({ "reason": "no RC-reachable vehicle for target" }),
                    );
                } else {
                    self.emit_event("rc.engaged", json!({ "vehicles": vids }));
                }
            }
            "input" => {
                let channels: Vec<i64> = message
                    .get("channels")
                    .and_then(Value::as_array)
                    .map(|a| a.iter().filter_map(Value::as_i64).collect())
                    .unwrap_or_default();
                let arm = message.get("arm").and_then(Value::as_bool).unwrap_or(false);
                let mode = message.get("mode").and_then(Value::as_u64).unwrap_or(0) as u8;
                host.set_input(rc::sanitize_channels(&channels), arm, mode);
            }
            "estop" => {
                let on = message.get("on").and_then(Value::as_bool).unwrap_or(true);
                host.estop(on);
                self.emit_event("rc.estop", json!({ "on": on }));
            }
            "disengage" => {
                let vids = host.disengage();
                if !vids.is_empty() {
                    self.emit_event("rc.disengaged", json!({ "vehicles": vids }));
                }
                // Release the agent-side session too (the explicit path; the
                // silence ladder would release on its own from the stream
                // stopping). Fire-and-forget per engaged vehicle.
                for vid in vids {
                    let dash = self.clone();
                    tokio::spawn(async move {
                        let _ = dash.commander.rc_disengage(vid).await;
                    });
                }
            }
            _ => {}
        }
    }

    fn cmd_system(self: &Arc<Self>, message: &Value) {
        let Some(vid) = self.vehicle_of(message) else { return };
        if message.get("command").and_then(Value::as_str) != Some("shutdown") {
            return;
        }
        // Double authorization: the UI already made the operator type the
        // vehicle id; the agent re-verifies it AND its own armed/busy state
        // before doing anything.
        if message.get("confirm").and_then(Value::as_str) != Some(vid.as_str()) {
            self.emit_event(
                "system.rejected",
                json!({ "vehicle": vid, "reason": "confirm phrase mismatch" }),
            );
            return;
        }
        let armed = lock(&self.last_sample)
            .get(&vid)
            .and_then(|s| s.get("armed"))
            .and_then(Value::as_bool)
            .unwrap_or(false);
        if armed {
            self.emit_event(
                "system.rejected",
                json!({ "vehicle": vid, "reason": "vehicle is armed" }),
            );
            return;
        }
        self.emit_event("system.shutdown_sent", json!({ "vehicle": vid }));
        self.hub.sync(); // the recording should hold this moment
        let dash = self.clone();
        tokio::spawn(async move {
            match dash.commander.system_shutdown(vid.clone(), vid.clone()).await {
                CmdResult::Ack(ack) => dash.emit_event(
                    "system.shutdown_result",
                    json!({
                        "vehicle": vid,
                        "status": if ack.accepted { "accepted" } else { "rejected" },
                        "message": ack.detail,
                    }),
                ),
                CmdResult::Timeout => {
                    dash.emit_event("system.shutdown_timeout", json!({ "vehicle": vid }));
                }
                CmdResult::Error(err) => dash.emit_event(
                    "system.shutdown_failed",
                    json!({ "vehicle": vid, "error": err }),
                ),
            }
        });
    }
}

// ───────────────────────────── bring-up ─────────────────────────────────────

/// A running dashboard: HTTP server + pollers over one engine.
pub struct Running {
    pub dash: Arc<Dashboard>,
    /// The actual bound address (`--http-port 0` binds ephemerally).
    pub addr: std::net::SocketAddr,
    pub cancel: CancellationToken,
    engine_shutdown: Option<ShutdownHandle>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
    server: Option<tokio::task::JoinHandle<()>>,
}

impl Running {
    /// Resolves when the dashboard is cancelled (ctrl-c handler etc.).
    pub async fn cancelled(&self) {
        self.cancel.cancelled().await;
    }

    /// Cancel + drain: sync the recording, stop the pollers and server,
    /// shut the engine down.
    pub async fn shutdown(mut self) {
        self.cancel.cancel();
        self.dash.hub.sync();
        tokio::time::sleep(Duration::from_millis(100)).await;
        for task in self.tasks.drain(..) {
            task.abort();
            let _ = task.await;
        }
        if let Some(server) = self.server.take() {
            let _ = server.await;
        }
        if let Some(shutdown) = self.engine_shutdown.take() {
            shutdown.shutdown().await;
        }
    }
}

/// Bring the whole dashboard up: engine + faces, commander, pollers, web
/// server. The production `main` calls this with [`providers::StubDetector`].
pub async fn start(
    config: DashConfig,
    detector: Arc<dyn DetectionProvider>,
) -> Result<Running, String> {
    let cancel = CancellationToken::new();
    let (engine, engine_shutdown) = ndn::bring_up(&config.links, &cancel).await?;
    let commander = Arc::new(ndn::NdnCommander::new(
        &engine,
        &cancel,
        &config.vehicles(),
        Duration::from_millis(config.investigate_timeout_ms),
    ));
    let dash = Arc::new(Dashboard::new(config.clone(), detector, commander));
    dash.attach_engine(engine.clone(), cancel.clone());
    let mut tasks = ndn::spawn_pollers(&dash, &engine, &cancel);
    // RC pilot surface (RC-CONTROL R2): build the send host over the
    // configured vehicles on the selected carriage (default: ndf-spark over
    // the engine — `/muas/v3/<vid>/rc/spark/<index>`; `--rc-data` demotes to
    // the frame-as-Data comparison bearer), which the agent fetches over the
    // fabric, and spawn the 50 Hz pacing loop.
    if !config.rc_vehicles.is_empty() {
        let host = rc::RcHost::with_carriage(&config.rc_vehicles, config.rc_carriage);
        host.serve(&engine, &cancel).await?;
        dash.attach_rc(host.clone());
        tasks.push(tokio::spawn(host.run(cancel.clone())));
    }

    let listener =
        tokio::net::TcpListener::bind((config.http_host.as_str(), config.http_port))
            .await
            .map_err(|e| format!("http bind {}:{}: {e}", config.http_host, config.http_port))?;
    let addr = listener.local_addr().map_err(|e| format!("local addr: {e}"))?;
    let router = server::router(dash.clone());
    let shutdown_signal = cancel.clone();
    let server = tokio::spawn(async move {
        let serve = axum::serve(listener, router)
            .with_graceful_shutdown(async move { shutdown_signal.cancelled().await });
        if let Err(err) = serve.await {
            tracing::warn!(%err, "http server ended");
        }
    });
    tracing::info!(%addr, "dash.serving");
    Ok(Running {
        dash,
        addr,
        cancel,
        engine_shutdown: Some(engine_shutdown),
        tasks,
        server: Some(server),
    })
}
