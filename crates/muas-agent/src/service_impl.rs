//! Provider-side implementation of `muas_contracts::services::VehicleService`.
//!
//! Every handler is the v2 ack callback, typed: gate with
//! `muas_contracts::policy`, set the busy state, drive the backend, journal
//! the decision, return the [`Ack`]. `raster_search` / `investigate` accept
//! at ack and FLY on a spawned mission task (`crate::mission` — the v2
//! fly_raster / fly_orbit / flyover loops). With a sensor feed fitted,
//! `sensor_capture` executes all three v2 modes (`now` captures here,
//! `override` flies-captures-resumes, `opportunistic` arms a watchpoint)
//! and `video_control` drives a live renderer session; without a feed both
//! remain the documented no-hardware stubs.
//!
//! Ack semantics (ROUND-3 command.result): an accepted ack's `detail` says
//! what WILL happen ("flying to point (~8 s), capturing camera, resuming
//! raster leg 3 after") — it is never an error; rejections carry the
//! policy `code` plus the human reason in `detail`.

use std::sync::atomic::Ordering;
use std::sync::Arc;

use muas_contracts::policy;
use muas_contracts::services::{
    sensor_mode, Ack, InvestigateRequest, QueueReorderRequest, RasterRequest, SensorRequest,
    TakeoffRequest, VehicleService, VideoRequest,
};
use muas_contracts::tasks::task_origin;
use crate::queue::{self, Submit, TaskParams};
use crate::{lock, mission, AgentCommand, AgentShared, BackendExt};

/// The agent's vehicle-service provider.
pub struct VehicleServiceImpl {
    pub shared: Arc<AgentShared>,
}

/// Point-in-time flight state read under one backend lock.
struct FlightSnapshot {
    home: Option<(f64, f64)>,
    position: Option<(f64, f64, f64)>,
    armed: bool,
}

impl VehicleServiceImpl {
    pub fn new(shared: Arc<AgentShared>) -> Self {
        Self { shared }
    }

    fn journal_ack(&self, op: &str, request: serde_json::Value, ack: &Ack) {
        self.shared.journal.event(
            &format!("service.{op}"),
            serde_json::json!({
                "request": request,
                "accepted": ack.accepted,
                "code": ack.code,
                "detail": ack.detail,
            }),
        );
    }

    /// Snapshot (home, position, armed) without holding the backend lock.
    fn flight_snapshot(&self) -> FlightSnapshot {
        let (home, position, armed) = {
            let backend = lock(&self.shared.backend);
            let b = backend.as_dyn_ref();
            (b.home(), b.position(), b.telemetry().armed)
        };
        FlightSnapshot {
            // Fall back to the agent-side home capture (lock-per-poll
            // takeoff records it there for the MAVLink backend).
            home: home.or(*lock(&self.shared.fallback_home)),
            position,
            armed,
        }
    }

    /// Abort ladder entry: raise the abort flag (running mission loops
    /// terminate within one control cycle), clear the busy label, cancel
    /// smart RTL, and flush every PENDING queue entry (the ladder is a
    /// blanket stop — the interrupting command owns the vehicle). Never a
    /// scoped operator cancel — lower `operator_abort` so the interrupted
    /// runner does NOT hand the vehicle to the idle policy.
    fn abort_running_task(&self) {
        self.shared.operator_abort.store(false, Ordering::Relaxed);
        self.shared.abort.store(true, Ordering::Relaxed);
        lock(&self.shared.busy).clear();
        queue::flush_pending(&self.shared, "ladder");
        let _ = self.shared.commands.send(AgentCommand::AbortRtl);
    }

    /// Map a [`Submit`] outcome to the service ack. `started_detail` is the
    /// immediate-start acceptance note (the pre-queue ack text, unchanged);
    /// `legacy_start` runs the pre-queue occupy+spawn path when the queue
    /// engine is disabled (`Submit::Disabled`).
    fn submit_ack(
        &self,
        outcome: Submit,
        started_detail: String,
        legacy_start: impl FnOnce(),
    ) -> Ack {
        match outcome {
            Submit::Started { .. } => Ack::ok_detail(started_detail),
            Submit::Queued { task_id, ahead, eta_to_start_s } => Ack::queued(format!(
                "task {task_id} queued at position {ahead}; starts in ~{eta_to_start_s:.0} s"
            )),
            Submit::Full { depth } => Ack::refuse(
                "queue-full",
                format!("task queue depth limit reached ({depth} pending)"),
            ),
            Submit::RtlOwned => Ack::reject(&policy::PolicyRejection::Busy {
                task: "rtl".to_string(),
            }),
            Submit::Disabled => {
                legacy_start();
                Ack::ok_detail(started_detail)
            }
        }
    }

    /// Occupy the vehicle for a freshly accepted task: set the busy label
    /// and clear any stale abort so the new task isn't instantly cancelled
    /// (v2 `set_busy` + `abort.clear()`).
    fn occupy(&self, label: &str) {
        *lock(&self.shared.busy) = label.to_string();
        self.shared.abort.store(false, Ordering::Relaxed);
        self.shared.operator_abort.store(false, Ordering::Relaxed);
    }

    /// Scoped cancel of the ACTIVE task: the label must match the current
    /// busy label exactly — a mismatch means the task the operator is
    /// looking at already ended (or was re-labelled), and blind aborts of
    /// "whatever runs now" are refused. With queued tasks pending, the
    /// queue continues with the next entry; only an empty queue hands the
    /// vehicle to the idle policy.
    fn abort_active_task(&self, label: &str) -> Ack {
        let matched = {
            let mut busy = lock(&self.shared.busy);
            if label.is_empty() || *busy != label {
                Err(busy.clone())
            } else {
                // Order matters (all under the busy lock): the runner's
                // interrupt check reads `abort || busy != label`, so
                // both abort flags are up before the label clears —
                // whoever observes the release also sees the operator
                // provenance and hands over to the idle policy / queue.
                self.shared.operator_abort.store(true, Ordering::Relaxed);
                self.shared.abort.store(true, Ordering::Relaxed);
                busy.clear();
                Ok(())
            }
        };
        match matched {
            Err(current) if current.is_empty() => {
                Ack::refuse("no-such-task", format!("no active task '{label}' (vehicle idle)"))
            }
            Err(current) => Ack::refuse(
                "no-such-task",
                format!("active task is '{current}', not '{label}'"),
            ),
            Ok(()) => {
                if label == "rtl" {
                    // The only busy label whose loop lives on the coord
                    // thread: tell it to stand down its smart RTL.
                    let _ = self.shared.commands.send(AgentCommand::AbortRtl);
                }
                self.shared.journal.event(
                    "task.aborted",
                    serde_json::json!({ "label": label, "by": "operator" }),
                );
                let pending = lock(&self.shared.tasks).pending_len();
                if pending > 0 {
                    Ack::ok_detail(format!(
                        "task '{label}' aborted; next queued task takes over ({pending} pending)"
                    ))
                } else {
                    Ack::ok_detail(format!(
                        "task '{label}' aborted; vehicle idle (idle policy takes over, no RTL)"
                    ))
                }
            }
        }
    }

    /// What an override detour resumes afterwards, for the ack detail
    /// ("resuming raster leg 3" / "holding here"). The raster leg comes off
    /// the vehicle's own `search/status` sample.
    fn override_resume_note(&self, busy: &str) -> String {
        if busy != "raster-search" {
            return "holding here".to_string();
        }
        let leg = lock(&self.shared.latest_search)
            .as_ref()
            .and_then(|bytes| {
                serde_json::from_slice::<uas_fleet_data::kinds::SearchStatus>(bytes).ok()
            })
            .map(|status| status.leg);
        match leg {
            Some(leg) => format!("resuming raster leg {leg}"),
            None => "resuming raster".to_string(),
        }
    }
}

impl VehicleService for VehicleServiceImpl {
    async fn flight_rtl(&self) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "flight_rtl").entered();
        // RTL is the abort ladder — never busy-gated; the running task
        // terminates within one cycle (abort flag raised, label cleared)
        // and every pending queue entry flushes (blanket stop).
        // A blanket ladder stop, not a scoped cancel (see abort_running_task).
        self.shared.operator_abort.store(false, Ordering::Relaxed);
        self.shared.abort.store(true, Ordering::Relaxed);
        lock(&self.shared.busy).clear();
        queue::flush_pending(&self.shared, "ladder");
        let ack = if self.shared.smart_rtl {
            *lock(&self.shared.busy) = "rtl".to_string();
            let _ = self.shared.commands.send(AgentCommand::SmartRtl);
            Ack::ok_detail("smart rtl engaged (slot-layered)")
        } else if lock(&self.shared.backend).as_dyn().rtl() {
            Ack::ok_detail("native rtl")
        } else {
            Ack::refuse("backend-refused", "autopilot refused RTL")
        };
        self.journal_ack("flight_rtl", serde_json::json!({}), &ack);
        ack
    }

    async fn flight_land(&self) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "flight_land").entered();
        self.abort_running_task();
        let ack = if lock(&self.shared.backend).as_dyn().land() {
            Ack::ok()
        } else {
            Ack::refuse("backend-refused", "autopilot refused LAND")
        };
        self.journal_ack("flight_land", serde_json::json!({}), &ack);
        ack
    }

    async fn flight_hold(&self) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "flight_hold").entered();
        self.abort_running_task();
        let ack = if lock(&self.shared.backend).as_dyn().hold() {
            Ack::ok()
        } else {
            Ack::refuse("backend-refused", "autopilot refused HOLD")
        };
        self.journal_ack("flight_hold", serde_json::json!({}), &ack);
        ack
    }

    async fn flight_takeoff(&self, req: TakeoffRequest) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "flight_takeoff").entered();
        let gate = policy::agl_guard(req.agl_m, self.shared.agl_bounds)
            .and_then(|()| policy::busy_guard(&lock(&self.shared.busy)));
        let ack = match gate {
            Err(rejection) => Ack::reject(&rejection),
            Ok(()) => {
                // Occupy the vehicle for the climb; the mission task
                // releases it when the (lock-per-poll) climb settles.
                self.occupy("takeoff");
                tokio::spawn(mission::takeoff_task(self.shared.clone(), req.agl_m));
                Ack::ok_detail("takeoff started")
            }
        };
        self.journal_ack(
            "flight_takeoff",
            serde_json::json!({ "agl_m": req.agl_m }),
            &ack,
        );
        ack
    }

    async fn raster_search(&self, req: RasterRequest) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "raster_search").entered();
        let home = self.flight_snapshot().home;
        // With the queue engine on, busy no longer refuses (accept-and-
        // queue); the field-safety rails still gate every request.
        let gate = if self.shared.queue_enabled {
            Ok(())
        } else {
            policy::busy_guard(&lock(&self.shared.busy))
        }
        .and_then(|()| policy::agl_guard(req.agl_m, self.shared.agl_bounds))
        .and_then(|()| policy::range_guard(home, &req.corners, self.shared.max_range_m));
        let ack = match gate {
            Err(rejection) => Ack::reject(&rejection),
            // Geometry is validated at ack, exactly the v2 "empty raster"
            // rejection; a good plan flies on the mission task.
            Ok(()) => match mission::plan_raster(&req) {
                Err(err) => Ack::refuse("bad-raster", err),
                Ok(plan) => {
                    let detail = format!(
                        "raster accepted: {} legs, {} captures",
                        plan.legs.len(),
                        plan.capture_count()
                    );
                    let outcome = queue::submit(
                        &self.shared,
                        TaskParams::Raster {
                            req: req.clone(),
                            start_leg: 0,
                            skip_captures: 0,
                        },
                        task_origin::OPERATOR,
                    );
                    self.submit_ack(outcome, detail, || {
                        self.occupy("raster-search");
                        tokio::spawn(mission::run_raster(self.shared.clone(), req.clone(), plan));
                    })
                }
            },
        };
        self.journal_ack(
            "raster_search",
            serde_json::to_value(&req).unwrap_or_default(),
            &ack,
        );
        ack
    }

    async fn investigate(&self, req: InvestigateRequest) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "investigate").entered();
        let home = self.flight_snapshot().home;
        // Busy queues instead of refusing when the queue engine is on.
        let gate = if self.shared.queue_enabled {
            Ok(())
        } else {
            policy::busy_guard(&lock(&self.shared.busy))
        }
        .and_then(|()| policy::agl_guard(req.agl_m, self.shared.agl_bounds))
        .and_then(|()| {
            policy::range_guard(home, &[(req.lat_deg, req.lon_deg)], self.shared.max_range_m)
        });
        let ack = match gate {
            Err(rejection) => Ack::reject(&rejection),
            // v2 geometry gate: a non-positive radius or turn count is
            // rejected at ack, not discovered mid-flight.
            Ok(()) if req.radius_m <= 0.0 => {
                Ack::refuse("bad-orbit", "invalid request geometry (radius_m <= 0)")
            }
            Ok(()) => {
                // Pattern by requested sensor + capability (ROUND-3):
                // audio-only jobs fly the acoustic flyover, camera keeps
                // the carrot orbit; the ack names the selected pattern.
                let pattern = mission::select_investigate_pattern(
                    &req,
                    self.shared.extras.iter().any(|e| e == "audio"),
                );
                let detail =
                    if pattern == muas_contracts::services::investigate_pattern::FLYOVER {
                        "acoustic flyover accepted".to_string()
                    } else {
                        "carrot-orbit accepted".to_string()
                    };
                // Origin: the mission machine always stamps a mission id on
                // dispatched jobs; bare requests are operator-issued.
                let origin = if req.mission_id.is_empty() {
                    task_origin::OPERATOR
                } else {
                    task_origin::DISPATCH
                };
                let outcome = queue::submit(
                    &self.shared,
                    TaskParams::Investigate { req: req.clone() },
                    origin,
                );
                self.submit_ack(outcome, detail, || {
                    self.occupy("investigate");
                    tokio::spawn(mission::run_investigate(self.shared.clone(), req.clone()));
                })
            }
        };
        self.journal_ack(
            "investigate",
            serde_json::to_value(&req).unwrap_or_default(),
            &ack,
        );
        ack
    }

    async fn sensor_capture(&self, req: SensorRequest) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "sensor_capture").entered();
        let snapshot = self.flight_snapshot();
        let busy = lock(&self.shared.busy).clone();
        let gate = (|| {
            if req.mode == sensor_mode::OVERRIDE {
                // The detour flies to the point: the range rail applies
                // (queued or not — the rails gate every acceptance).
                policy::range_guard(
                    snapshot.home,
                    &[(req.lat_deg, req.lon_deg)],
                    self.shared.max_range_m,
                )?;
            }
            // Audio is short-range: gate tasked capture points on mic reach
            // (mode `now` captures wherever the vehicle already is).
            if req.sensor == "audio" && req.mode != sensor_mode::NOW {
                let vehicle = snapshot
                    .position
                    .map(|(lat, lon, _)| (lat, lon))
                    .unwrap_or((req.lat_deg, req.lon_deg));
                policy::audio_range_guard(
                    vehicle,
                    (req.lat_deg, req.lon_deg),
                    self.shared.audio_range_m,
                )?;
            }
            Ok(())
        })();
        let ack = match gate {
            Err(rejection) => Ack::reject(&rejection),
            // Mode `override` (v2 fly-capture-resume): fly to the point,
            // capture there, then resume the pre-empted task by re-issuing
            // its target — the ack says exactly what will happen. v2 could
            // only pre-empt a raster (detour) or an idle vehicle; anything
            // else owning the vehicle refused busy. With the queue engine
            // on, those refusals become accept-and-queue (the override runs
            // as a queue task once the occupying task finishes); rtl keeps
            // the refusal (never queue work behind a return-to-launch).
            Ok(()) if req.mode == sensor_mode::OVERRIDE
                && (busy == "investigate"
                    || busy == "rtl"
                    || busy == "takeoff"
                    || busy == "sensor-override"
                    || self.shared.detour.load(Ordering::Relaxed)) =>
            {
                if self.shared.queue_enabled && !busy.is_empty() && busy != "rtl" {
                    let outcome = queue::submit(
                        &self.shared,
                        TaskParams::SensorOverride { req: req.clone() },
                        task_origin::OPERATOR,
                    );
                    self.submit_ack(
                        outcome,
                        format!("flying to point, capturing {}", req.sensor),
                        || {}, // unreachable: queue_enabled checked above
                    )
                } else {
                    let task = if busy.is_empty() {
                        "sensor-override".to_string()
                    } else {
                        busy.clone()
                    };
                    Ack::reject(&policy::PolicyRejection::Busy { task })
                }
            }
            // No sensor feed fitted: the pre-v3.1 stub behavior (busy /
            // queue semantics above still apply first, exactly like the
            // v2 gate ordering).
            Ok(()) if self.shared.sensor_feed.is_none() => {
                Ack::ok_detail("accepted; capture execution stubbed (no sensor feed)")
            }
            // Mode `now`: capture at the current pose, publish the artifact
            // over the data plane, surface the result on `sensor/last`.
            Ok(()) if req.mode == sensor_mode::NOW => {
                tokio::spawn(crate::sensor::capture_now_task(self.shared.clone(), req.clone()));
                Ack::ok_detail(format!("capturing {} here now", req.sensor))
            }
            Ok(()) if req.mode == sensor_mode::OVERRIDE => {
                let airborne = snapshot.armed
                    && snapshot.position.is_some_and(|(_, _, agl)| agl >= 1.0);
                if !airborne {
                    Ack::refuse("not-airborne", "override capture needs an airborne vehicle")
                } else {
                    let here = snapshot
                        .position
                        .map(|(lat, lon, _)| (lat, lon))
                        .unwrap_or((req.lat_deg, req.lon_deg));
                    let eta_s = policy::dist_m(here, (req.lat_deg, req.lon_deg))
                        / crate::sensor::OVERRIDE_SPEED_M_S
                        + 3.0;
                    let resume_note = self.override_resume_note(&busy);
                    // Claim the vehicle race-free BEFORE spawning: the
                    // detour flag suspends a running raster; an idle
                    // vehicle is additionally occupied by the busy label.
                    let resume_task = if busy.is_empty() {
                        self.occupy("sensor-override");
                        "sensor-override".to_string()
                    } else {
                        busy.clone()
                    };
                    self.shared.detour.store(true, Ordering::Relaxed);
                    tokio::spawn(crate::sensor::override_capture_task(
                        self.shared.clone(),
                        req.clone(),
                        resume_task,
                    ));
                    Ack::ok_detail(format!(
                        "flying to point (~{eta_s:.0} s), capturing {}, {resume_note} after",
                        req.sensor
                    ))
                }
            }
            // Mode `opportunistic`: arm a watchpoint that fires in passing.
            // Registered by id so `task_abort("watchpoint:<id>")` can remove
            // it without touching whatever task owns the vehicle.
            Ok(()) if req.mode == sensor_mode::OPPORTUNISTIC => {
                let radius_m = if req.radius_m > 0.0 { req.radius_m } else { 15.0 };
                let expiry_s = if req.expiry_s > 0.0 { req.expiry_s } else { 120.0 };
                let id = format!(
                    "wp-{}",
                    self.shared.watchpoint_seq.fetch_add(1, Ordering::Relaxed) + 1
                );
                let session = self.shared.cancel.child_token();
                lock(&self.shared.watchpoints).push(crate::Watchpoint {
                    id: id.clone(),
                    sensor: req.sensor.clone(),
                    cancel: session.clone(),
                });
                tokio::spawn(crate::sensor::watchpoint_task(
                    self.shared.clone(),
                    req.clone(),
                    id.clone(),
                    session,
                ));
                Ack::ok_detail(format!(
                    "watchpoint {id} armed: {} fires within {radius_m:.0} m, expires in {expiry_s:.0} s",
                    req.sensor
                ))
            }
            Ok(()) => Ack::refuse(
                "bad-mode",
                format!("unknown capture mode '{}' (now|override|opportunistic)", req.mode),
            ),
        };
        self.journal_ack(
            "sensor_capture",
            serde_json::to_value(&req).unwrap_or_default(),
            &ack,
        );
        ack
    }

    async fn video_control(&self, req: VideoRequest) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "video_control").entered();
        // With a sensor feed fitted the knob is real: a session task renders
        // frames into the `video/live` latest-wins buffer, which the GCS
        // relay fetches over the fabric — the same path a real camera's
        // MJPEG pipeline will use. Without a feed, the v2-era stub ack.
        let ack = if self.shared.sensor_feed.is_none() {
            Ack::ok_detail("accepted; video pipeline stubbed (no sensor feed)")
        } else if req.enabled {
            let session = self.shared.cancel.child_token();
            {
                let mut slot = lock(&self.shared.video_session);
                if let Some(previous) = slot.take() {
                    previous.cancel(); // restart with the new parameters
                }
                *slot = Some(session.clone());
            }
            tokio::spawn(crate::sensor::video_task(self.shared.clone(), req.fps, session));
            Ack::ok_detail("video started (sensor feed)")
        } else {
            if let Some(session) = lock(&self.shared.video_session).take() {
                session.cancel();
            }
            Ack::ok_detail("video stopped")
        };
        self.journal_ack(
            "video_control",
            serde_json::to_value(&req).unwrap_or_default(),
            &ack,
        );
        ack
    }

    async fn task_abort(&self, label: String) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "task_abort").entered();
        let ack = if let Some(id) = label.strip_prefix("watchpoint:") {
            // Watchpoints ride along with whatever the vehicle is doing —
            // cancelling one by id never touches the active task. The
            // watchpoint task unregisters itself when the token fires.
            let token = lock(&self.shared.watchpoints)
                .iter()
                .find(|w| w.id == id)
                .map(|w| w.cancel.clone());
            match token {
                Some(token) => {
                    token.cancel();
                    self.shared.journal.event(
                        "task.aborted",
                        serde_json::json!({ "label": label, "by": "operator" }),
                    );
                    Ack::ok_detail(format!("watchpoint {id} cancelled"))
                }
                None => Ack::refuse("no-such-task", format!("no armed watchpoint '{id}'")),
            }
        } else if label.starts_with("tsk-") {
            // Queue-entry abort by id: a PENDING entry is removed without
            // touching the flight; the ACTIVE entry's id is equivalent to
            // a label abort of its kind.
            match queue::abort_by_id(&self.shared, &label) {
                queue::ById::Pending => {
                    Ack::ok_detail(format!("pending task {label} removed from the queue"))
                }
                queue::ById::Active(kind) => self.abort_active_task(kind),
                queue::ById::None => {
                    Ack::refuse("no-such-task", format!("no queue task '{label}'"))
                }
            }
        } else {
            self.abort_active_task(&label)
        };
        self.journal_ack("task_abort", serde_json::json!({ "label": label }), &ack);
        ack
    }

    async fn queue_reorder(&self, req: QueueReorderRequest) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "queue_reorder").entered();
        let ack = if !self.shared.queue_enabled {
            Ack::refuse("queue-disabled", "task queue engine is disabled (--no-queue)")
        } else {
            match queue::reorder(&self.shared, &req.ordered_task_ids) {
                Ok(true) => Ack::ok_detail(
                    "queue reordered; active task splits — its remainder resumes at the new position",
                ),
                Ok(false) => Ack::ok_detail("queue reordered"),
                Err(err) => Ack::refuse("bad-reorder", err),
            }
        };
        self.journal_ack(
            "queue_reorder",
            serde_json::json!({ "ordered_task_ids": req.ordered_task_ids }),
            &ack,
        );
        ack
    }

    async fn rc_disengage(&self) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "rc_disengage").entered();
        // The RC task owns the session: raise the request flag and let it
        // release on its next tick (override released, `rc-manual` busy
        // cleared, `rc.released{reason:"operator"}` journaled, and a
        // paused mission resumed via the queue kick).
        let ack = if self.shared.rc_engaged.load(Ordering::Relaxed) {
            self.shared.rc_disengage.store(true, Ordering::Relaxed);
            Ack::ok_detail("rc disengage requested; override releasing")
        } else {
            Ack::refuse("rc-not-engaged", "no rc-over-ndn session is engaged")
        };
        self.journal_ack("rc_disengage", serde_json::json!({}), &ack);
        ack
    }

    async fn system_shutdown(&self, confirm: String) -> Ack {
        let span = tracing::info_span!("service-invocation", op = "system_shutdown");
        let gate = {
            let _entered = span.enter();
            // Ack gate (first authorization).
            let armed = self.flight_snapshot().armed;
            let busy = lock(&self.shared.busy).clone();
            policy::shutdown_guard(&confirm, &self.shared.vehicle_id, armed, &busy).and_then(
                |()| {
                    // Handler re-verification (second authorization): re-read
                    // live state, exactly the v2 double-check — a takeoff or
                    // task acked between the two reads flips the answer.
                    let armed = self.flight_snapshot().armed;
                    let busy = lock(&self.shared.busy).clone();
                    policy::shutdown_guard(&confirm, &self.shared.vehicle_id, armed, &busy)
                },
            )
        };
        let ack = match gate {
            Err(rejection) => Ack::reject(&rejection),
            Ok(()) => {
                self.shared
                    .journal
                    .event("system.shutdown", serde_json::json!({ "confirm": confirm }));
                // Flush + fsync the journal before acking (v2: flush, sync).
                self.shared.journal.sync().await;
                let _ = self.shared.commands.send(AgentCommand::Shutdown);
                // DEVIATION (documented): v2 powered the companion off after
                // a 3 s delay; the v3 dev build logs and exits the process
                // gracefully instead — no poweroff is issued.
                Ack::ok_detail("shutdown accepted: journal synced; agent process exiting (no poweroff in v3 dev build)")
            }
        };
        let _entered = span.enter();
        self.journal_ack(
            "system_shutdown",
            serde_json::json!({ "confirm": "<redacted>" }),
            &ack,
        );
        ack
    }
}
