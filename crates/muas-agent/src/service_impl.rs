//! Provider-side implementation of `muas_contracts::services::VehicleService`.
//!
//! Every handler is the v2 ack callback, typed: gate with
//! `muas_contracts::policy`, set the busy state, drive the backend, journal
//! the decision, return the [`Ack`]. `raster_search` / `investigate` accept
//! at ack and FLY on a spawned mission task (`crate::mission` — the v2
//! fly_raster / fly_orbit loops); `sensor_capture` / `video_control` remain
//! **stubbed** (no camera/audio hardware in this build — accepted +
//! journaled, no capture executes; see the STUB markers).

use std::sync::atomic::Ordering;
use std::sync::Arc;

use muas_contracts::policy;
use muas_contracts::services::{
    sensor_mode, Ack, InvestigateRequest, RasterRequest, SensorRequest, TakeoffRequest,
    VehicleService, VideoRequest,
};
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
    /// smart RTL.
    fn abort_running_task(&self) {
        self.shared.abort.store(true, Ordering::Relaxed);
        lock(&self.shared.busy).clear();
        let _ = self.shared.commands.send(AgentCommand::AbortRtl);
    }

    /// Occupy the vehicle for a freshly accepted task: set the busy label
    /// and clear any stale abort so the new task isn't instantly cancelled
    /// (v2 `set_busy` + `abort.clear()`).
    fn occupy(&self, label: &str) {
        *lock(&self.shared.busy) = label.to_string();
        self.shared.abort.store(false, Ordering::Relaxed);
    }
}

impl VehicleService for VehicleServiceImpl {
    async fn flight_rtl(&self) -> Ack {
        let _span = tracing::info_span!("service-invocation", op = "flight_rtl").entered();
        // RTL is the abort ladder — never busy-gated; the running task
        // terminates within one cycle (abort flag raised, label cleared).
        self.shared.abort.store(true, Ordering::Relaxed);
        lock(&self.shared.busy).clear();
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
        let gate = policy::busy_guard(&lock(&self.shared.busy))
            .and_then(|()| policy::agl_guard(req.agl_m, self.shared.agl_bounds))
            .and_then(|()| policy::range_guard(home, &req.corners, self.shared.max_range_m));
        let ack = match gate {
            Err(rejection) => Ack::reject(&rejection),
            // Geometry is validated at ack, exactly the v2 "empty raster"
            // rejection; a good plan flies on the mission task.
            Ok(()) => match mission::plan_raster(&req) {
                Err(err) => Ack::refuse("bad-raster", err),
                Ok(plan) => {
                    self.occupy("raster-search");
                    let detail = format!(
                        "raster accepted: {} legs, {} captures",
                        plan.legs.len(),
                        plan.capture_count()
                    );
                    tokio::spawn(mission::run_raster(self.shared.clone(), req.clone(), plan));
                    Ack::ok_detail(detail)
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
        let gate = policy::busy_guard(&lock(&self.shared.busy))
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
                self.occupy("investigate");
                tokio::spawn(mission::run_investigate(self.shared.clone(), req.clone()));
                Ack::ok_detail("carrot-orbit accepted")
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
        let position = self.flight_snapshot().position;
        let busy = lock(&self.shared.busy).clone();
        let gate = (|| {
            // v2: override (fly-capture-resume) is rejected mid-investigation;
            // other modes ride alongside the running task.
            if req.mode == sensor_mode::OVERRIDE && busy == "investigate" {
                return Err(policy::PolicyRejection::Busy { task: busy.clone() });
            }
            // Audio is short-range: gate tasked capture points on mic reach
            // (mode `now` captures wherever the vehicle already is).
            if req.sensor == "audio" && req.mode != sensor_mode::NOW {
                let vehicle = position
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
            // STUB (M3): capture scheduling (now / override / opportunistic
            // watchpoints) is not executed yet — accepted + journaled only.
            Ok(()) => Ack::ok_detail("accepted; capture execution stubbed at M3 (task journaled)"),
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
        // STUB (M3): the MJPEG pipeline (CameraHub) is not ported yet — the
        // knob is accepted + journaled so dashboards can already drive it.
        let ack = Ack::ok_detail("accepted; video pipeline stubbed at M3");
        self.journal_ack(
            "video_control",
            serde_json::to_value(&req).unwrap_or_default(),
            &ack,
        );
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
