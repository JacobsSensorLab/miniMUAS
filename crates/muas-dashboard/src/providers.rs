//! Injection seams: the detection provider (stub / scripted /
//! [`crate::detect::SimpleDetector`]), the vehicle commander (the NDN
//! service client in production, scripted fakes in tests), and the
//! simulation-control seam an embedding virtual deployment plugs in.

use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::Mutex;

use muas_contracts::services::Ack;
use serde_json::Value;

use crate::mission::{DetectOutcome, InvestigateOrder, RasterOrder};

/// A boxed, sendable future (the traits stay dyn-compatible).
pub type BoxFuture<T> = Pin<Box<dyn Future<Output = T> + Send + 'static>>;

// ───────────────────────────── detection ────────────────────────────────────

/// The offboard detection seam (v2's `perception/detect-object` service).
///
/// STUB status: the v3 perception service is a later increment, so the
/// production wiring uses [`StubDetector`] (every frame is a clean miss)
/// and tests script outcomes through [`ScriptedDetector`]. The mission
/// state machine is detector-agnostic either way.
pub trait DetectionProvider: Send + Sync {
    /// Run detection over one published frame; resolves to the outcome the
    /// mission machine consumes.
    fn detect(&self, mission_id: String, frame: String, object_query: String)
        -> BoxFuture<DetectOutcome>;
}

/// Production stand-in until the perception service lands: every frame is
/// an explicit miss so the mission still counts detects and completes.
pub struct StubDetector;

impl DetectionProvider for StubDetector {
    fn detect(&self, _mission_id: String, _frame: String, _query: String)
        -> BoxFuture<DetectOutcome> {
        Box::pin(async {
            DetectOutcome::Miss("detection provider stubbed (perception service pending)".into())
        })
    }
}

/// Scripted detector for tests: maps frame names to outcomes; unknown
/// frames miss.
#[derive(Default)]
pub struct ScriptedDetector {
    outcomes: Mutex<HashMap<String, DetectOutcome>>,
}

impl ScriptedDetector {
    /// Script one frame's outcome.
    pub fn script(&self, frame: impl Into<String>, outcome: DetectOutcome) {
        self.outcomes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert(frame.into(), outcome);
    }
}

impl DetectionProvider for ScriptedDetector {
    fn detect(&self, _mission_id: String, frame: String, _query: String)
        -> BoxFuture<DetectOutcome> {
        let outcome = self
            .outcomes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get(&frame)
            .cloned()
            .unwrap_or(DetectOutcome::Miss(String::new()));
        Box::pin(async move { outcome })
    }
}

// ───────────────────────────── sim control ──────────────────────────────────

/// The simulation-control seam: an embedding virtual deployment attaches an
/// implementation (its HTTP/WS control endpoint client) with
/// [`crate::Dashboard::attach_sim`]; the anomaly-placement tool's WS
/// commands (`{"cmd":"sim","op":..,"params":..}`) are forwarded through it.
/// Real deployments never attach one, and the UI never shows the tool
/// (capability-gated by the hello message).
pub trait SimControl: Send + Sync {
    /// Execute one control operation (e.g. `place_anomaly`,
    /// `clear_anomalies`) against the deployment; resolves to the
    /// endpoint's JSON reply.
    fn call(&self, op: String, params: Value) -> BoxFuture<Result<Value, String>>;
}

// ───────────────────────────── commander ────────────────────────────────────

/// One service-call outcome, typed like the v2 async callback trio
/// (response / timeout / transport error).
#[derive(Debug, Clone, PartialEq)]
pub enum CmdResult {
    /// The provider acked (accepted or typed rejection).
    Ack(Ack),
    /// No answer within the deadline.
    Timeout,
    /// The call could not be transported (no route, engine down, ...).
    Error(String),
}

/// The vehicle service-call seam. Production: `VehicleServiceClient` over
/// `FaceRpcCarrier` (see `crate::ndn::NdnCommander`); tests: a scripted
/// fake.
pub trait Commander: Send + Sync {
    /// `flight/rtl|land|hold|takeoff` (takeoff carries the AGL).
    fn flight(&self, vehicle: String, command: String, agl_m: Option<f64>)
        -> BoxFuture<CmdResult>;
    /// `raster-search` toward the WUAS.
    fn raster_search(&self, vehicle: String, order: RasterOrder) -> BoxFuture<CmdResult>;
    /// `investigate` toward an IUAS.
    fn investigate(&self, vehicle: String, order: InvestigateOrder) -> BoxFuture<CmdResult>;
    /// Scoped cancel of one named task (`task_abort`): `label` is the
    /// vehicle's busy label (`raster-search`, `investigate`,
    /// `sensor-override`, `takeoff`, `rtl`), a queue id (`tsk-<n>` removes
    /// that pending entry), or `watchpoint:<id>`.
    fn task_abort(&self, vehicle: String, label: String) -> BoxFuture<CmdResult>;
    /// `queue_reorder`: the FULL desired queue order (every current queue
    /// task id — active first unless deliberately displaced; displacement
    /// splits the active task agent-side).
    fn queue_reorder(&self, vehicle: String, ordered_task_ids: Vec<String>)
        -> BoxFuture<CmdResult>;
    /// `sensor/capture`.
    fn sensor_capture(
        &self,
        vehicle: String,
        request: muas_contracts::services::SensorRequest,
    ) -> BoxFuture<CmdResult>;
    /// `video/control`.
    fn video_control(
        &self,
        vehicle: String,
        request: muas_contracts::services::VideoRequest,
    ) -> BoxFuture<CmdResult>;
    /// Authorized companion shutdown (`confirm` must equal the vehicle id).
    fn system_shutdown(&self, vehicle: String, confirm: String) -> BoxFuture<CmdResult>;
}

/// Test commander: every call resolves to a preset result and is logged.
pub struct ScriptedCommander {
    /// Calls seen, as `(vehicle, op)` pairs.
    pub calls: Mutex<Vec<(String, String)>>,
    result: CmdResult,
}

impl ScriptedCommander {
    /// A commander answering every call with `result`.
    pub fn answering(result: CmdResult) -> Self {
        Self { calls: Mutex::new(Vec::new()), result }
    }

    fn answer(&self, vehicle: String, op: &str) -> BoxFuture<CmdResult> {
        self.calls
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .push((vehicle, op.to_string()));
        let result = self.result.clone();
        Box::pin(async move { result })
    }
}

impl Commander for ScriptedCommander {
    fn flight(&self, vehicle: String, command: String, _agl_m: Option<f64>)
        -> BoxFuture<CmdResult> {
        let op = format!("flight/{command}");
        self.answer(vehicle, &op)
    }
    fn raster_search(&self, vehicle: String, _order: RasterOrder) -> BoxFuture<CmdResult> {
        self.answer(vehicle, "raster-search")
    }
    fn investigate(&self, vehicle: String, _order: InvestigateOrder) -> BoxFuture<CmdResult> {
        self.answer(vehicle, "investigate")
    }
    fn task_abort(&self, vehicle: String, label: String) -> BoxFuture<CmdResult> {
        let op = format!("task_abort/{label}");
        self.answer(vehicle, &op)
    }
    fn queue_reorder(&self, vehicle: String, ordered_task_ids: Vec<String>)
        -> BoxFuture<CmdResult> {
        let op = format!("queue_reorder/{}", ordered_task_ids.join(","));
        self.answer(vehicle, &op)
    }
    fn sensor_capture(
        &self,
        vehicle: String,
        _request: muas_contracts::services::SensorRequest,
    ) -> BoxFuture<CmdResult> {
        self.answer(vehicle, "sensor/capture")
    }
    fn video_control(
        &self,
        vehicle: String,
        _request: muas_contracts::services::VideoRequest,
    ) -> BoxFuture<CmdResult> {
        self.answer(vehicle, "video/control")
    }
    fn system_shutdown(&self, vehicle: String, _confirm: String) -> BoxFuture<CmdResult> {
        self.answer(vehicle, "system/shutdown")
    }
}
