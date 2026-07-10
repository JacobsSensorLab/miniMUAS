//! The v3 vehicle service contract: one `#[ndn_service]` trait mirroring the
//! v2 task-intent services (`docs/v3/surveys/minimuas-v2.md` §Drone agent).
//!
//! # Shape
//!
//! v2 named each op as its own NDNSF service (`/muas/v2/<vid>/flight/rtl`,
//! `.../sensor/capture`, ...). The `#[ndn_service]` macro prefers one trait
//! per provider — every op becomes an `OpId` under a single [`ServiceId`] —
//! so v3 hosts the whole vehicle surface as one service rooted at
//! [`crate::names::vehicle_prefix`] (`/muas/v3/<vid>`), with op names
//! (`flight_rtl`, `sensor_capture`, ...) carrying the old sub-path meaning as
//! a single name component (`/muas/v3/<vid>/flight_rtl` on the wire for the
//! rpc carrier).
//!
//! # Framing
//!
//! `ndn-service-core` provides [`Frame`] only for scalars (`u64`, `bool`,
//! `String`, ...) — **not `f64`** — so the flight-request types here cannot
//! ride `#[derive(Frame)]`'s per-field composition. Instead every
//! request/response struct is serde `Serialize`/`Deserialize` and implements
//! `Frame` as one JSON field ([`json_frame!`]). This keeps the wire
//! self-describing and byte-compatible with v2's JSON-dict payload
//! convention, at the cost of the length-prefixed append-only evolution rule
//! (JSON tolerates unknown fields anyway).
//!
//! Every op returns an [`Ack`]: the v2 ack-gate decision, typed. Rejections
//! carry the [`crate::policy::PolicyRejection`] code + human detail.

use serde::{Deserialize, Serialize};

use bytes::Bytes;
use ndn_service_core::{Frame, ServiceError};
use ndn_service_macro::ndn_service;

use crate::policy::PolicyRejection;

/// Implement [`Frame`] as a single JSON document for serde types.
macro_rules! json_frame {
    ($($t:ty),+ $(,)?) => {$(
        impl Frame for $t {
            fn encode(&self) -> Bytes {
                Bytes::from(serde_json::to_vec(self).expect("plain data types encode infallibly"))
            }
            fn decode(bytes: &[u8]) -> Result<Self, ServiceError> {
                serde_json::from_slice(bytes).map_err(|e| ServiceError::Decode(e.to_string()))
            }
        }
    )+};
}

// ---------------------------------------------------------------------------
// Ack — the typed ack-gate decision every op returns
// ---------------------------------------------------------------------------

/// Service acknowledgement: v2's `AckDecision`, typed. `accepted == false`
/// carries the policy rejection `code` (see
/// [`PolicyRejection::code`]) and a human-readable `detail`.
///
/// # `detail` is not an error (ROUND-3 command.result semantics)
///
/// `detail` is the provider's free-form note in BOTH directions: on an
/// accepted ack it says what will happen ("flying to point, ~8 s, resuming
/// raster leg 3 after"); on a rejection it is the reason. Consumers must
/// never surface an accepted ack's `detail` under an `error` label — an
/// error exists only when `accepted == false` (then `code` is the
/// machine-readable discriminator and `detail` the human text).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct Ack {
    pub accepted: bool,
    /// Machine-readable rejection code; empty when accepted.
    #[serde(default)]
    pub code: String,
    /// Human-readable detail (rejection reason or acceptance note).
    #[serde(default)]
    pub detail: String,
}

impl Ack {
    /// Accepted, no note.
    pub fn ok() -> Self {
        Self {
            accepted: true,
            ..Self::default()
        }
    }

    /// Accepted with a note (e.g. "accepted; execution stubbed").
    pub fn ok_detail(detail: impl Into<String>) -> Self {
        Self {
            accepted: true,
            code: String::new(),
            detail: detail.into(),
        }
    }

    /// Rejected by a typed policy rule.
    pub fn reject(rejection: &PolicyRejection) -> Self {
        Self {
            accepted: false,
            code: rejection.code().to_string(),
            detail: rejection.to_string(),
        }
    }

    /// Rejected outside the policy table (e.g. backend refused the command).
    pub fn refuse(code: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            accepted: false,
            code: code.into(),
            detail: detail.into(),
        }
    }

    /// Collapse an ack-gate result: `Ok` ⇒ accepted, `Err` ⇒ typed rejection.
    pub fn gate(result: Result<(), PolicyRejection>) -> Self {
        match result {
            Ok(()) => Self::ok(),
            Err(rejection) => Self::reject(&rejection),
        }
    }
}

// ---------------------------------------------------------------------------
// Request types (serde + Frame via JSON)
// ---------------------------------------------------------------------------

/// `flight_takeoff`: guarded, AGL-gated, occupies the vehicle while climbing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct TakeoffRequest {
    /// Target altitude above ground level, metres.
    pub agl_m: f64,
}

/// `raster_search` (WUAS): serpentine survey of the corner-defined area.
/// Field names mirror the v2 dashboard raster params.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct RasterRequest {
    /// Survey altitude AGL, metres.
    pub agl_m: f64,
    /// Leg spacing, metres.
    pub spacing_m: f64,
    /// Along-track capture interval, metres (0 = no captures).
    #[serde(default)]
    pub capture_every_m: f64,
    /// Commanded ground speed, m/s.
    #[serde(default)]
    pub speed_m_s: f64,
    /// Area corners as `(lat_deg, lon_deg)` (v2 corners mode; center mode is
    /// expanded to corners by the caller).
    pub corners: Vec<(f64, f64)>,
    /// Detection object query (e.g. "person"), empty = stub detector.
    #[serde(default)]
    pub object_query: String,
    /// Minimum detection confidence 0..1.
    #[serde(default)]
    pub min_confidence: f64,
    /// Distinct-target separation, metres.
    #[serde(default)]
    pub target_separation_m: f64,
    /// Mission id the artifacts publish under.
    #[serde(default)]
    pub mission_id: String,
}

/// Investigation flight patterns (additive `pattern` field on
/// [`InvestigateRequest`]; absent/empty means [`AUTO`](investigate_pattern::AUTO)).
pub mod investigate_pattern {
    /// Provider selects by requested sensor + capability: audio-only jobs
    /// fly the acoustic flyover, everything else the carrot orbit.
    pub const AUTO: &str = "auto";
    /// Force the continuous carrot-chasing orbit.
    pub const ORBIT: &str = "orbit";
    /// Force the acoustic flyover (transit → dip over target → climb out).
    pub const FLYOVER: &str = "flyover";
}

/// `investigate` (IUAS): close inspection of a target point — a continuous
/// carrot-chasing orbit (camera) or an acoustic flyover (audio), selected by
/// `pattern`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct InvestigateRequest {
    /// Target latitude, degrees.
    pub lat_deg: f64,
    /// Target longitude, degrees.
    pub lon_deg: f64,
    /// Orbit/cruise altitude AGL, metres.
    pub agl_m: f64,
    /// Orbit radius (or flyover dip radius), metres.
    pub radius_m: f64,
    /// Number of turns to fly (flyover: number of passes).
    #[serde(default)]
    pub turns: f64,
    /// Sensors to run during the inspection (`"camera"`, `"audio"`).
    #[serde(default)]
    pub sensors: Vec<String>,
    /// Mission id the artifacts publish under.
    #[serde(default)]
    pub mission_id: String,
    /// Flight pattern: see [`investigate_pattern`]. Additive — absent or
    /// empty on the wire means `auto` (older callers keep orbit behavior
    /// for camera jobs).
    #[serde(default)]
    pub pattern: String,
}

/// `sensor_capture` modes, byte-for-byte with v2.
pub mod sensor_mode {
    /// Capture immediately at the current position.
    pub const NOW: &str = "now";
    /// Fly-capture-resume (rejected mid-investigation).
    pub const OVERRIDE: &str = "override";
    /// Watchpoint with radius + expiry, captured in passing.
    pub const OPPORTUNISTIC: &str = "opportunistic";
}

/// `sensor_capture`: one tasked capture (camera or audio).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct SensorRequest {
    /// `"camera"` or `"audio"`.
    pub sensor: String,
    /// See [`sensor_mode`].
    pub mode: String,
    /// Capture point latitude, degrees (ignored for mode `now`).
    #[serde(default)]
    pub lat_deg: f64,
    /// Capture point longitude, degrees (ignored for mode `now`).
    #[serde(default)]
    pub lon_deg: f64,
    /// Watchpoint radius, metres (mode `opportunistic`).
    #[serde(default)]
    pub radius_m: f64,
    /// Watchpoint expiry, seconds (mode `opportunistic`).
    #[serde(default)]
    pub expiry_s: f64,
    /// Capture duration, seconds (audio).
    #[serde(default)]
    pub duration_s: f64,
    /// Mission id the artifact publishes under.
    #[serde(default)]
    pub mission_id: String,
}

/// `video_control`: MJPEG live-video knob.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct VideoRequest {
    /// Start (`true`) or stop (`false`) the stream.
    pub enabled: bool,
    /// Frame width, pixels (0 = provider default).
    #[serde(default)]
    pub width: u32,
    /// Frame height, pixels (0 = provider default).
    #[serde(default)]
    pub height: u32,
    /// Frames per second (0 = provider default).
    #[serde(default)]
    pub fps: u32,
    /// JPEG quality 1..100 (0 = provider default).
    #[serde(default)]
    pub quality: u32,
}

json_frame!(
    Ack,
    TakeoffRequest,
    RasterRequest,
    InvestigateRequest,
    SensorRequest,
    VideoRequest,
);

// ---------------------------------------------------------------------------
// The service trait
// ---------------------------------------------------------------------------

/// The per-vehicle v3 service surface (v2 parity, one trait). Host it under
/// [`crate::names::vehicle_prefix`] with any `ndn-service-core` carrier; the
/// macro generates `VehicleServiceDispatch` (provider side) and
/// `VehicleServiceClient` (caller side).
///
/// Ack-gating contract (enforced by providers via [`crate::policy`]):
/// - `flight_takeoff`: AGL guard + busy guard; occupies the vehicle.
/// - `flight_rtl` / `flight_land` / `flight_hold`: never gated on busy — they
///   ARE the abort ladder (running task terminates within one cycle).
/// - `raster_search` / `investigate`: busy guard, AGL guard, range guard.
/// - `sensor_capture`: busy rules per mode; audio range guard.
/// - `system_shutdown`: `confirm` must equal the vehicle id, refused while
///   armed or busy — checked at ack AND at commit.
#[ndn_service]
pub trait VehicleService {
    /// Return to launch (smart slot-layered RTL when a fleet is configured).
    async fn flight_rtl(&self) -> Ack;
    /// Land in place.
    async fn flight_land(&self) -> Ack;
    /// Position-hold in place.
    async fn flight_hold(&self) -> Ack;
    /// Arm + climb to `req.agl_m`.
    async fn flight_takeoff(&self, req: TakeoffRequest) -> Ack;
    /// Serpentine survey (WUAS role).
    async fn raster_search(&self, req: RasterRequest) -> Ack;
    /// Carrot-orbit inspection (IUAS role).
    async fn investigate(&self, req: InvestigateRequest) -> Ack;
    /// Tasked sensor capture.
    async fn sensor_capture(&self, req: SensorRequest) -> Ack;
    /// Live video stream control.
    async fn video_control(&self, req: VideoRequest) -> Ack;
    /// Authorized companion shutdown; `confirm` must equal the vehicle id.
    async fn system_shutdown(&self, confirm: String) -> Ack;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy;

    #[test]
    fn ack_frames_round_trip() {
        let acks = [
            Ack::ok(),
            Ack::ok_detail("accepted; execution stubbed"),
            Ack::reject(&PolicyRejection::Busy {
                task: "raster-search".into(),
            }),
            Ack::refuse("backend-refused", "mode change rejected"),
        ];
        for ack in acks {
            let decoded = Ack::decode(&ack.encode()).unwrap();
            assert_eq!(decoded, ack);
        }
    }

    #[test]
    fn rejected_ack_carries_policy_code_and_detail() {
        let rejection = policy::agl_guard(50.0, policy::AglBounds::default()).unwrap_err();
        let ack = Ack::reject(&rejection);
        assert!(!ack.accepted);
        assert_eq!(ack.code, "agl-out-of-bounds");
        assert!(ack.detail.contains("50"), "detail: {}", ack.detail);
    }

    #[test]
    fn request_frames_round_trip() {
        let raster = RasterRequest {
            agl_m: 12.0,
            spacing_m: 8.0,
            capture_every_m: 5.0,
            speed_m_s: 4.0,
            corners: vec![(35.0, -90.0), (35.001, -90.0), (35.001, -90.001)],
            object_query: "person".into(),
            min_confidence: 0.5,
            target_separation_m: 10.0,
            mission_id: "m1".into(),
        };
        assert_eq!(RasterRequest::decode(&raster.encode()).unwrap(), raster);

        let takeoff = TakeoffRequest { agl_m: 6.5 };
        assert_eq!(TakeoffRequest::decode(&takeoff.encode()).unwrap(), takeoff);
    }

    #[test]
    fn json_wire_tolerates_missing_optional_fields() {
        // Peers may send minimal dicts; #[serde(default)] fills the rest.
        let req: SensorRequest =
            serde_json::from_str(r#"{"sensor":"camera","mode":"now"}"#).unwrap();
        assert_eq!(req.sensor, "camera");
        assert_eq!(req.mode, sensor_mode::NOW);
        assert_eq!(req.duration_s, 0.0);
    }

    #[test]
    fn investigate_pattern_is_additive_and_defaults_to_auto_semantics() {
        // Pre-pattern callers send no `pattern`: decodes to empty, which
        // providers must treat as `auto`.
        let old: InvestigateRequest = serde_json::from_str(
            r#"{"lat_deg":35.0,"lon_deg":-90.0,"agl_m":8.0,"radius_m":6.0}"#,
        )
        .unwrap();
        assert_eq!(old.pattern, "");

        let new = InvestigateRequest {
            pattern: investigate_pattern::FLYOVER.to_string(),
            sensors: vec!["audio".into()],
            ..InvestigateRequest::default()
        };
        let decoded = InvestigateRequest::decode(&new.encode()).unwrap();
        assert_eq!(decoded, new);
        assert_eq!(decoded.pattern, "flyover");
    }

    /// The command.result contract (ROUND-3): success notes ride `detail`,
    /// never an `error`-named field — the wire shape has exactly the three
    /// documented keys in both directions.
    #[test]
    fn ack_wire_shape_has_no_error_field() {
        for ack in [
            Ack::ok_detail("flying to point, ~8s, resuming raster leg 3 after"),
            Ack::refuse("busy", "vehicle busy with task 'investigate'"),
        ] {
            let value: serde_json::Value = serde_json::from_slice(&ack.encode()).unwrap();
            let mut keys: Vec<&str> =
                value.as_object().unwrap().keys().map(String::as_str).collect();
            keys.sort_unstable();
            assert_eq!(keys, vec!["accepted", "code", "detail"]);
            assert!(value.get("error").is_none());
        }
        // Accepted acks never carry a rejection code.
        assert!(Ack::ok_detail("note").code.is_empty());
    }
}
