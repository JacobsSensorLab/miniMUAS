//! NDN side of the dashboard: engine + UDP faces (exactly the muas-agent
//! bring-up pattern), the latest-wins pollers (~3 Hz Consumer fetches with
//! MustBeFresh), the `VehicleService` client commander over
//! `FaceRpcCarrier`, the artifact fetcher, and the binary video relay.

use std::collections::{BTreeSet, HashMap};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use ndn_app::{Consumer, EngineAppExt};
use ndn_engine::builder::{EngineBuilder, EngineConfig};
use ndn_engine::{ForwarderEngine, ShutdownHandle};
use ndn_face::UdpFace;
use ndn_packet::encode::InterestBuilder;
use ndn_packet::Name;
use ndn_rpc::FaceRpcCarrier;
use ndn_service_core::{ServiceError, ServiceId};
use serde_json::{json, Value};
use tokio_util::sync::CancellationToken;
use tracing::{debug, info};
use uas_fleet_data::kinds::{CapabilityProfile, FrameHeader, SearchStatus, TelemetrySample};

use muas_contracts::names;
use muas_contracts::services::{
    InvestigateRequest, RasterRequest, SensorRequest, TakeoffRequest, VehicleServiceClient,
    VideoRequest,
};

use crate::config::UdpLink;
use crate::mission::{InvestigateOrder, RasterOrder};
use crate::providers::{BoxFuture, CmdResult, Commander};
use crate::Dashboard;

/// Latest-wins fetch Interest lifetime (matches the agent's peer fetcher:
/// stale-fast data wants short lifetimes).
const FETCH_LIFETIME: Duration = Duration::from_millis(800);

/// Bring the forwarding engine up with the configured point-to-point UDP
/// faces (the muas-agent pattern: bind, add face, route, settle).
pub async fn bring_up(
    links: &[UdpLink],
    cancel: &CancellationToken,
) -> Result<(ForwarderEngine, ShutdownHandle), String> {
    let (engine, shutdown) = EngineBuilder::new(EngineConfig::default())
        .build()
        .await
        .map_err(|e| format!("engine build failed: {e}"))?;
    for link in links {
        let face_id = engine.faces().alloc_id();
        let face = UdpFace::bind(link.local, link.remote, face_id)
            .await
            .map_err(|e| format!("udp face {} -> {}: {e}", link.local, link.remote))?;
        engine.add_face(face, cancel.child_token());
        if let Some(route) = &link.route {
            let prefix: Name = route
                .parse()
                .map_err(|e| format!("bad route prefix '{route}': {e:?}"))?;
            engine.fib().add_nexthop(&prefix, face_id, 0);
        }
        info!(local = %link.local, remote = %link.remote, route = ?link.route, "udp face up");
    }
    if !links.is_empty() {
        // Faces settle on the real clock (muas-agent pattern).
        tokio::time::sleep(Duration::from_millis(150)).await;
    }
    Ok((engine, shutdown))
}

/// Fetch the freshest sample of a vehicle stream (MustBeFresh so cached
/// stale copies are skipped — the agent stamps no freshness period, so
/// every fetch reaches the producer).
async fn fetch_latest(consumer: &mut Consumer, name: &str) -> Option<bytes::Bytes> {
    let name: Name = name.parse().ok()?;
    let data = consumer
        .fetch_with(InterestBuilder::new(name).must_be_fresh().lifetime(FETCH_LIFETIME))
        .await
        .ok()?;
    data.content().cloned()
}

// ───────────────────────────── commander ────────────────────────────────────

type Client = VehicleServiceClient<FaceRpcCarrier>;

/// Production [`Commander`]: one short-timeout client per vehicle for
/// command acks, one long-timeout client for investigate (v2 sized these
/// timeouts per call; `FaceRpcCarrier`'s timeout is per client).
pub struct NdnCommander {
    short: Arc<HashMap<String, Client>>,
    long: Arc<HashMap<String, Client>>,
}

impl NdnCommander {
    /// Build clients for every vehicle over the shared engine.
    pub fn new(
        engine: &ForwarderEngine,
        cancel: &CancellationToken,
        vehicles: &[String],
        investigate_timeout: Duration,
    ) -> Self {
        let mut short = HashMap::new();
        let mut long = HashMap::new();
        for vid in vehicles {
            let prefix: Name = names::vehicle_prefix(vid)
                .parse()
                .expect("vehicle prefix is a valid name");
            short.insert(
                vid.clone(),
                VehicleServiceClient::new(
                    FaceRpcCarrier::client(engine.app_consumer(cancel.child_token()))
                        .with_timeout(Duration::from_secs(15)),
                    ServiceId::new(prefix.clone()),
                ),
            );
            long.insert(
                vid.clone(),
                VehicleServiceClient::new(
                    FaceRpcCarrier::client(engine.app_consumer(cancel.child_token()))
                        .with_timeout(investigate_timeout),
                    ServiceId::new(prefix),
                ),
            );
        }
        Self { short: Arc::new(short), long: Arc::new(long) }
    }
}

fn to_result(r: Result<muas_contracts::services::Ack, ServiceError>) -> CmdResult {
    match r {
        Ok(ack) => CmdResult::Ack(ack),
        // FaceRpcCarrier reports an unanswered call as NotFound/Transport;
        // fold both timeout-shaped failures into the v2 timeout leg.
        Err(ServiceError::NotFound) => CmdResult::Timeout,
        Err(err) => CmdResult::Error(err.to_string()),
    }
}

fn no_vehicle(vehicle: &str) -> CmdResult {
    CmdResult::Error(format!("unknown vehicle '{vehicle}'"))
}

impl Commander for NdnCommander {
    fn flight(&self, vehicle: String, command: String, agl_m: Option<f64>)
        -> BoxFuture<CmdResult> {
        let clients = self.short.clone();
        Box::pin(async move {
            let Some(client) = clients.get(&vehicle) else { return no_vehicle(&vehicle) };
            let result = match command.as_str() {
                "rtl" => client.flight_rtl().await,
                "land" => client.flight_land().await,
                "hold" => client.flight_hold().await,
                "takeoff" => {
                    client
                        .flight_takeoff(TakeoffRequest { agl_m: agl_m.unwrap_or(5.0) })
                        .await
                }
                other => return CmdResult::Error(format!("unknown flight command '{other}'")),
            };
            to_result(result)
        })
    }

    fn raster_search(&self, vehicle: String, order: RasterOrder) -> BoxFuture<CmdResult> {
        let clients = self.short.clone();
        Box::pin(async move {
            let Some(client) = clients.get(&vehicle) else { return no_vehicle(&vehicle) };
            to_result(
                client
                    .raster_search(RasterRequest {
                        agl_m: order.agl_m,
                        spacing_m: order.spacing_m,
                        capture_every_m: order.capture_every_m,
                        speed_m_s: order.speed_m_s,
                        corners: order.corners,
                        object_query: order.object_query,
                        min_confidence: order.min_confidence,
                        target_separation_m: order.target_separation_m,
                        mission_id: order.mission_id,
                    })
                    .await,
            )
        })
    }

    fn investigate(&self, vehicle: String, order: InvestigateOrder) -> BoxFuture<CmdResult> {
        let clients = self.long.clone();
        Box::pin(async move {
            let Some(client) = clients.get(&vehicle) else { return no_vehicle(&vehicle) };
            to_result(
                client
                    .investigate(InvestigateRequest {
                        lat_deg: order.lat_deg,
                        lon_deg: order.lon_deg,
                        agl_m: order.agl_m,
                        radius_m: order.radius_m,
                        turns: order.turns,
                        sensors: order.sensors,
                        mission_id: order.mission_id,
                    })
                    .await,
            )
        })
    }

    fn sensor_capture(&self, vehicle: String, request: SensorRequest) -> BoxFuture<CmdResult> {
        let clients = self.long.clone();
        Box::pin(async move {
            let Some(client) = clients.get(&vehicle) else { return no_vehicle(&vehicle) };
            to_result(client.sensor_capture(request).await)
        })
    }

    fn video_control(&self, vehicle: String, request: VideoRequest) -> BoxFuture<CmdResult> {
        let clients = self.short.clone();
        Box::pin(async move {
            let Some(client) = clients.get(&vehicle) else { return no_vehicle(&vehicle) };
            to_result(client.video_control(request).await)
        })
    }

    fn system_shutdown(&self, vehicle: String, confirm: String) -> BoxFuture<CmdResult> {
        let clients = self.short.clone();
        Box::pin(async move {
            let Some(client) = clients.get(&vehicle) else { return no_vehicle(&vehicle) };
            to_result(client.system_shutdown(confirm).await)
        })
    }
}

// ───────────────────────────── pollers ──────────────────────────────────────

/// Spawn every background poller onto the runtime. One poller task PER
/// STREAM (the v2 lesson: a slow vehicle must not stall the others).
pub fn spawn_pollers(
    dash: &Arc<Dashboard>,
    engine: &ForwarderEngine,
    cancel: &CancellationToken,
) -> Vec<tokio::task::JoinHandle<()>> {
    let mut tasks = Vec::new();
    for vid in dash.vehicles() {
        tasks.push(tokio::spawn(telemetry_poller(
            dash.clone(),
            engine.app_consumer(cancel.child_token()),
            vid.clone(),
            cancel.clone(),
        )));
        tasks.push(tokio::spawn(coord_poller(
            dash.clone(),
            engine.app_consumer(cancel.child_token()),
            vid,
            cancel.clone(),
        )));
    }
    tasks.push(tokio::spawn(search_poller(
        dash.clone(),
        engine.app_consumer(cancel.child_token()),
        cancel.clone(),
    )));
    tasks.push(tokio::spawn(capabilities_poller(
        dash.clone(),
        engine.app_consumer(cancel.child_token()),
        cancel.clone(),
    )));
    tasks.push(tokio::spawn(sensor_event_poller(
        dash.clone(),
        engine.app_consumer(cancel.child_token()),
        cancel.clone(),
    )));
    tasks
}

/// ~3 Hz telemetry follower for one vehicle (the agents publish at 4 Hz).
/// Link age is measured on OUR clock from the last NEW `gps_time_ns`
/// (skew-immune); clock skew is reported separately, exactly like v2.
async fn telemetry_poller(
    dash: Arc<Dashboard>,
    mut consumer: Consumer,
    vehicle: String,
    cancel: CancellationToken,
) {
    let name = names::vehicle_stream(&vehicle, "telemetry/live");
    let mut interval = tokio::time::interval(Duration::from_millis(300));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut last_ns: Option<u64> = None;
    let mut changed_at = tokio::time::Instant::now();
    let mut last_success: Option<tokio::time::Instant> = None;
    loop {
        tokio::select! {
            () = cancel.cancelled() => break,
            _ = interval.tick() => {}
        }
        match fetch_latest(&mut consumer, &name).await {
            Some(payload) => {
                let Ok(sample_value) = serde_json::from_slice::<Value>(&payload) else {
                    continue;
                };
                let Ok(sample) = serde_json::from_value::<TelemetrySample>(sample_value.clone())
                else {
                    continue;
                };
                let now = tokio::time::Instant::now();
                if last_ns != Some(sample.gps_time_ns) {
                    last_ns = Some(sample.gps_time_ns);
                    changed_at = now;
                }
                last_success = Some(now);
                let age_s = now.duration_since(changed_at).as_secs_f64();
                let now_ns = crate::hub::now_ns();
                let skew_s = (now_ns as i128 - sample.gps_time_ns as i128) as f64 / 1e9;
                dash.set_last_sample(&vehicle, sample_value.clone());
                dash.lens.on_sample(&vehicle, &sample, now_ns);
                dash.hub.broadcast(&json!({
                    "type": "telemetry",
                    "vehicle": vehicle,
                    "sample": sample_value,
                    "age_s": (age_s * 10.0).round() / 10.0,
                    "skew_s": (skew_s * 10.0).round() / 10.0,
                }));
            }
            None => {
                // Stale-marker danger: tell the UI explicitly how old we are.
                let silent_s = last_success
                    .map(|t| tokio::time::Instant::now().duration_since(t).as_secs_f64());
                dash.hub.broadcast(&json!({
                    "type": "telemetry_stale",
                    "vehicle": vehicle,
                    "silent_s": silent_s.map(|s| (s * 10.0).round() / 10.0),
                }));
            }
        }
    }
}

/// 2 Hz search-status follower (only while a search is running). Feeds new
/// frame names into the mission machine oldest-first and drives search
/// completion from the terminal stream states (v3 deviation: the raster
/// *ack* returns immediately, so completion rides `search/status` instead
/// of a blocking service response).
async fn search_poller(dash: Arc<Dashboard>, mut consumer: Consumer, cancel: CancellationToken) {
    let vehicle = dash.wuas_id();
    let name = names::vehicle_stream(&vehicle, "search/status");
    let mut interval = tokio::time::interval(Duration::from_millis(500));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            () = cancel.cancelled() => break,
            _ = interval.tick() => {}
        }
        if dash.mission_state() != "searching" {
            continue;
        }
        let Some(payload) = fetch_latest(&mut consumer, &name).await else { continue };
        let Ok(status_value) = serde_json::from_slice::<Value>(&payload) else { continue };
        let Ok(status) = serde_json::from_value::<SearchStatus>(status_value.clone()) else {
            continue;
        };
        {
            let (pending, done) = dash.detect_counters();
            dash.hub.broadcast(&json!({
                "type": "search_status",
                "vehicle": vehicle,
                "status": status_value,
                "detects_pending": pending,
                "detects_done": done,
            }));
        }
        // last_frames is newest-first; dispatch oldest-first so detections
        // leave (and usually return) in capture order.
        for frame in status.last_frames.iter().rev() {
            let actions = dash.with_mission(|m| m.on_new_frame(frame));
            dash.apply_actions(actions);
        }
        match status.state.as_str() {
            "done" | "found" | "aborted" => {
                let actions = dash.with_mission(|m| {
                    m.on_search_response(true, &status.state, status.frames_captured, "")
                });
                dash.apply_actions(actions);
            }
            "failed" => {
                let actions = dash.with_mission(|m| {
                    m.on_search_response(false, &status.state, status.frames_captured, "search failed")
                });
                dash.apply_actions(actions);
            }
            _ => {}
        }
    }
}

/// 0.1 Hz capability follower: which investigation sensors each vehicle
/// advertises (CapabilityProfile extras; legacy assumption = camera).
async fn capabilities_poller(
    dash: Arc<Dashboard>,
    mut consumer: Consumer,
    cancel: CancellationToken,
) {
    let vehicles = dash.vehicles();
    let mut interval = tokio::time::interval(Duration::from_secs(10));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            () = cancel.cancelled() => break,
            _ = interval.tick() => {}
        }
        for vid in &vehicles {
            let name = names::vehicle_stream(vid, "telemetry/state");
            let Some(payload) = fetch_latest(&mut consumer, &name).await else { continue };
            let Ok(profile) = serde_json::from_slice::<CapabilityProfile>(&payload) else {
                continue;
            };
            let sensors: BTreeSet<String> = {
                let s: BTreeSet<String> = profile
                    .extras
                    .iter()
                    .filter(|s| s.as_str() == "camera" || s.as_str() == "audio")
                    .cloned()
                    .collect();
                if s.is_empty() {
                    ["camera".to_string()].into()
                } else {
                    s
                }
            };
            dash.lens.set_sensors(vid, sensors.iter().cloned().collect());
            let actions = dash.with_mission(|m| m.set_capabilities(vid, sensors));
            dash.apply_actions(actions);
        }
    }
}

/// 1 Hz coord-status follower: relays each vehicle's cooperative-avoidance
/// entries. Additive v3 message type (`"coord"`); the ported frontend
/// ignores it, replay tooling gets the data.
async fn coord_poller(
    dash: Arc<Dashboard>,
    mut consumer: Consumer,
    vehicle: String,
    cancel: CancellationToken,
) {
    let name = names::vehicle_stream(&vehicle, "coord/status");
    let mut interval = tokio::time::interval(Duration::from_secs(1));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut last: Option<Value> = None;
    loop {
        tokio::select! {
            () = cancel.cancelled() => break,
            _ = interval.tick() => {}
        }
        let Some(payload) = fetch_latest(&mut consumer, &name).await else { continue };
        let Ok(entries) = serde_json::from_slice::<Value>(&payload) else { continue };
        if last.as_ref() == Some(&entries) {
            continue;
        }
        last = Some(entries.clone());
        dash.hub.broadcast(&json!({
            "type": "coord",
            "vehicle": vehicle,
            "entries": entries,
        }));
    }
}

/// Tasked-capture result relay (v2 `_poll_sensor_events_forever`): results
/// the service response can't carry — opportunistic watchpoints fire long
/// after their ack — ride the `sensor/last` latest-wins name. The payload
/// schema is pinned to the v2 `SensorCaptureResult` JSON dict until
/// muas-contracts grows the typed twin (agent-side capture execution is a
/// later increment).
async fn sensor_event_poller(
    dash: Arc<Dashboard>,
    mut consumer: Consumer,
    cancel: CancellationToken,
) {
    let vehicles = dash.vehicles();
    let mut interval = tokio::time::interval(Duration::from_millis(1_500));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut seen: HashMap<String, (String, u64, String)> = HashMap::new();
    loop {
        tokio::select! {
            () = cancel.cancelled() => break,
            _ = interval.tick() => {}
        }
        for vid in &vehicles {
            let name = names::vehicle_stream(vid, "sensor/last");
            let Some(payload) = fetch_latest(&mut consumer, &name).await else { continue };
            let Ok(result) = serde_json::from_slice::<Value>(&payload) else { continue };
            let s = |k: &str| result.get(k).and_then(Value::as_str).unwrap_or("").to_string();
            let key = (
                s("request_id"),
                result.get("gps_time_ns").and_then(Value::as_u64).unwrap_or(0),
                s("status"),
            );
            if seen.get(vid) == Some(&key) {
                continue;
            }
            seen.insert(vid.clone(), key);
            on_sensor_result(&dash, vid, &result);
        }
    }
}

/// v2 `_on_sensor_result`: event line + captured artifacts onto the
/// sensor-data map layer.
fn on_sensor_result(dash: &Arc<Dashboard>, vehicle: &str, result: &Value) {
    let s = |k: &str| result.get(k).and_then(Value::as_str).unwrap_or("").to_string();
    let status = s("status");
    let sensor = s("sensor");
    let mut fields = json!({
        "vehicle": vehicle,
        "request": s("request_id"),
        "sensor": sensor,
        "status": status,
    });
    let message = s("message");
    if !message.is_empty() {
        fields["message"] = json!(message);
    }
    let lat = result.get("lat_deg").and_then(Value::as_f64);
    let lon = result.get("lon_deg").and_then(Value::as_f64);
    if status == "captured" {
        fields["lat"] = json!(lat.unwrap_or(0.0));
        fields["lon"] = json!(lon.unwrap_or(0.0));
    }
    dash.emit_event("sensor.result", fields);
    if status != "captured" {
        return;
    }
    let artifacts = result
        .get("artifacts")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    for name in artifacts.iter().filter_map(Value::as_str) {
        dash.add_sensor_data(json!({
            "vehicle": vehicle,
            "sensor": sensor,
            "kind": if sensor == "audio" { "audio/wav" } else { "image/jpeg" },
            "name": name,
            "lat": lat.unwrap_or(0.0),
            "lon": lon.unwrap_or(0.0),
            "t": crate::hub::now_ns() as f64 / 1e9,
            "source": "tasked",
            "label": format!("tasked {sensor}"),
        }));
    }
}

// ───────────────────────────── video relay ──────────────────────────────────

/// Poll one vehicle's latest-wins `video/live` name and forward new frames
/// as binary WS messages (`[vehicle index][jpeg]`), with fps/kbps/seq stats
/// every 2 s — the v2 relay, engine-fetched.
pub async fn video_relay(
    dash: Arc<Dashboard>,
    mut consumer: Consumer,
    vehicle: String,
    index: u8,
    enabled: Arc<AtomicBool>,
    cancel: CancellationToken,
) {
    let name = names::vehicle_stream(&vehicle, "video/live");
    let mut last_seq: u64 = 0;
    let mut window_start = tokio::time::Instant::now();
    let mut window_bytes: usize = 0;
    let mut window_frames: usize = 0;
    while enabled.load(Ordering::Relaxed) && !cancel.is_cancelled() {
        match fetch_latest(&mut consumer, &name).await {
            Some(payload) if payload.len() > 8 => {
                let seq = u64::from_be_bytes(payload[..8].try_into().expect("8-byte prefix"));
                if seq <= last_seq && seq != 0 {
                    tokio::time::sleep(Duration::from_millis(80)).await;
                    continue;
                }
                last_seq = seq;
                let jpeg = &payload[8..];
                window_bytes += jpeg.len();
                window_frames += 1;
                let mut frame = Vec::with_capacity(1 + jpeg.len());
                frame.push(index);
                frame.extend_from_slice(jpeg);
                dash.hub.broadcast_binary(frame);
                let elapsed = window_start.elapsed().as_secs_f64();
                if elapsed >= 2.0 {
                    dash.hub.broadcast(&json!({
                        "type": "video_stats",
                        "vehicle": vehicle,
                        "fps": ((window_frames as f64 / elapsed) * 10.0).round() / 10.0,
                        "kbps": (window_bytes as f64 * 8.0 / elapsed / 1000.0).round(),
                        "seq": seq,
                    }));
                    window_start = tokio::time::Instant::now();
                    window_bytes = 0;
                    window_frames = 0;
                }
            }
            _ => {
                // Stream gap (producer restarting, radio loss): brief pause,
                // then re-poll — the next success is the live frame.
                tokio::time::sleep(Duration::from_millis(150)).await;
            }
        }
    }
    debug!(vehicle, "video relay stopped");
}

// ───────────────────────────── artifacts ────────────────────────────────────

/// Fetch an artifact object and split the `MUASFRAME1` container into
/// `(body, declared content type)` — the `/artifact?name=` backend.
pub async fn fetch_artifact(mut consumer: Consumer, name: &str) -> Option<(Vec<u8>, String)> {
    let parsed: Name = name.parse().ok()?;
    let payload = tokio::time::timeout(Duration::from_secs(15), consumer.fetch_object(parsed))
        .await
        .ok()?
        .ok()?;
    match FrameHeader::split_frame(&payload) {
        Ok((header, body)) => Some((body.to_vec(), header.kind)),
        Err(err) => {
            debug!(name, %err, "artifact is not a MUAS frame container");
            None
        }
    }
}
