//! miniMUAS v3 drone agent library (M3): flight services over
//! ndn-service-core carriers, engine-served latest-wins telemetry, PeerGuard
//! fleet coordination + slot-layered smart RTL, power-loss-safe journals with
//! an optional Block-chain mirror, and an optional ndf-spark telemetry lane.
//!
//! The binary (`main.rs`) is a thin CLI shell; `muas-sim` embeds
//! [`Agent::start`] directly for co-simulation.
//!
//! # Serving mechanism (documented choice)
//!
//! Latest-wins data (`telemetry/live`, `coord/status`) is served with
//! `ndn_app::Node::serve` on an in-process app face
//! (`engine.app_node(..)`): the handler answers every Interest with the
//! freshest sample and stamps no freshness period, so cached copies are
//! immediately stale and MustBeFresh consumers always reach the producer.
//! That is the simplest engine-served latest-wins mechanism in the stack —
//! `AppRuntime::serve` is chain-resume (journal history), not a live sample
//! feed, and SVS publishers add group-sync machinery this point-to-point
//! fleet doesn't need yet.

pub mod config;
pub mod coord;
pub mod journal;
pub mod mission;
pub mod sensor;
pub mod service_impl;
pub mod telemetry;

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use bytes::Bytes;
use muas_contracts::names;
use muas_contracts::policy::AglBounds;
use muas_contracts::services::VehicleServiceDispatch;
use ndn_app::{Consumer, EngineAppExt, ServeGuard};
use ndn_engine::builder::{EngineBuilder, EngineConfig};
use ndn_engine::{ForwarderEngine, ShutdownHandle};
use ndn_face::UdpFace;
use ndn_ndnsf::NdnsfCarrier;
use ndn_packet::Name;
use ndn_rpc::FaceRpcCarrier;
use ndn_service_core::{Carrier, ServiceId};
use ndn_sync::{SvSyncConfig, SvsPubSub};
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn, Instrument};
use uas_fleet_node::coordination::PeerGuardConfig;
use uas_fleet_node::flight_backend::{FlightBackend, SimFlightBackend, SIM_TICK_S};
use uas_flight::deconflict::DeconflictionEnvelope;
use uas_mavlink::MavlinkFlightBackend;

pub use config::{AgentConfig, CarrierKind, Endpoint, ParseOutcome, UdpLink, HELP};
pub use journal::JournalHandle;
pub use sensor::{SensorFeed, SensorFeedConfig, SensorPose, SyntheticFeed};

/// Lock a mutex, recovering from poisoning (a panicked task must not wedge
/// the whole agent — v2's "failures never kill the process" posture).
pub(crate) fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

// ---------------------------------------------------------------------------
// Flight backend seam
// ---------------------------------------------------------------------------

/// How a non-blocking takeoff initiation went (see
/// [`TickableBackend::takeoff_begin`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TakeoffStart {
    /// The vehicle refused to launch (mode/arm/NAV_TAKEOFF refused, ground
    /// check failed, no position fix). The reason is journaled.
    Refused(&'static str),
    /// Already airborne (idempotent), or the climb settled synchronously
    /// (the sim's ensure_airborne is instant in wall time).
    Airborne,
    /// The climb was commanded and is in progress; poll telemetry to watch
    /// it settle. `home` is the launch position captured at the ground arm
    /// (smart-RTL home — stored agent-side because the trait-level backend
    /// only records it on its own blocking takeoff path).
    Climbing { home: Option<(f64, f64)> },
}

/// The agent's backend seam: the uas-fleet-node [`FlightBackend`] surface
/// plus the sim's caller-driven motion tick ([`SimFlightBackend::advance`] is
/// inherent, not on the trait; MAVLink advances itself from the autopilot's
/// telemetry stream, so its tick is a no-op) and a NON-BLOCKING takeoff
/// initiation so the agent can watch the climb lock-per-poll instead of
/// holding the backend mutex for its duration (KNOWN-ISSUES #4).
pub trait TickableBackend: FlightBackend + Send {
    /// Advance the (sim) motion model by `dt` seconds.
    fn advance(&mut self, _dt: f64) {}

    /// Begin a takeoff WITHOUT blocking on the climb: force GUIDED, run the
    /// ground/altitude sanity checks, arm, command NAV_TAKEOFF, return.
    /// The caller (see [`mission::ensure_airborne`]) then polls telemetry
    /// with short locks, keeping the v2 climb-stall and settle ladder.
    fn takeoff_begin(&mut self, agl_m: f64) -> TakeoffStart;
}

impl TickableBackend for SimFlightBackend {
    fn advance(&mut self, dt: f64) {
        SimFlightBackend::advance(self, dt);
    }

    fn takeoff_begin(&mut self, agl_m: f64) -> TakeoffStart {
        // The sim's ensure_airborne advances SIMULATED time internally and
        // returns immediately in wall time — safe under one short lock.
        if FlightBackend::ensure_airborne(self, agl_m) {
            TakeoffStart::Airborne
        } else {
            TakeoffStart::Refused("sim climb did not settle")
        }
    }
}

impl TickableBackend for MavlinkFlightBackend {
    fn takeoff_begin(&mut self, agl_m: f64) -> TakeoffStart {
        // Lock-per-poll restructure of uas-mavlink's `ensure_airborne`
        // command phase (its inherent version blocks through the climb with
        // `&mut self` held). Constants mirror the uas-mavlink ladder.
        const ALREADY_AIRBORNE_AGL_M: f64 = 3.0;
        const GROUND_AGL_TOLERANCE_M: f64 = 1.5;

        let armed = self.link().is_armed();
        // About to launch from here: THIS is home for smart RTL.
        let home = if armed {
            None
        } else {
            MavlinkFlightBackend::position(self).map(|(lat, lon, _)| (lat, lon))
        };
        // GUIDED up front (no-op when already there): a vehicle that
        // finished a previous flight sits in RTL/LAND, where ArduCopter
        // rejects arming and ignores guided targets.
        if !matches!(self.link().set_mode(uas_mavlink::CopterMode::Guided), Ok(true)) {
            return TakeoffStart::Refused("mode change to GUIDED refused");
        }
        let Some((_, _, agl)) = MavlinkFlightBackend::position(self) else {
            return TakeoffStart::Refused("no position telemetry");
        };
        if armed && agl >= ALREADY_AIRBORNE_AGL_M {
            return TakeoffStart::Airborne; // already flying; idempotent
        }
        if !armed {
            // A DISARMED vehicle is on the ground by definition; a nonzero
            // AGL means the altitude estimate is lying (2026-06-15 rail).
            if agl.abs() > GROUND_AGL_TOLERANCE_M {
                return TakeoffStart::Refused("altitude sensor disagrees with ground");
            }
            if !matches!(self.link().arm(), Ok(true)) {
                return TakeoffStart::Refused("arm refused (prearm checks?)");
            }
        }
        if !matches!(self.link().takeoff(agl_m), Ok(true)) {
            return TakeoffStart::Refused("NAV_TAKEOFF refused");
        }
        TakeoffStart::Climbing { home }
    }
}

/// Trait-object helpers so callers can hold the box and still use the
/// `FlightBackend` surface uniformly.
pub trait BackendExt {
    fn as_dyn(&mut self) -> &mut dyn FlightBackend;
    fn as_dyn_ref(&self) -> &dyn FlightBackend;
}

impl BackendExt for Box<dyn TickableBackend> {
    fn as_dyn(&mut self) -> &mut dyn FlightBackend {
        &mut **self
    }
    fn as_dyn_ref(&self) -> &dyn FlightBackend {
        &**self
    }
}

/// The shared flight backend: service handlers, the telemetry loop, and the
/// coordination thread all drive the same boxed backend.
pub type SharedBackend = Arc<Mutex<Box<dyn TickableBackend>>>;

// ---------------------------------------------------------------------------
// Shared agent state
// ---------------------------------------------------------------------------

/// Commands from service handlers into the agent's control plumbing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgentCommand {
    /// Engage slot-layered smart RTL on the coordination thread.
    SmartRtl,
    /// Cancel an in-flight smart RTL (land/hold took over).
    AbortRtl,
    /// Authorized shutdown: flush, cancel everything, exit gracefully.
    Shutdown,
}

/// State shared between service handlers, loops, and the coord thread.
pub struct AgentShared {
    pub vehicle_id: String,
    pub backend: SharedBackend,
    /// Active long-running task label; empty = idle (the v2 busy guard).
    pub busy: Mutex<String>,
    /// The v2 abort flag: raised by rtl/land/hold, cleared when a new task
    /// starts; every mission loop honors it within one control cycle.
    pub abort: std::sync::atomic::AtomicBool,
    pub agl_bounds: AglBounds,
    pub max_range_m: f64,
    pub audio_range_m: f64,
    /// Smart RTL available (vehicle holds a slot in `--fleet-ids`).
    pub smart_rtl: bool,
    /// Freshest published telemetry sample (served under `telemetry/live`).
    pub latest_telemetry: Mutex<Option<Bytes>>,
    /// Freshest coord entries (served under `coord/status`).
    pub latest_coord: Mutex<Bytes>,
    /// Freshest raster progress (served under `search/status`); `None`
    /// until the first raster runs.
    pub latest_search: Mutex<Option<Bytes>>,
    /// CapabilityProfile served under `telemetry/state` (static per boot).
    pub latest_state: Mutex<Option<Bytes>>,
    /// Smart-RTL home captured by the agent's lock-per-poll takeoff (the
    /// MAVLink backend only records home on its own blocking takeoff path).
    pub fallback_home: Mutex<Option<(f64, f64)>>,
    /// Freshest live-video payload (`[8-byte BE seq][jpeg]`) served under
    /// `video/live`; `None` until the video pipeline runs.
    pub latest_video: Mutex<Option<Bytes>>,
    /// Freshest tasked-capture result (v2 `SensorCaptureResult` JSON dict)
    /// served under `sensor/last`.
    pub latest_sensor: Mutex<Option<Bytes>>,
    /// The pluggable sensor feed (`None` = no sensors fitted).
    pub sensor_feed: Option<Arc<dyn sensor::SensorFeed>>,
    /// Mission-artifact publisher (present iff a sensor feed is fitted).
    pub frames: Option<Arc<sensor::FramePublisher>>,
    /// Live video session cancel handle (`video_control` swaps it).
    pub video_session: Mutex<Option<CancellationToken>>,
    /// The agent's root cancel token (ad-hoc capture/video tasks tie their
    /// lifetime to it so shutdown never leaks a renderer loop).
    pub cancel: CancellationToken,
    pub journal: JournalHandle,
    pub commands: mpsc::UnboundedSender<AgentCommand>,
}

impl AgentShared {
    /// Smart-RTL home: the backend's own capture, or the agent-side capture
    /// from [`mission::ensure_airborne`].
    pub fn home(&self) -> Option<(f64, f64)> {
        lock(&self.backend)
            .as_dyn_ref()
            .home()
            .or(*lock(&self.fallback_home))
    }
}

// ---------------------------------------------------------------------------
// Agent bring-up
// ---------------------------------------------------------------------------

/// A running agent. Dropping the handle does NOT stop the agent; call
/// [`AgentHandle::shutdown`] (or cancel its token) for a clean stop.
pub struct AgentHandle {
    pub shared: Arc<AgentShared>,
    pub cancel: CancellationToken,
    pub engine: ForwarderEngine,
    tasks: Vec<tokio::task::JoinHandle<()>>,
    coord_thread: Option<std::thread::JoinHandle<()>>,
    engine_shutdown: Option<ShutdownHandle>,
    _serve_guards: Vec<ServeGuard>,
}

impl AgentHandle {
    /// Resolves when something (ctrl-c handler, shutdown service, embedder)
    /// cancels the agent.
    pub async fn cancelled(&self) {
        self.cancel.cancelled().await;
    }

    /// Cancel + drain: stop loops, join the coordination thread, sync the
    /// journal, shut the engine down.
    pub async fn shutdown(mut self) {
        self.cancel.cancel();
        self.shared.journal.sync().await;
        // Give cancel-aware loops a beat to exit, then abort the rest (the
        // carrier serve loops have no cancel leg of their own).
        tokio::time::sleep(Duration::from_millis(300)).await;
        for task in self.tasks.drain(..) {
            task.abort();
            let _ = task.await;
        }
        if let Some(thread) = self.coord_thread.take() {
            // The coord thread wakes at 2 Hz and observes the cancel.
            let _ = tokio::task::spawn_blocking(move || thread.join()).await;
        }
        if let Some(shutdown) = self.engine_shutdown.take() {
            shutdown.shutdown().await;
        }
    }
}

/// Facade: `Agent::start(config)`.
pub struct Agent;

impl Agent {
    /// Bring the agent up: backend, engine + UDP faces, journal (+ optional
    /// chain mirror), service hosting on the configured carrier, telemetry
    /// loop (+ optional spark lane), peer fetcher + coordination thread.
    pub async fn start(config: AgentConfig) -> Result<AgentHandle, String> {
        let mission = tracing::info_span!("mission", vehicle_id = %config.vehicle_id);
        let cancel = CancellationToken::new();
        let mut tasks = Vec::new();

        // -- flight backend ------------------------------------------------
        let (backend, source): (Box<dyn TickableBackend>, &'static str) = match &config.endpoint {
            Endpoint::Sim { lat_deg, lon_deg } => {
                (Box::new(SimFlightBackend::new(*lat_deg, *lon_deg)), "sim")
            }
            Endpoint::Mavlink(endpoint) => {
                let link = uas_mavlink::LinkConfig::new(config.vehicle_id.clone(), endpoint.clone());
                let mut backend = MavlinkFlightBackend::new(link);
                let endpoint = endpoint.clone();
                let backend = tokio::task::spawn_blocking(move || {
                    match backend.connect(Duration::from_secs(15)) {
                        Ok(identity) => info!(?identity, "mavlink connected"),
                        Err(err) => warn!(%err, endpoint, "mavlink connect failed; flying blind until the link comes up"),
                    }
                    backend
                })
                .await
                .map_err(|e| format!("mavlink connect task panicked: {e}"))?;
                (Box::new(backend), "mavlink")
            }
        };
        let backend: SharedBackend = Arc::new(Mutex::new(backend));

        // -- engine + faces --------------------------------------------------
        let (engine, engine_shutdown) = EngineBuilder::new(EngineConfig::default())
            .build()
            .await
            .map_err(|e| format!("engine build failed: {e}"))?;
        for link in &config.links {
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
        if !config.links.is_empty() {
            // Faces settle on the real clock (compute_socket.rs pattern).
            tokio::time::sleep(Duration::from_millis(150)).await;
        }

        // -- journal (+ optional Block-chain mirror) -------------------------
        let chain = if config.journal_chain {
            let identity = ndf_apps::Identity::new(
                &names::vehicle_prefix(&config.vehicle_id),
                "companion",
                ed25519_dalek::SigningKey::from_bytes(&journal_key_seed(&config.vehicle_id)),
            );
            let runtime = ndf_apps::AppRuntime::attach(engine.clone(), identity, cancel.child_token());
            let address = runtime.identity().chain("journal");
            info!(root = %address.root, "journal chain mirror attached");
            Some(journal::ChainMirror { runtime, address })
        } else {
            None
        };
        let (journal, journal_task) = journal::spawn(
            &config.vehicle_id,
            config.log_dir.clone(),
            config.run_id.clone(),
            chain,
        );
        tasks.push(journal_task);

        // -- sensor feed (pluggable seam; see sensor.rs) ----------------------
        let mut sensor_meta = muas_contracts::sensors::SensorMeta::default();
        let (sensor_feed, frames): (
            Option<Arc<dyn sensor::SensorFeed>>,
            Option<Arc<sensor::FramePublisher>>,
        ) = match &config.sensor_feed {
            sensor::SensorFeedConfig::None => (None, None),
            synth @ sensor::SensorFeedConfig::Synthetic { .. } => {
                let has_audio = config.extras.iter().any(|e| e == "audio");
                let feed = sensor::SyntheticFeed::new(synth, has_audio, config.audio_range_m)
                    .expect("synthetic variant builds a synthetic feed");
                // Ground truth arrives OVER THE NETWORK (bridge + fabric),
                // like every peer stream — no process-local shortcut.
                tasks.push(sensor::spawn_anomaly_fetcher(
                    engine.app_consumer(cancel.child_token()),
                    feed.anomaly_name.clone(),
                    feed.cache(),
                    cancel.clone(),
                ));
                sensor_meta = feed.sensor_meta();
                let node = engine.app_node(cancel.child_token());
                info!(anomaly_name = %feed.anomaly_name, "synthetic sensor feed up");
                (
                    Some(Arc::new(feed) as Arc<dyn sensor::SensorFeed>),
                    Some(Arc::new(sensor::FramePublisher::new(node))),
                )
            }
        };

        // -- shared state ----------------------------------------------------
        let (cmd_tx, mut cmd_rx) = mpsc::unbounded_channel();
        let shared = Arc::new(AgentShared {
            vehicle_id: config.vehicle_id.clone(),
            backend: backend.clone(),
            busy: Mutex::new(String::new()),
            abort: std::sync::atomic::AtomicBool::new(false),
            agl_bounds: config.agl_bounds,
            max_range_m: config.max_range_m,
            audio_range_m: config.audio_range_m,
            smart_rtl: config.smart_rtl_available() && !config.peer_ids().is_empty(),
            latest_telemetry: Mutex::new(None),
            latest_coord: Mutex::new(Bytes::from_static(b"[]")),
            latest_search: Mutex::new(None),
            latest_state: Mutex::new(None),
            fallback_home: Mutex::new(None),
            latest_video: Mutex::new(None),
            latest_sensor: Mutex::new(None),
            sensor_feed,
            frames,
            video_session: Mutex::new(None),
            cancel: cancel.clone(),
            journal: journal.clone(),
            commands: cmd_tx,
        });
        journal.event(
            "agent.up",
            serde_json::json!({
                "source": source,
                "fleet_ids": config.fleet_ids,
                "carrier": if config.carrier == CarrierKind::Rpc { "rpc" } else { "ndnsf" },
            }),
        );

        // -- latest-wins serving (telemetry/live|state, search/status,
        //    coord/status, video/live, sensor/last) ---------------------------
        // telemetry/state is the (static) v2 CapabilityProfile: which
        // investigation extras this vehicle advertises for dispatch — plus
        // the ADDITIVE `sensor_meta` key (muas-contracts::sensors) when a
        // feed is fitted, which the dashboard's sensor layer renders from.
        let capability_bytes: Bytes = {
            let profile = uas_fleet_data::kinds::CapabilityProfile {
                extras: config.extras.clone(),
                gimbal: false,
                gps_time_ns: telemetry::gps_time_ns(),
                mode_control: true,
                obstacle_map: false,
                position: true,
                signal_sensor: false,
                vehicle_id: config.vehicle_id.clone(),
                velocity: false,
                yaw_control: true,
            };
            let mut value =
                serde_json::to_value(&profile).map_err(|e| format!("profile encode: {e}"))?;
            muas_contracts::sensors::merge_into_profile(&mut value, &sensor_meta);
            Bytes::from(serde_json::to_vec(&value).map_err(|e| format!("profile encode: {e}"))?)
        };
        *lock(&shared.latest_state) = Some(capability_bytes);
        let node = engine.app_node(cancel.child_token());
        let mut serve_guards = Vec::new();
        type ReadLatest = fn(&AgentShared) -> Option<Bytes>;
        let streams: [(&str, ReadLatest); 6] = [
            ("telemetry/live", |s| lock(&s.latest_telemetry).clone()),
            ("telemetry/state", |s| lock(&s.latest_state).clone()),
            ("search/status", |s| lock(&s.latest_search).clone()),
            ("coord/status", |s| Some(lock(&s.latest_coord).clone())),
            ("video/live", |s| lock(&s.latest_video).clone()),
            ("sensor/last", |s| lock(&s.latest_sensor).clone()),
        ];
        for (stream, read) in streams {
            let name: Name = names::vehicle_stream(&config.vehicle_id, stream)
                .parse()
                .map_err(|e| format!("bad stream name: {e:?}"))?;
            let shared_for_serve = shared.clone();
            let guard = node
                .serve(name, move |interest, responder| {
                    let latest = read(&shared_for_serve);
                    async move {
                        if let Some(bytes) = latest {
                            let _ = responder.respond((*interest.name).clone(), bytes).await;
                        }
                        // No sample yet: drop the Interest (consumer times out).
                    }
                })
                .await
                .map_err(|e| format!("serve {stream}: {e}"))?;
            serve_guards.push(guard);
        }

        // -- service hosting ---------------------------------------------------
        let svc_prefix: Name = names::vehicle_prefix(&config.vehicle_id)
            .parse()
            .map_err(|e| format!("bad vehicle prefix: {e:?}"))?;
        let svc = ServiceId::new(svc_prefix.clone());
        let dispatch = Arc::new(VehicleServiceDispatch(Arc::new(
            service_impl::VehicleServiceImpl::new(shared.clone()),
        )));
        match config.carrier {
            CarrierKind::Rpc => {
                let producer = engine.register_producer(svc_prefix.clone(), cancel.child_token());
                let server = FaceRpcCarrier::server(producer);
                tasks.push(tokio::spawn(
                    async move {
                        if let Err(err) = server.serve(&svc, dispatch).await {
                            warn!(%err, "rpc service loop ended");
                        }
                    }
                    .instrument(mission.clone()),
                ));
            }
            CarrierKind::Ndnsf => {
                let listen = config
                    .ndnsf_listen
                    .ok_or_else(|| "--carrier ndnsf requires --ndnsf-listen".to_string())?;
                let (carrier, pump_tasks) = ndnsf_carrier_over_udp(
                    listen,
                    config.ndnsf_peers.clone(),
                    &config.vehicle_id,
                    cancel.clone(),
                )
                .await?;
                tasks.extend(pump_tasks);
                tasks.push(tokio::spawn(
                    async move {
                        if let Err(err) = carrier.serve(&svc, dispatch).await {
                            warn!(%err, "ndnsf service loop ended");
                        }
                    }
                    .instrument(mission.clone()),
                ));
            }
        }

        // -- telemetry loop (+ sim motion tick, + spark lane) --------------------
        let mut spark = match config.spark_udp {
            Some(dest) => Some(
                telemetry::SparkLane::bind(dest)
                    .await
                    .map_err(|e| format!("spark lane bind: {e}"))?,
            ),
            None => None,
        };
        {
            let shared = shared.clone();
            let cancel = cancel.clone();
            let hz = config.telemetry_hz.max(0.1);
            tasks.push(tokio::spawn(
                async move {
                    let mut interval =
                        tokio::time::interval(Duration::from_secs_f64(1.0 / hz));
                    loop {
                        tokio::select! {
                            _ = cancel.cancelled() => break,
                            _ = interval.tick() => {}
                        }
                        let _tick = tracing::debug_span!("telemetry-tick").entered();
                        let (snapshot, source) = {
                            let backend = lock(&shared.backend);
                            (backend.as_dyn_ref().telemetry(), backend.as_dyn_ref().source())
                        };
                        let busy = lock(&shared.busy).clone();
                        let sample = telemetry::build_sample(
                            &shared.vehicle_id,
                            &busy,
                            source,
                            &snapshot,
                        );
                        let bytes = match serde_json::to_vec(&sample) {
                            Ok(bytes) => Bytes::from(bytes),
                            Err(_) => continue,
                        };
                        *lock(&shared.latest_telemetry) = Some(bytes.clone());
                        drop(_tick);
                        if let Some(lane) = spark.as_mut() {
                            lane.emit(&bytes).await;
                        }
                    }
                }
                .instrument(mission.clone()),
            ));
        }
        if matches!(config.endpoint, Endpoint::Sim { .. }) {
            let backend = backend.clone();
            let cancel = cancel.clone();
            tasks.push(tokio::spawn(
                async move {
                    let mut interval =
                        tokio::time::interval(Duration::from_secs_f64(SIM_TICK_S));
                    loop {
                        tokio::select! {
                            _ = cancel.cancelled() => break,
                            _ = interval.tick() => {}
                        }
                        lock(&backend).advance(SIM_TICK_S);
                    }
                }
                .instrument(mission.clone()),
            ));
        }

        // -- coordination: peer fetcher + coord thread ----------------------------
        let mut coord_thread = None;
        let mut rtl_tx: Option<std::sync::mpsc::Sender<coord::RtlCommand>> = None;
        if !config.fleet_ids.is_empty() {
            let caches = Arc::new(coord::PeerCaches::default());
            let (fetch_tx, fetch_rx) = mpsc::unbounded_channel();
            let consumer: Consumer = engine.app_consumer(cancel.child_token());
            tasks.push(tokio::spawn(
                coord::peer_fetcher(consumer, caches.clone(), fetch_rx, cancel.clone())
                    .instrument(mission.clone()),
            ));
            let (tx, rx) = std::sync::mpsc::channel();
            rtl_tx = Some(tx);
            coord_thread = Some(coord::spawn_coord_thread(
                coord::CoordThreadConfig {
                    vehicle_id: config.vehicle_id.clone(),
                    peer_ids: config.peer_ids(),
                    fleet_ids: config.fleet_ids.clone(),
                    guard: PeerGuardConfig {
                        envelope: DeconflictionEnvelope {
                            horizontal_sep_m: config.hsep_m,
                            vertical_sep_m: config.vsep_m,
                            horizon_s: config.horizon_s,
                            ..DeconflictionEnvelope::default()
                        },
                        floor_agl_m: config.floor_agl_m,
                        grace_s: config.grace_s,
                        ..PeerGuardConfig::default()
                    },
                    rtl: uas_fleet_node::coordination::SmartRtlConfig {
                        base_agl_m: config.rtl_base_agl_m,
                        separation_m: config.rtl_sep_m,
                    },
                },
                shared.clone(),
                caches,
                fetch_tx,
                rx,
                cancel.clone(),
            ));
        }

        // -- command router -----------------------------------------------------
        {
            let cancel = cancel.clone();
            let journal = journal.clone();
            tasks.push(tokio::spawn(
                async move {
                    loop {
                        let cmd = tokio::select! {
                            _ = cancel.cancelled() => break,
                            cmd = cmd_rx.recv() => match cmd { Some(c) => c, None => break },
                        };
                        match cmd {
                            AgentCommand::SmartRtl => {
                                if let Some(tx) = rtl_tx.as_ref() {
                                    let _ = tx.send(coord::RtlCommand::Engage);
                                }
                            }
                            AgentCommand::AbortRtl => {
                                if let Some(tx) = rtl_tx.as_ref() {
                                    let _ = tx.send(coord::RtlCommand::Abort);
                                }
                            }
                            AgentCommand::Shutdown => {
                                journal.event("agent.shutdown", serde_json::json!({}));
                                cancel.cancel();
                                break;
                            }
                        }
                    }
                }
                .instrument(mission.clone()),
            ));
        }

        info!(parent: &mission, vehicle = %config.vehicle_id, "agent up");
        Ok(AgentHandle {
            shared,
            cancel,
            engine,
            tasks,
            coord_thread,
            engine_shutdown: Some(engine_shutdown),
            _serve_guards: serve_guards,
        })
    }
}

/// Dev-grade signing seed for the journal chain: hashed from the vehicle id
/// and the boot clock. NOT name-derived trust (ndf-apps warns against that) —
/// followers pin the `writer_key` from the ChainAddress we advertise; a real
/// deployment supplies enrolled keys instead.
fn journal_key_seed(vehicle_id: &str) -> [u8; 32] {
    use std::hash::{Hash, Hasher};
    let mut seed = [0u8; 32];
    let boot_ns = telemetry::gps_time_ns();
    for (i, chunk) in seed.chunks_mut(8).enumerate() {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        (vehicle_id, boot_ns, i as u64).hash(&mut hasher);
        chunk.copy_from_slice(&hasher.finish().to_le_bytes());
    }
    seed
}

/// Wire an `NdnsfCarrier` over a UDP group lane: the SVS pub/sub's byte
/// streams pump through one socket to a static peer list (the in-memory hub
/// of ndn-ndnsf's tests, with datagrams instead of channels). `.insecure()`
/// — the fleet trust bundle is a later increment (documented).
async fn ndnsf_carrier_over_udp(
    listen: std::net::SocketAddr,
    peers: Vec<std::net::SocketAddr>,
    vehicle_id: &str,
    cancel: CancellationToken,
) -> Result<(NdnsfCarrier, Vec<tokio::task::JoinHandle<()>>), String> {
    let socket = Arc::new(
        tokio::net::UdpSocket::bind(listen)
            .await
            .map_err(|e| format!("ndnsf udp bind {listen}: {e}"))?,
    );
    let (out_tx, mut out_rx) = mpsc::channel::<Bytes>(256);
    let (in_tx, in_rx) = mpsc::channel::<Bytes>(256);
    let mut tasks = Vec::new();
    {
        let socket = socket.clone();
        let cancel = cancel.clone();
        tasks.push(tokio::spawn(async move {
            loop {
                let msg = tokio::select! {
                    _ = cancel.cancelled() => break,
                    msg = out_rx.recv() => match msg { Some(m) => m, None => break },
                };
                for peer in &peers {
                    let _ = socket.send_to(&msg, peer).await;
                }
            }
        }));
    }
    {
        let socket = socket.clone();
        let cancel = cancel.clone();
        tasks.push(tokio::spawn(async move {
            let mut buf = vec![0u8; 65536];
            loop {
                let received = tokio::select! {
                    _ = cancel.cancelled() => break,
                    r = socket.recv_from(&mut buf) => r,
                };
                match received {
                    Ok((len, _from)) => {
                        if in_tx
                            .send(Bytes::copy_from_slice(&buf[..len]))
                            .await
                            .is_err()
                        {
                            break;
                        }
                    }
                    Err(err) => {
                        warn!(%err, "ndnsf lane recv error");
                        break;
                    }
                }
            }
        }));
    }
    let group: Name = names::GROUP_PREFIX
        .parse()
        .map_err(|e| format!("bad group prefix: {e:?}"))?;
    let node: Name = names::vehicle_prefix(vehicle_id)
        .parse()
        .map_err(|e| format!("bad node name: {e:?}"))?;
    let ps = SvsPubSub::join(group.clone(), node.clone(), out_tx, in_rx, SvSyncConfig::default());
    Ok((NdnsfCarrier::new(ps, node, group).insecure(), tasks))
}
