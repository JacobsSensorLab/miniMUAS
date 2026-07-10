//! Fleet coordination wiring: PeerGuard + SmartRtl (uas-fleet-node) driven on
//! a dedicated thread, with engine-backed transport closures.
//!
//! # Threading model
//!
//! `PeerGuard`/`SmartRtl` hold non-`Send` callback boxes, so (like the v2
//! Python coordination thread) they live on ONE dedicated OS thread ticking at
//! ~2 Hz. The injected `CoordTransport` closures are sync; the network legs
//! are bridged to the async engine side:
//!
//! - **fetch**: the closure enqueues a fetch request (peer + kind) to the
//!   async [`peer_fetcher`] task and returns the freshest cached sample. The
//!   PeerGuard's adaptive schedule therefore still drives *wire* polling
//!   cadence exactly (one request per closure call); results land one tick
//!   (≤0.5 s) later than a blocking fetch would — an accepted deviation from
//!   the blocking v2 fetch, documented here.
//! - **publish**: the closure serializes our coord entries into the shared
//!   latest-wins buffer that the engine-side `Node::serve` handler answers
//!   from — on NDN, publishing latest-wins data IS making it fetchable.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;
use ndn_app::Consumer;
use ndn_packet::encode::InterestBuilder;
use ndn_packet::Name;
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;
use tracing::{debug, info, warn};
use uas_fleet_node::coordination::{
    CoordStatus, CoordTransport, FlightControl, PeerGuard, PeerGuardConfig, PeerTelemetry,
    SmartRtl, SmartRtlConfig, SmartRtlStatus,
};

use crate::{lock, AgentShared, BackendExt, SharedBackend};

/// Coordination tick period (~2 Hz; the active-conflict poll cap is 0.5 s so
/// nothing in PeerGuard wants a faster clock).
const COORD_TICK: Duration = Duration::from_millis(500);

/// Peer-data fetch Interest lifetime (latest-wins samples go stale fast; a
/// short lifetime keeps the fetcher queue drained at 2 Hz polling).
const FETCH_LIFETIME: Duration = Duration::from_millis(600);

/// What the coordination thread asks the async fetcher for.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum FetchReq {
    Telemetry(String),
    Coord(String),
}

/// Freshest fetched peer data, shared between the fetcher task (writer) and
/// the coordination thread's transport closures (readers).
#[derive(Default)]
pub struct PeerCaches {
    pub telemetry: std::sync::Mutex<HashMap<String, PeerTelemetry>>,
    pub coord: std::sync::Mutex<HashMap<String, CoordStatus>>,
}

/// Commands into the coordination thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtlCommand {
    /// Start slot-layered smart RTL.
    Engage,
    /// Cancel an in-flight smart RTL (land/hold took over).
    Abort,
}

/// Adapter: the coordination layer's [`FlightControl`] over the agent's
/// shared flight backend (holding the whole [`AgentShared`] so `home()` can
/// fall back to the agent-side capture from the lock-per-poll takeoff).
pub struct BackendFlightControl {
    backend: SharedBackend,
    shared: Arc<AgentShared>,
}

impl BackendFlightControl {
    pub fn new(shared: Arc<AgentShared>) -> Self {
        Self {
            backend: shared.backend.clone(),
            shared,
        }
    }
}

impl FlightControl for BackendFlightControl {
    fn position(&self) -> (f64, f64, f64) {
        lock(&self.backend)
            .as_dyn_ref()
            .position()
            .unwrap_or((0.0, 0.0, 0.0))
    }

    fn velocity_ne(&self) -> (f64, f64) {
        lock(&self.backend).as_dyn_ref().velocity_ne()
    }

    fn set_alt_bias(&mut self, bias_m: f64) {
        lock(&self.backend).as_dyn().set_alt_bias(bias_m);
    }

    fn avoid_bias(&self) -> f64 {
        lock(&self.backend).as_dyn_ref().avoid_bias()
    }

    fn home(&self) -> Option<(f64, f64)> {
        self.shared.home()
    }

    fn rtl(&mut self) -> bool {
        lock(&self.backend).as_dyn().rtl()
    }

    fn land(&mut self) -> bool {
        lock(&self.backend).as_dyn().land()
    }

    fn goto(&mut self, lat_deg: f64, lon_deg: f64, agl_m: f64) {
        lock(&self.backend).as_dyn().goto(lat_deg, lon_deg, agl_m, None);
    }

    fn at_target(&self, lat_deg: f64, lon_deg: f64, agl_m: f64, tol_m: f64) -> bool {
        lock(&self.backend)
            .as_dyn_ref()
            .at_target(lat_deg, lon_deg, agl_m, tol_m)
    }

    fn telemetry_mode(&self) -> String {
        lock(&self.backend).as_dyn_ref().telemetry().mode
    }
}

/// Async side: serially serve fetch requests from the coordination thread
/// over the engine (`Consumer::fetch` with MustBeFresh so content-store
/// copies of stale latest-wins data are skipped), updating [`PeerCaches`].
pub async fn peer_fetcher(
    mut consumer: Consumer,
    caches: Arc<PeerCaches>,
    mut rx: mpsc::UnboundedReceiver<FetchReq>,
    cancel: CancellationToken,
) {
    loop {
        let first = tokio::select! {
            _ = cancel.cancelled() => break,
            req = rx.recv() => match req { Some(r) => r, None => break },
        };
        // Coalesce the burst: PeerGuard may have queued several requests in
        // one tick (or while a slow fetch was in flight) — fetch each unique
        // target once.
        let mut batch: Vec<FetchReq> = vec![first];
        let mut seen: HashSet<FetchReq> = batch.iter().cloned().collect();
        while let Ok(req) = rx.try_recv() {
            if seen.insert(req.clone()) {
                batch.push(req);
            }
        }
        for req in batch {
            let (peer, stream) = match &req {
                FetchReq::Telemetry(peer) => (peer.clone(), "telemetry/live"),
                FetchReq::Coord(peer) => (peer.clone(), "coord/status"),
            };
            let name: Name = match muas_contracts::names::vehicle_stream(&peer, stream).parse() {
                Ok(name) => name,
                Err(_) => continue,
            };
            let fetched = consumer
                .fetch_with(
                    InterestBuilder::new(name)
                        .must_be_fresh()
                        .lifetime(FETCH_LIFETIME),
                )
                .await;
            let Ok(data) = fetched else {
                debug!(peer, stream, "peer fetch failed (peer offline or stale)");
                continue;
            };
            let Some(content) = data.content() else { continue };
            match req {
                FetchReq::Telemetry(peer) => {
                    match serde_json::from_slice::<PeerTelemetry>(content) {
                        Ok(sample) => {
                            lock(&caches.telemetry).insert(peer, sample);
                        }
                        Err(err) => debug!(peer, %err, "bad peer telemetry payload"),
                    }
                }
                FetchReq::Coord(peer) => match serde_json::from_slice::<CoordStatus>(content) {
                    Ok(status) => {
                        lock(&caches.coord).insert(peer, status);
                    }
                    Err(err) => debug!(peer, %err, "bad peer coord payload"),
                },
            }
        }
    }
}

/// Configuration handed to the coordination thread.
pub struct CoordThreadConfig {
    pub vehicle_id: String,
    pub peer_ids: Vec<String>,
    pub fleet_ids: Vec<String>,
    pub guard: PeerGuardConfig,
    pub rtl: SmartRtlConfig,
}

/// Spawn the coordination thread. Owns PeerGuard (+ any active SmartRtl),
/// ticks both at ~2 Hz until `cancel`.
pub fn spawn_coord_thread(
    config: CoordThreadConfig,
    shared: Arc<AgentShared>,
    caches: Arc<PeerCaches>,
    fetch_tx: mpsc::UnboundedSender<FetchReq>,
    rtl_rx: std::sync::mpsc::Receiver<RtlCommand>,
    cancel: CancellationToken,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name(format!("coord-{}", config.vehicle_id))
        .spawn(move || coord_thread_main(config, shared, caches, fetch_tx, rtl_rx, cancel))
        .expect("spawn coordination thread")
}

fn coord_thread_main(
    config: CoordThreadConfig,
    shared: Arc<AgentShared>,
    caches: Arc<PeerCaches>,
    fetch_tx: mpsc::UnboundedSender<FetchReq>,
    rtl_rx: std::sync::mpsc::Receiver<RtlCommand>,
    cancel: CancellationToken,
) {
    let span = tracing::info_span!("mission", vehicle_id = %config.vehicle_id);
    let _entered = span.enter();

    // -- transport closures (built here so the non-Send boxes never cross a
    //    thread boundary) --
    let transport = {
        let (caches_t, caches_c) = (caches.clone(), caches.clone());
        let (tx_t, tx_c) = (fetch_tx.clone(), fetch_tx);
        let publish_shared = shared.clone();
        CoordTransport {
            fetch_telemetry: Box::new(move |peer: &str| {
                let _ = tx_t.send(FetchReq::Telemetry(peer.to_string()));
                lock(&caches_t.telemetry).get(peer).copied()
            }),
            fetch_coord: Box::new(move |peer: &str| {
                let _ = tx_c.send(FetchReq::Coord(peer.to_string()));
                lock(&caches_c.coord).get(peer).cloned()
            }),
            publish_coord: Box::new(move |entries: CoordStatus| {
                match serde_json::to_vec(&entries) {
                    Ok(bytes) => *lock(&publish_shared.latest_coord) = Bytes::from(bytes),
                    Err(err) => warn!(%err, "coord: entries failed to encode"),
                }
            }),
        }
    };

    let journal = shared.journal.clone();
    let on_guard_event = Box::new(move |event: uas_fleet_node::coordination::CoordEvent| {
        tracing::info_span!("coord-event", kind = %event.kind).in_scope(|| {
            journal.event(
                &event.kind.clone(),
                serde_json::to_value(&event).unwrap_or_default(),
            );
        });
    });

    let mut guard = PeerGuard::new(
        config.vehicle_id.clone(),
        config.peer_ids.clone(),
        transport,
        config.guard,
    )
    .with_on_event(on_guard_event);

    let mut flight = BackendFlightControl::new(shared.clone());
    let mut rtl: Option<SmartRtl> = None;
    let t0 = std::time::Instant::now();

    info!(peers = ?config.peer_ids, "coordination thread up");
    while !cancel.is_cancelled() {
        let now_s = t0.elapsed().as_secs_f64();

        // Commands from the service layer.
        while let Ok(cmd) = rtl_rx.try_recv() {
            match cmd {
                RtlCommand::Engage => {
                    if rtl.is_none() {
                        let journal = shared.journal.clone();
                        let smart = SmartRtl::new(
                            &config.vehicle_id,
                            config.fleet_ids.iter().cloned(),
                            config.rtl,
                        )
                        .with_on_event(Box::new(move |event| {
                            tracing::info_span!("coord-event", kind = %event.kind).in_scope(
                                || {
                                    journal.event(
                                        &event.kind.clone(),
                                        serde_json::to_value(&event).unwrap_or_default(),
                                    );
                                },
                            );
                        }));
                        info!(slot_agl_m = smart.slot_agl_m(), "smart rtl engaged");
                        rtl = Some(smart);
                    }
                }
                RtlCommand::Abort => {
                    if let Some(active) = rtl.as_mut() {
                        active.cancel();
                    }
                }
            }
        }

        guard.tick(now_s, &mut flight);

        if let Some(active) = rtl.as_mut() {
            if let SmartRtlStatus::Done(outcome) = active.tick(now_s, &mut flight) {
                shared.journal.event(
                    "rtl.done",
                    serde_json::json!({ "outcome": outcome.as_str() }),
                );
                let mut busy = lock(&shared.busy);
                if *busy == "rtl" {
                    busy.clear();
                }
                drop(busy);
                rtl = None;
            }
        }

        std::thread::sleep(COORD_TICK);
    }
    info!("coordination thread down");
}
