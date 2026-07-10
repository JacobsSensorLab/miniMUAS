//! uas-console dogfood: the dashboard's track + vehicle-tile view models
//! are built and rendered THROUGH the console `Binder`
//! (match → authorize → instantiate) on every telemetry sample, and the
//! instrument descriptor is served at `/instrument.json`.
//!
//! The parity UI does not depend on these render outputs (the ported
//! frontend keeps drawing from the raw v2 message schema), but the binding
//! genuinely runs: per-vehicle instance manifests of the fleet
//! `telemetry-sample` kind are published into the console pack's frozen
//! DAG, and each sample renders `track.live` + `panel.vehicle` through a
//! matched, authorized `Binding`. Outputs are exposed at `/views`.

use std::collections::HashMap;
use std::sync::Mutex;

use serde_json::{json, Value};
use uas_console::registry::RenderCtx;
use uas_console::view::{TrackView, VehicleTile, ViewModel};
use uas_console::{
    builtin_registry, console_instrument, console_pack, Binder, Budget, ConsolePack,
    RendererRegistry,
};
use uas_fleet_data::kinds::TelemetrySample;

use muas_contracts::names;

struct LensInner {
    tracks: HashMap<String, TrackView>,
    /// Latest render outputs per vehicle: `{ "track.live": .., "panel.vehicle": .. }`.
    outputs: HashMap<String, Value>,
    /// Sensor chips folded into the tile (from CapabilityProfile extras).
    sensors: HashMap<String, Vec<String>>,
}

/// The console pack + registry + per-vehicle instance manifests, ready to
/// bind.
pub struct LensHost {
    pack: ConsolePack,
    registry: RendererRegistry,
    manifests: HashMap<String, manifest::Hash>,
    namespaces: Vec<String>,
    inner: Mutex<LensInner>,
}

impl LensHost {
    /// Assemble the pack and publish one `telemetry-sample` instance
    /// manifest per vehicle (the shape every publisher of a fleet kind
    /// emits alongside its stream).
    pub fn new(vehicles: &[String]) -> Self {
        let mut pack = console_pack();
        let telemetry_kind = uas_console::intents::fleet_terms().telemetry_sample;
        let mut manifests = HashMap::new();
        let mut namespaces = Vec::new();
        for vid in vehicles {
            let subject = names::vehicle_stream(vid, "telemetry/live");
            let hash = pack
                .publish(&uas_console::intents::kind_instance(telemetry_kind, &subject))
                .expect("instance manifest encodes");
            manifests.insert(vid.clone(), hash);
            namespaces.push(subject);
        }
        Self {
            pack,
            registry: builtin_registry(),
            manifests,
            namespaces,
            inner: Mutex::new(LensInner {
                tracks: HashMap::new(),
                outputs: HashMap::new(),
                sensors: HashMap::new(),
            }),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, LensInner> {
        self.inner.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Record a capability advertisement (sensor chips on the tile).
    pub fn set_sensors(&self, vehicle: &str, sensors: Vec<String>) {
        self.lock().sensors.insert(vehicle.to_string(), sensors);
    }

    /// Fold one telemetry sample: update the track ring, grade the tile,
    /// then bind + render both lenses. Bind failures are surfaced in the
    /// `/views` payload — never a silent fallback (that is the point of
    /// the dogfood).
    pub fn on_sample(&self, vehicle: &str, sample: &TelemetrySample, now_ns: u64) {
        let Some(&manifest) = self.manifests.get(vehicle) else { return };

        let (track_vm, tile_vm) = {
            let mut inner = self.lock();
            let track = inner
                .tracks
                .entry(vehicle.to_string())
                .or_insert_with(|| TrackView::new(vehicle));
            track.push_sample(sample);
            let track_vm = ViewModel::Track(track.clone());
            let mut tile = VehicleTile::from_sample(sample, now_ns);
            if let Some(sensors) = inner.sensors.get(vehicle) {
                tile.sensors = sensors.clone();
            }
            (track_vm, ViewModel::Vehicle(tile))
        };

        let binder = Binder::new(&self.registry);
        let contracts = self.pack.contracts();
        let frontier = self.pack.frontier();
        let ctx = RenderCtx { now_ns };
        let mut out = json!({});
        for (intent, vm) in [
            (uas_console::intents::intent::TRACK_LIVE, &track_vm),
            (uas_console::intents::intent::PANEL_VEHICLE, &tile_vm),
        ] {
            let rendered = binder
                .bind(&self.pack.dag, manifest, &contracts, intent, &frontier, Budget::generous())
                .map_err(|e| e.to_string())
                .and_then(|binding| {
                    binding
                        .render(vm, &ctx)
                        .map(|o| {
                            json!({
                                "renderer": o.renderer,
                                "verdict": format!("{:?}", binding.verdict()),
                                "body": o.body,
                            })
                        })
                        .map_err(|e| e.to_string())
                });
            out[intent] = match rendered {
                Ok(v) => v,
                Err(error) => {
                    tracing::warn!(vehicle, intent, %error, "lens bind/render failed");
                    json!({ "error": error })
                }
            };
        }
        self.lock().outputs.insert(vehicle.to_string(), out);
    }

    /// Latest render outputs per vehicle (`GET /views`).
    pub fn views(&self) -> Value {
        let inner = self.lock();
        Value::Object(
            inner
                .outputs
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        )
    }

    /// The instrument descriptor JSON (`GET /instrument.json`).
    pub fn instrument_json(&self) -> String {
        console_instrument(&self.pack, &self.registry, self.namespaces.clone())
            .to_json()
            .unwrap_or_else(|e| format!("{{\"error\":{:?}}}", e.to_string()))
    }
}
