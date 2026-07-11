//! The surface catalog (`GET /catalog.json`, ROUND-3 §3½): this
//! dashboard's manifested self-description — the render contracts it can
//! Express, the WS data kinds it understands, and its surface-native
//! widgets (ready-to-use, zero-authoring building blocks that exist in
//! `assets/dashboard.html` TODAY — nothing aspirational is listed).
//!
//! Anti-drift construction: the document is assembled from the typed
//! registries in this module (never a hand-written JSON blob), the
//! contract entries are read out of uas-console's ACTUAL contract
//! documents (`console_contract()` / `inspect_contract()` clauses, the
//! same bytes the Binder matches against), and two tripwires guard the
//! kind list:
//!
//! 1. a source-scan test extracts every message-type string literal this
//!    crate broadcasts (and every type the embedded frontend dispatches)
//!    and asserts each appears in [`UNDERSTOOD_KINDS`];
//! 2. the hub records every `type` it actually broadcasts at runtime
//!    ([`crate::hub::Hub::observed_types`]) and the smoke test asserts
//!    that set is a subset of the catalog — so a new message type cannot
//!    ship without cataloguing itself.
//!
//! This round the catalog is deliberately read-only: placement/binding UI
//! is builder-mode's milestone; the shelf existing and being honest is
//! the deliverable.

use serde::Serialize;
use serde_json::{json, Value};

/// One WS message kind this surface understands (receives on `/ws` and
/// routes through the frontend's `dispatch()`).
#[derive(Clone, Copy, Debug, Serialize)]
pub struct KindSpec {
    pub kind: &'static str,
    pub description: &'static str,
    /// Surface-native widget ids that render this kind.
    pub widgets: &'static [&'static str],
}

/// One surface-native widget: a ready-to-use building block that exists
/// in the embedded frontend today (`ready` is honest, not aspirational).
#[derive(Clone, Copy, Debug, Serialize)]
pub struct WidgetSpec {
    pub id: &'static str,
    pub name: &'static str,
    /// What it renders, operator-facing.
    pub renders: &'static str,
    /// The WS kind it needs (empty = self-contained / HTTP-fed).
    pub required_kind: &'static str,
    pub ready: bool,
}

/// One render-contract clause, read from the uas-console contract
/// documents this surface actually binds through (see [`crate::lens`]).
#[derive(Clone, Debug, Serialize)]
pub struct ContractEntry {
    pub contract: String,
    pub intent: String,
    /// `express` | `approximate` | `refuse`.
    pub verdict: &'static str,
    /// Native renderer id (empty on refusals).
    pub renderer: String,
    pub source: &'static str,
}

/// Every WS `type` this surface understands. THE single registry — the
/// hub tripwire and the source-scan test both check against it.
pub const UNDERSTOOD_KINDS: &[KindSpec] = &[
    KindSpec {
        kind: "hello",
        description: "Session snapshot on WS connect: roster, enable gates, \
                      capabilities, sensor meta, task queues, mission state, \
                      recording flag, GCS survey, sim capability, catalog_url.",
        widgets: &["panel.vehicle_tile", "panel.video_tile", "panel.queue_strip"],
    },
    KindSpec {
        kind: "telemetry",
        description: "One vehicle's latest TelemetrySample + link age + clock \
                      skew (3–4 Hz, latest-wins).",
        widgets: &["map.vehicles", "panel.vehicle_tile", "map.coverage"],
    },
    KindSpec {
        kind: "telemetry_stale",
        description: "A vehicle's telemetry went silent; markers grey out and \
                      the link tag turns red (honest aging, no extrapolation).",
        widgets: &["map.vehicles", "panel.vehicle_tile"],
    },
    KindSpec {
        kind: "search_status",
        description: "Raster progress: frames captured, leg counter, detect \
                      counters for the mission banner.",
        widgets: &["overlay.mission_banner"],
    },
    KindSpec {
        kind: "raster_preview",
        description: "The raster plan the WUAS would fly (uas-flight's own \
                      geometry — preview == flight): corners, legs, captures, \
                      estimate.",
        widgets: &["map.plan", "map.coverage"],
    },
    KindSpec {
        kind: "event",
        description: "One operational event ({kind, t, …}, optionally \
                      georeferenced): mission/detect/target/command/coord/\
                      sensor/record/system lifecycles.",
        widgets: &[
            "strip.event_log",
            "strip.command_log",
            "overlay.toasts",
            "map.events",
            "map.taskings",
            "map.targets",
            "panel.targets",
        ],
    },
    KindSpec {
        kind: "capabilities",
        description: "A vehicle's advertised sensors + sensor_meta (hfov, DRI \
                      ranges, mounts, audio reach) — the sensor layer renders \
                      only what is advertised.",
        widgets: &["map.sensors", "panel.vehicle_tile"],
    },
    KindSpec {
        kind: "sensor_data",
        description: "One captured artifact registration (frame/audio clip) \
                      with where/when/why provenance.",
        widgets: &["map.data"],
    },
    KindSpec {
        kind: "task_queue",
        description: "One vehicle's ordered task queue snapshot \
                      (TaskQueueStatus): active progress, pending order, split \
                      continuations, finished tail.",
        widgets: &["panel.queue_strip"],
    },
    KindSpec {
        kind: "video_stats",
        description: "Video relay stats per vehicle: fps, kbps, sequence.",
        widgets: &["panel.video_tile"],
    },
    KindSpec {
        kind: "coord",
        description: "A vehicle's cooperative-coordination entries (peer, \
                      mode coop/coop-pending/unco, biases). Understood and \
                      recorded; map rendering is queued (the ⚠ bias chip on \
                      telemetry is the current operator surface).",
        widgets: &[],
    },
    KindSpec {
        kind: "net",
        description: "The deployment's 1 Hz fabric snapshot: per-face link \
                      counters/rates, active link profile, GCS anchor, and \
                      per-prefix rates from the bridge taps (namespace lens).",
        widgets: &[
            "map.net.fields",
            "map.net.pulses",
            "map.net.overlay",
            "panel.namespace_chips",
        ],
    },
    KindSpec {
        kind: "sim_anomalies",
        description: "Virtual-deployment ground truth: placed anomalies \
                      (never present in a real deployment).",
        widgets: &["map.sim_truth"],
    },
];

/// The binary WS channel (not a JSON kind, catalogued separately so the
/// wire surface is fully described).
pub const BINARY_CHANNELS: &[(&str, &str)] = &[(
    "video_frame",
    "[vehicle index byte][jpeg] — live video frames relayed from the \
     vehicle's video/live stream; never recorded (the v2 rule).",
)];

/// Surface-native widgets that exist in the embedded frontend TODAY.
/// This round catalogues the real shelf; no new renderers are built here.
pub const WIDGETS: &[WidgetSpec] = &[
    WidgetSpec {
        id: "map.vehicles",
        name: "Vehicle markers",
        renders: "Heading triangles with smoothed motion (τ≈0.25 s), trails, \
                  stale-grey aging, fleet-framing rings, id + AGL labels.",
        required_kind: "telemetry",
        ready: true,
    },
    WidgetSpec {
        id: "map.plan",
        name: "Raster plan overlay",
        renders: "Area outline, serpentine legs, capture points of the \
                  previewed plan (the exact artifact that executes).",
        required_kind: "raster_preview",
        ready: true,
    },
    WidgetSpec {
        id: "map.coverage",
        name: "Coverage sweep",
        renders: "Planned footprint along the legs + live footprint swept so \
                  far this mission (hfov @ AGL).",
        required_kind: "raster_preview",
        ready: true,
    },
    WidgetSpec {
        id: "map.sensors",
        name: "Sensor coverage",
        renders: "Forward FoV cones with shaded D/R/I bands, nadir ground \
                  quads (attitude-aware when telemetry carries it), audio \
                  omni circles / beam lobes — from advertised sensor_meta only.",
        required_kind: "capabilities",
        ready: true,
    },
    WidgetSpec {
        id: "map.events",
        name: "Event marks",
        renders: "Georeferenced event diamonds fading with age, hover \
                  tooltip, click-to-locate from the log.",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "map.data",
        name: "Captured-data glyphs",
        renders: "Squares = frames, circles = audio; filled = tasked, hollow \
                  = mission evidence; click plays the artifact.",
        required_kind: "sensor_data",
        ready: true,
    },
    WidgetSpec {
        id: "map.taskings",
        name: "Sensor-tasking glyphs",
        renders: "Watchpoint rings + bullseye pins with inspect/cancel \
                  popover (scoped abort, no RTL).",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "map.targets",
        name: "Target markers",
        renders: "Mission targets by status (queued/investigating/done/\
                  failed) + hollow unconfirmed candidates with promote/\
                  dismiss popover.",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "map.sim_truth",
        name: "Sim ground truth",
        renders: "Placed anomaly markers (visual ⊗ / audio arcs) with \
                  select-to-delete; virtual deployments only.",
        required_kind: "sim_anomalies",
        ready: true,
    },
    WidgetSpec {
        id: "map.net.fields",
        name: "Fabric coverage fields",
        renders: "Soft per-node coverage/activity fields (radius+brightness \
                  track recent traffic, log-scaled, clamped); overlap blends \
                  brighter = shared medium/contention. The R2a default view \
                  of the medium.",
        required_kind: "net",
        ready: true,
    },
    WidgetSpec {
        id: "map.net.pulses",
        name: "Emission pulses",
        renders: "Attributable expanding rings at the emitter, rate-limited \
                  to ~3/s per node; busier emission rates fold into field \
                  intensity instead of visual static.",
        required_kind: "net",
        ready: true,
    },
    WidgetSpec {
        id: "map.net.overlay",
        name: "Overlay bearers",
        renders: "The demoted phase-1 per-pair lines: thin, squared ends, \
                  explicitly an overlay (UDP faces), never the medium. OFF \
                  by default.",
        required_kind: "net",
        ready: true,
    },
    WidgetSpec {
        id: "panel.vehicle_tile",
        name: "Vehicle tile",
        renders: "Per-vehicle chip grid: mode (+⚠ bias), AGL/rangefinder, \
                  battery grade, armed, task (scoped ✕), source, clock Δ, \
                  link age tag; enable gate, takeoff, RTL/Land/Hold, \
                  companion power-off.",
        required_kind: "telemetry",
        ready: true,
    },
    WidgetSpec {
        id: "panel.queue_strip",
        name: "Task-queue strip",
        renders: "Active task bar (progress/eta, scoped ✕), pending chips in \
                  run order (drag to reorder, split warnings), collapsed \
                  finished tail.",
        required_kind: "task_queue",
        ready: true,
    },
    WidgetSpec {
        id: "panel.video_tile",
        name: "Video tile",
        renders: "Live JPEG stream per vehicle with enable toggle and \
                  fps/kbps/seq stats.",
        required_kind: "video_stats",
        ready: true,
    },
    WidgetSpec {
        id: "panel.targets",
        name: "Detection panel",
        renders: "Target cards with per-sensor job chips (queue linkage, \
                  cancel), unconfirmed-candidate dispositions, trigger-frame \
                  thumbnail.",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "panel.namespace_chips",
        name: "Namespace lens chips",
        renders: "Top name prefixes by measured traffic; selecting one \
                  colors the network fields/pulses by that namespace.",
        required_kind: "net",
        ready: true,
    },
    WidgetSpec {
        id: "strip.event_log",
        name: "Event log strip",
        renders: "Filterable event log (category chips, vehicle, text; \
                  chips=union, others intersect), frame-ordered detect \
                  lines, click-to-locate.",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "strip.command_log",
        name: "Command mini-log",
        renders: "Per-command lifecycle (sent → acked/refused → outcome) \
                  with scoped-cancel ✕ on abortable entries.",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "overlay.toasts",
        name: "Lifecycle toasts",
        renders: "Transient command/sensor lifecycle toasts (ok/warn/err).",
        required_kind: "event",
        ready: true,
    },
    WidgetSpec {
        id: "overlay.mission_banner",
        name: "Mission banner",
        renders: "Frames / leg / detects / targets counters over the map.",
        required_kind: "search_status",
        ready: true,
    },
    WidgetSpec {
        id: "overlay.legend",
        name: "Adaptive legend",
        renders: "Symbology of exactly the layers currently on, including \
                  the network sub-layers and active namespace lens.",
        required_kind: "",
        ready: true,
    },
    WidgetSpec {
        id: "transport.replay",
        name: "Replay transport",
        renders: "Deterministic scrub over a recorded session through the \
                  same dispatch handlers as live (fed by /replays, not WS).",
        required_kind: "",
        ready: true,
    },
    WidgetSpec {
        id: "panel.display",
        name: "Display config",
        renders: "Per-layer opacities, imagery veil, icon/line scaling, \
                  network sub-layer toggles; persisted.",
        required_kind: "",
        ready: true,
    },
    WidgetSpec {
        id: "panel.catalog",
        name: "Catalog browser",
        renders: "This catalog: searchable widgets/kinds/contracts with \
                  per-entry detail popovers (read-only this round).",
        required_kind: "",
        ready: true,
    },
];

/// HTTP endpoints this surface serves (matches `server::router`).
pub const ENDPOINTS: &[(&str, &str)] = &[
    ("/", "embedded single-page console"),
    ("/ws", "the everything-WebSocket (kinds above + binary video)"),
    ("/preview", "POST: raster preview from uas-flight's own geometry"),
    ("/tiles/{z}/{x}/{y}", "satellite tile cache (+optional proxy)"),
    ("/replays", "recording index"),
    ("/replays/{name}", "one recording (uas-console JSONL)"),
    ("/artifact", "NDN artifact fetch by name"),
    ("/instrument.json", "uas-console instrument descriptor"),
    ("/views", "latest Binder-rendered track/tile outputs"),
    ("/catalog.json", "this surface catalog"),
];

/// The kind names alone (tripwire assertions compare against this).
pub fn kind_names() -> Vec<&'static str> {
    UNDERSTOOD_KINDS.iter().map(|k| k.kind).collect()
}

/// Contract entries read from the uas-console contract DOCUMENTS this
/// surface binds through — derived from the same canonical bytes the
/// Binder matches, so the catalog cannot drift from the real contracts.
pub fn contract_entries() -> Vec<ContractEntry> {
    use manifest::model::{Clause, Document, Via};
    let mut out = Vec::new();
    for built in [
        uas_console::intents::console_contract(),
        uas_console::intents::inspect_contract(),
    ] {
        let Document::Contract(c) = &built.document else { continue };
        for clause in &c.clauses {
            let (intent, verdict, via) = match clause {
                Clause::Express { intent, via, .. } => (intent, "express", via.as_ref()),
                Clause::Approximate { intent, via, .. } => (intent, "approximate", via.as_ref()),
                Clause::Refuse { intent } => (intent, "refuse", None),
            };
            let renderer = match via {
                Some(Via::Native(id)) => id.clone(),
                _ => String::new(),
            };
            out.push(ContractEntry {
                contract: c.label.clone(),
                intent: intent.name.clone(),
                verdict,
                renderer,
                source: "uas-console",
            });
        }
    }
    out
}

/// Assemble the catalog document from the typed registries above.
pub fn catalog(vehicles: &[String]) -> Value {
    json!({
        "type": "surface_catalog",
        "surface": "muas-dashboard",
        "version": env!("CARGO_PKG_VERSION"),
        "generated": "server-side from the typed registry (catalog.rs) and \
                      uas-console contract documents — not a hand-written blob",
        "vehicles": vehicles,
        "understood_kinds": UNDERSTOOD_KINDS,
        "binary_channels": BINARY_CHANNELS
            .iter()
            .map(|(id, description)| json!({ "id": id, "description": description }))
            .collect::<Vec<_>>(),
        "widgets": WIDGETS,
        "contracts": contract_entries(),
        "endpoints": ENDPOINTS
            .iter()
            .map(|(path, description)| json!({ "path": path, "description": description }))
            .collect::<Vec<_>>(),
        "notes": {
            "read_only": "browsing/help only this round — placement and \
                          binding UI is builder-mode's milestone",
            "drift_tripwires": [
                "source-scan test: every `\"type\": \"…\"` this crate emits \
                 and every `m.type === \"…\"` the frontend dispatches must \
                 appear in understood_kinds",
                "runtime: hub.observed_types() ⊆ understood_kinds \
                 (asserted by the smoke test)"
            ],
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    /// Extract every string literal that follows a `"type":` key (Rust
    /// `json!` sources) from `text`.
    fn json_type_literals(text: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut rest = text;
        while let Some(at) = rest.find("\"type\"") {
            rest = &rest[at + "\"type\"".len()..];
            let after = rest.trim_start();
            let Some(after) = after.strip_prefix(':') else { continue };
            let after = after.trim_start();
            let Some(after) = after.strip_prefix('"') else { continue };
            if let Some(end) = after.find('"') {
                out.push(after[..end].to_string());
            }
        }
        out
    }

    /// Extract every `m.type === "…"` comparison from the frontend.
    fn dispatch_type_literals(text: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut rest = text;
        while let Some(at) = rest.find("m.type === \"") {
            rest = &rest[at + "m.type === \"".len()..];
            if let Some(end) = rest.find('"') {
                out.push(rest[..end].to_string());
            }
        }
        out
    }

    #[test]
    fn drift_tripwire_every_emitted_type_is_catalogued() {
        let kinds: BTreeSet<&str> = kind_names().into_iter().collect();
        let src_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/src");
        let mut scanned = 0;
        for entry in std::fs::read_dir(src_dir).expect("src dir") {
            let path = entry.expect("dir entry").path();
            if path.extension().and_then(|e| e.to_str()) != Some("rs") {
                continue;
            }
            let text = std::fs::read_to_string(&path).expect("source readable");
            for t in json_type_literals(&text) {
                // The catalog document's own type tag is not a WS kind.
                if t == "surface_catalog" {
                    continue;
                }
                assert!(
                    kinds.contains(t.as_str()),
                    "`{}` broadcasts type \"{t}\" — add it to catalog::UNDERSTOOD_KINDS",
                    path.display()
                );
                scanned += 1;
            }
        }
        assert!(scanned >= 10, "the scan found the known emitters ({scanned})");
    }

    #[test]
    fn drift_tripwire_frontend_dispatch_is_catalogued_and_covered() {
        let kinds: BTreeSet<&str> = kind_names().into_iter().collect();
        let html = include_str!("../assets/dashboard.html");
        let dispatched: BTreeSet<String> =
            dispatch_type_literals(html).into_iter().collect();
        assert!(
            dispatched.len() >= 10,
            "dispatch() handles the known kinds ({dispatched:?})"
        );
        for t in &dispatched {
            assert!(
                kinds.contains(t.as_str()),
                "frontend dispatches \"{t}\" — add it to catalog::UNDERSTOOD_KINDS"
            );
        }
        // And the reverse gap is DECLARED, not hidden: any catalogued kind
        // the frontend does not dispatch must list no rendering widgets
        // (today: `coord`, whose map rendering is queued).
        for k in UNDERSTOOD_KINDS {
            if !dispatched.contains(k.kind) && k.kind != "hello" {
                assert!(
                    k.widgets.is_empty(),
                    "kind \"{}\" claims widgets but the frontend never dispatches it",
                    k.kind
                );
            }
        }
    }

    #[test]
    fn widget_kind_references_resolve_and_widgets_are_ready() {
        let kinds: BTreeSet<&str> = kind_names().into_iter().collect();
        let widget_ids: BTreeSet<&str> = WIDGETS.iter().map(|w| w.id).collect();
        assert_eq!(widget_ids.len(), WIDGETS.len(), "widget ids unique");
        for w in WIDGETS {
            assert!(w.ready, "only ready-today widgets are catalogued: {}", w.id);
            if !w.required_kind.is_empty() {
                assert!(
                    kinds.contains(w.required_kind),
                    "widget {} requires unknown kind {}",
                    w.id,
                    w.required_kind
                );
            }
        }
        for k in UNDERSTOOD_KINDS {
            for wid in k.widgets {
                assert!(
                    widget_ids.contains(wid),
                    "kind {} references unknown widget {wid}",
                    k.kind
                );
            }
        }
    }

    #[test]
    fn contracts_derive_from_the_real_console_documents() {
        let entries = contract_entries();
        let express: Vec<(&str, &str)> = entries
            .iter()
            .filter(|e| e.verdict == "express")
            .map(|e| (e.intent.as_str(), e.renderer.as_str()))
            .collect();
        // The console's built-in lenses, exactly as the contract declares.
        assert!(express.contains(&("track.live", "console.track.live")));
        assert!(express.contains(&("panel.vehicle", "console.panel.vehicle")));
        assert!(express.contains(&("raw.inspect", "console.raw.inspect")));
        // The honest refusal is part of the catalog too.
        assert!(entries
            .iter()
            .any(|e| e.intent == "track.predict" && e.verdict == "refuse"));
    }

    #[test]
    fn catalog_document_assembles_from_typed_data() {
        let doc = catalog(&["wuas-01".into(), "iuas-01".into()]);
        assert_eq!(doc["type"], "surface_catalog");
        assert_eq!(
            doc["understood_kinds"].as_array().unwrap().len(),
            UNDERSTOOD_KINDS.len()
        );
        assert_eq!(doc["widgets"].as_array().unwrap().len(), WIDGETS.len());
        assert!(!doc["contracts"].as_array().unwrap().is_empty());
        assert!(doc["endpoints"]
            .as_array()
            .unwrap()
            .iter()
            .any(|e| e["path"] == "/catalog.json"));
    }
}
