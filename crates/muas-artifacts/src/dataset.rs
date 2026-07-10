//! The ONE mission dataset every artifact lenses over.
//!
//! A [`MissionDataset`] is run-scoped: the run's *configuration* (the inputs
//! — link profile, coordination knobs, floors, carrier, stack revisions) and
//! its *outcomes* (every event / telemetry / coordination / service / link
//! datum) belong to one `run_id`. Every datum carries the index of the Block
//! it was decoded from, and every Block carries `(content hash, chain root,
//! chain seq)` — provenance is the hash of the named Block the datum came
//! from, never a file copy.
//!
//! Nothing in here is per-artifact: the report, the deck, the demo, and the
//! comparison all read this same structure directly. There are no artifact
//! transformation caches to drift out of sync — that is the "no data silos"
//! demonstration made concrete in code.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// 32-byte content hash (SHA-256 over the signed Block packet).
pub type Hash32 = [u8; 32];

/// Lowercase hex of a content hash.
pub fn hex(hash: &Hash32) -> String {
    let mut s = String::with_capacity(64);
    for b in hash {
        use std::fmt::Write;
        let _ = write!(s, "{b:02x}");
    }
    s
}

/// Decode a lowercase/uppercase hex string into `N` bytes.
pub fn from_hex<const N: usize>(s: &str) -> Result<[u8; N], String> {
    let s = s.trim();
    if s.len() != N * 2 {
        return Err(format!("expected {} hex chars, got {}", N * 2, s.len()));
    }
    let mut out = [0u8; N];
    for (i, chunk) in s.as_bytes().chunks(2).enumerate() {
        let hi = (chunk[0] as char).to_digit(16).ok_or("bad hex digit")?;
        let lo = (chunk[1] as char).to_digit(16).ok_or("bad hex digit")?;
        out[i] = ((hi << 4) | lo) as u8;
    }
    Ok(out)
}

/// One resolved Block's identity: what an artifact cites.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BlockRef {
    /// SHA-256 over the signed packet — the substrate identity.
    pub hash: Hash32,
    /// The chain root the Block lives on (its name).
    pub chain: String,
    /// Position on that chain.
    pub seq: u64,
}

impl BlockRef {
    /// The citation form (hex hash) artifacts embed.
    pub fn citation(&self) -> Citation {
        Citation { hash: hex(&self.hash), chain: self.chain.clone(), seq: self.seq }
    }
}

/// A provenance citation: `(block hash, chain, seq)` in printable form.
/// Two artifacts consuming the same datum MUST produce equal `Citation`s —
/// asserted by test.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Citation {
    /// Lowercase hex content hash.
    pub hash: String,
    /// Chain root name.
    pub chain: String,
    /// Chain sequence.
    pub seq: u64,
}

/// What kind of datum a line decoded into (classification, not schema).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatumKind {
    /// A vehicle state sample (from the dashboard recording stream).
    Telemetry,
    /// A coordination event (`coord.*` journal events, `coord` broadcasts).
    Coord,
    /// A service/flight event (`service.*`, `flight.*` journal events).
    Service,
    /// A smart-RTL event (`rtl.*`).
    Rtl,
    /// Link/network health (`video_stats`, `telemetry_stale`, `spark.*`).
    Link,
    /// Everything else (mission events, agent lifecycle, run config).
    Event,
}

/// One datum + its provenance (`block` indexes [`MissionDataset::blocks`]).
#[derive(Clone, Debug)]
pub struct Datum {
    /// Index into [`MissionDataset::blocks`] — the Block this line came from.
    pub block: usize,
    /// Datum time, ns since the Unix epoch.
    pub t_ns: u64,
    /// Vehicle the datum is about, when stated.
    pub vehicle: Option<String>,
    /// Classification.
    pub kind: DatumKind,
    /// The wire kind/type label (`coord.coop`, `telemetry`, ...).
    pub label: String,
    /// The full decoded line (journal event object, or recording `event`).
    pub body: Value,
}

/// The run's configuration — the *input* side of the input→output
/// association artifacts lead with. Flattened to dotted keys
/// (`link_profile.loss_pct`, `coord.grace_s`, ...). A key on which vehicles
/// disagree is stored per vehicle as `key@vehicle`.
#[derive(Clone, Debug, Default)]
pub struct RunConfig {
    /// Declared run id (from the `run.config` record / per-line `run_id`).
    pub run_id: Option<String>,
    /// Flattened settings.
    pub settings: BTreeMap<String, Value>,
    /// True when no typed `run.config` record was found and the settings
    /// were synthesized from what the journals imply (`agent.up`). Fields
    /// not inferable are simply absent — rendered as unknown, never guessed.
    pub synthesized: bool,
}

impl RunConfig {
    /// A setting rendered for humans; `None` = unknown.
    pub fn display(&self, key: &str) -> Option<String> {
        self.settings.get(key).map(display_value)
    }
}

/// Compact human form of a JSON settings value.
pub fn display_value(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        Value::Array(items) => items
            .iter()
            .map(display_value)
            .collect::<Vec<_>>()
            .join(","),
        other => other.to_string(),
    }
}

/// The one dataset: run config + provenance-carrying data.
#[derive(Clone, Debug, Default)]
pub struct MissionDataset {
    /// The run's input configuration.
    pub run: RunConfig,
    /// Every resolved Block, in resolution order (chain by chain, seq order).
    pub blocks: Vec<BlockRef>,
    /// Every datum, time-sorted after [`MissionDataset::finish`].
    pub data: Vec<Datum>,
    /// Raw per-vehicle config records, pre-merge (vehicle, flattened map).
    cfg_records: Vec<(String, BTreeMap<String, Value>)>,
}

fn flatten(prefix: &str, value: &Value, out: &mut BTreeMap<String, Value>) {
    match value {
        Value::Object(map) => {
            for (k, v) in map {
                let key = if prefix.is_empty() { k.clone() } else { format!("{prefix}.{k}") };
                flatten(&key, v, out);
            }
        }
        other => {
            out.insert(prefix.to_string(), other.clone());
        }
    }
}

fn classify_journal(kind: &str) -> DatumKind {
    if kind.starts_with("coord.") {
        DatumKind::Coord
    } else if kind.starts_with("service.") || kind.starts_with("flight.") {
        DatumKind::Service
    } else if kind.starts_with("rtl.") {
        DatumKind::Rtl
    } else if kind.starts_with("spark.") || kind.starts_with("link.") {
        DatumKind::Link
    } else {
        DatumKind::Event
    }
}

impl MissionDataset {
    /// Empty dataset.
    pub fn new() -> Self {
        Self::default()
    }

    /// Ingest one journal Block (JSONL of agent journal events). Lines that
    /// fail to parse are skipped — a journal is power-loss-safe, its tail
    /// line may be torn.
    pub fn add_journal_block(&mut self, block: BlockRef, payload: &[u8]) {
        let idx = self.push_block(block);
        for line in payload.split(|b| *b == b'\n') {
            if line.iter().all(u8::is_ascii_whitespace) {
                continue;
            }
            let Ok(value) = serde_json::from_slice::<Value>(line) else { continue };
            let kind = value["kind"].as_str().unwrap_or("").to_string();
            let t_ns = value["ts_ns"].as_u64().unwrap_or(0);
            let vehicle = value["vehicle_id"].as_str().map(str::to_string);
            if let Some(run_id) = value["run_id"].as_str() {
                if self.run.run_id.is_none() {
                    self.run.run_id = Some(run_id.to_string());
                }
            }
            if kind == "run.config" {
                self.record_config(vehicle.as_deref().unwrap_or("?"), &value);
            }
            self.data.push(Datum {
                block: idx,
                t_ns,
                vehicle,
                kind: classify_journal(&kind),
                label: kind,
                body: value,
            });
        }
    }

    /// Ingest one dashboard-recording Block (JSONL of uas-console
    /// `RecordedEvent`s: `{"t_ns": .., "event": {"type": ..}}`).
    pub fn add_recording_block(&mut self, block: BlockRef, payload: &[u8]) {
        let idx = self.push_block(block);
        for line in payload.split(|b| *b == b'\n') {
            if line.iter().all(u8::is_ascii_whitespace) {
                continue;
            }
            let Ok(value) = serde_json::from_slice::<Value>(line) else { continue };
            let t_ns = value["t_ns"].as_u64().unwrap_or(0);
            let event = value.get("event").cloned().unwrap_or(Value::Null);
            let ty = event["type"].as_str().unwrap_or("").to_string();
            let (kind, label, vehicle) = match ty.as_str() {
                "telemetry" => (
                    DatumKind::Telemetry,
                    "telemetry".to_string(),
                    event["vehicle"].as_str().map(str::to_string),
                ),
                "coord" => (
                    DatumKind::Coord,
                    "coord".to_string(),
                    event["vehicle"].as_str().map(str::to_string),
                ),
                "video_stats" | "telemetry_stale" => (
                    DatumKind::Link,
                    ty.clone(),
                    event["vehicle"].as_str().map(str::to_string),
                ),
                "event" => (
                    DatumKind::Event,
                    event["kind"].as_str().unwrap_or("event").to_string(),
                    event["vehicle"].as_str().map(str::to_string),
                ),
                other => (DatumKind::Event, other.to_string(), None),
            };
            self.data.push(Datum { block: idx, t_ns, vehicle, kind, label, body: event });
        }
    }

    fn push_block(&mut self, block: BlockRef) -> usize {
        self.blocks.push(block);
        self.blocks.len() - 1
    }

    fn record_config(&mut self, vehicle: &str, line: &Value) {
        if self.run.run_id.is_none() {
            if let Some(id) = line["run_id"]
                .as_str()
                .or_else(|| line["config"]["run_id"].as_str())
            {
                self.run.run_id = Some(id.to_string());
            }
        }
        let mut flat = BTreeMap::new();
        if let Some(config) = line.get("config").filter(|c| c.is_object()) {
            flatten("", config, &mut flat);
        } else if let Some(map) = line.as_object() {
            // Tolerant form: the whole line minus envelope fields.
            for (k, v) in map {
                if matches!(k.as_str(), "kind" | "ts_ns" | "vehicle_id" | "run_id") {
                    continue;
                }
                flatten(k, v, &mut flat);
            }
        }
        flat.remove("run_id");
        self.cfg_records.push((vehicle.to_string(), flat));
    }

    /// Finish ingestion: time-sort the data and merge per-vehicle config
    /// records into the run's settings map. If no `run.config` record was
    /// present, synthesize the inferable subset from `agent.up` and mark
    /// the config synthesized (unknown fields stay unknown).
    pub fn finish(&mut self) {
        self.data.sort_by_key(|d| d.t_ns);
        if self.cfg_records.is_empty() {
            self.synthesize_config();
            return;
        }
        // key -> vehicle -> value
        let mut by_key: BTreeMap<String, BTreeMap<String, Value>> = BTreeMap::new();
        for (vehicle, flat) in &self.cfg_records {
            for (k, v) in flat {
                by_key.entry(k.clone()).or_default().insert(vehicle.clone(), v.clone());
            }
        }
        let n_vehicles = self.cfg_records.iter().map(|(v, _)| v).collect::<BTreeSet<_>>().len();
        for (key, per_vehicle) in by_key {
            let distinct: BTreeSet<String> =
                per_vehicle.values().map(|v| v.to_string()).collect();
            if distinct.len() == 1 && per_vehicle.len() == n_vehicles {
                let v = per_vehicle.into_values().next().expect("non-empty");
                self.run.settings.insert(key, v);
            } else {
                // Vehicles disagree (or some are silent): keep each side.
                for (vehicle, v) in per_vehicle {
                    self.run.settings.insert(format!("{key}@{vehicle}"), v);
                }
            }
        }
    }

    fn synthesize_config(&mut self) {
        self.run.synthesized = true;
        for d in &self.data {
            if d.label != "agent.up" {
                continue;
            }
            if let Some(fleet) = d.body.get("fleet_ids") {
                self.run.settings.entry("fleet_ids".into()).or_insert_with(|| fleet.clone());
                if let Some(n) = fleet.as_array().map(Vec::len) {
                    self.run
                        .settings
                        .entry("fleet_size".into())
                        .or_insert_with(|| Value::from(n as u64));
                }
            }
            if let Some(carrier) = d.body.get("carrier") {
                self.run.settings.entry("carrier".into()).or_insert_with(|| carrier.clone());
            }
            if let Some(source) = d.body.get("source") {
                self.run
                    .settings
                    .entry("endpoint.source".into())
                    .or_insert_with(|| source.clone());
            }
        }
    }

    /// The run's display label: the declared `run_id`, else a
    /// content-derived one (`run-<first block hash prefix>`), so an
    /// unlabeled run is still stably nameable.
    pub fn run_label(&self) -> String {
        match &self.run.run_id {
            Some(id) => id.clone(),
            None => match self.blocks.first() {
                Some(b) => format!("run-{}", &hex(&b.hash)[..8]),
                None => "run-empty".to_string(),
            },
        }
    }

    /// All vehicle ids seen in the data, sorted.
    pub fn vehicles(&self) -> Vec<String> {
        self.data
            .iter()
            .filter_map(|d| d.vehicle.clone())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect()
    }

    /// First datum time (ns), 0 when empty.
    pub fn t0(&self) -> u64 {
        self.data.first().map(|d| d.t_ns).unwrap_or(0)
    }

    /// Last datum time (ns), 0 when empty.
    pub fn t1(&self) -> u64 {
        self.data.last().map(|d| d.t_ns).unwrap_or(0)
    }

    /// Data of one kind, in time order.
    pub fn of_kind(&self, kind: DatumKind) -> impl Iterator<Item = &Datum> {
        self.data.iter().filter(move |d| d.kind == kind)
    }

    /// The citation for one datum's source Block.
    pub fn citation_of(&self, d: &Datum) -> Citation {
        self.blocks[d.block].citation()
    }

    /// Sorted, deduplicated citations of every Block whose data matches the
    /// predicate — the footnote set of a report section.
    pub fn citations_where(&self, mut pred: impl FnMut(&Datum) -> bool) -> Vec<Citation> {
        let idxs: BTreeSet<usize> =
            self.data.iter().filter(|d| pred(d)).map(|d| d.block).collect();
        idxs.into_iter().map(|i| self.blocks[i].citation()).collect()
    }

    /// Every Block's citation (the whole-artifact provenance set).
    pub fn all_citations(&self) -> Vec<Citation> {
        let mut c: Vec<Citation> = self.blocks.iter().map(BlockRef::citation).collect();
        c.sort();
        c.dedup();
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn bref(byte: u8, chain: &str, seq: u64) -> BlockRef {
        BlockRef { hash: [byte; 32], chain: chain.into(), seq }
    }

    #[test]
    fn hex_round_trips() {
        let h = [0xabu8; 32];
        assert_eq!(from_hex::<32>(&hex(&h)).unwrap(), h);
        assert!(from_hex::<32>("zz").is_err());
    }

    #[test]
    fn journal_lines_classify_and_carry_provenance() {
        let mut ds = MissionDataset::new();
        let lines = [
            json!({"kind":"run.config","ts_ns":10,"vehicle_id":"iuas-01","run_id":"r1",
                   "config":{"coord":{"grace_s":2.5},"carrier":"rpc"}}),
            json!({"kind":"agent.up","ts_ns":11,"vehicle_id":"iuas-01","run_id":"r1",
                   "fleet_ids":["iuas-01","wuas-01"],"carrier":"rpc"}),
            json!({"kind":"coord.coop","ts_ns":20,"vehicle_id":"iuas-01","peer":"wuas-01","bias_m":2.0}),
            json!({"kind":"service.flight_takeoff","ts_ns":15,"vehicle_id":"iuas-01","accepted":true}),
            json!({"kind":"rtl.done","ts_ns":30,"vehicle_id":"iuas-01","outcome":"landed"}),
        ];
        let payload = lines.iter().map(|l| l.to_string()).collect::<Vec<_>>().join("\n");
        ds.add_journal_block(bref(1, "/muas/v3/iuas-01/journal/companion", 0), payload.as_bytes());
        ds.finish();

        assert_eq!(ds.run.run_id.as_deref(), Some("r1"));
        assert!(!ds.run.synthesized);
        assert_eq!(ds.run.display("coord.grace_s").as_deref(), Some("2.5"));
        assert_eq!(ds.of_kind(DatumKind::Coord).count(), 1);
        assert_eq!(ds.of_kind(DatumKind::Service).count(), 1);
        assert_eq!(ds.of_kind(DatumKind::Rtl).count(), 1);
        // Time-sorted: service (15) before coord (20).
        let labels: Vec<&str> = ds.data.iter().map(|d| d.label.as_str()).collect();
        assert_eq!(labels, ["run.config", "agent.up", "service.flight_takeoff", "coord.coop", "rtl.done"]);
        // Every datum cites the one block.
        for d in &ds.data {
            assert_eq!(ds.citation_of(d).hash, hex(&[1u8; 32]));
        }
    }

    #[test]
    fn missing_run_config_synthesizes_and_marks_unknown() {
        let mut ds = MissionDataset::new();
        let line = json!({"kind":"agent.up","ts_ns":1,"vehicle_id":"iuas-01",
                          "fleet_ids":["iuas-01"],"carrier":"ndnsf","source":"sim"});
        ds.add_journal_block(bref(2, "/c", 0), line.to_string().as_bytes());
        ds.finish();
        assert!(ds.run.synthesized);
        assert_eq!(ds.run.display("carrier").as_deref(), Some("ndnsf"));
        assert_eq!(ds.run.display("fleet_size").as_deref(), Some("1"));
        assert_eq!(ds.run.display("coord.grace_s"), None, "unknown stays unknown");
        assert!(ds.run_label().starts_with("run-"), "content-derived label");
    }

    #[test]
    fn disagreeing_vehicle_settings_split_per_vehicle() {
        let mut ds = MissionDataset::new();
        for (vid, floor, byte) in [("iuas-01", 3.5, 3u8), ("wuas-01", 4.0, 4u8)] {
            let line = json!({"kind":"run.config","ts_ns":1,"vehicle_id":vid,"run_id":"r2",
                              "config":{"coord":{"floor_agl_m":floor,"grace_s":2.5}}});
            ds.add_journal_block(bref(byte, &format!("/{vid}"), 0), line.to_string().as_bytes());
        }
        ds.finish();
        assert_eq!(ds.run.display("coord.grace_s").as_deref(), Some("2.5"), "agreement collapses");
        assert_eq!(ds.run.display("coord.floor_agl_m@iuas-01").as_deref(), Some("3.5"));
        assert_eq!(ds.run.display("coord.floor_agl_m@wuas-01").as_deref(), Some("4.0"));
    }

    #[test]
    fn recording_lines_classify() {
        let mut ds = MissionDataset::new();
        let lines = [
            json!({"t_ns":5,"event":{"type":"telemetry","vehicle":"iuas-01",
                   "sample":{"lat_deg":35.0,"lon_deg":-90.0,"agl_m":6.0}}}),
            json!({"t_ns":6,"event":{"type":"video_stats","vehicle":"iuas-01","fps":12.0}}),
            json!({"t_ns":7,"event":{"type":"event","kind":"mission.start"}}),
            json!({"t_ns":8,"event":{"type":"telemetry_stale","vehicle":"wuas-01","silent_s":3.0}}),
        ];
        let payload = lines.iter().map(|l| l.to_string()).collect::<Vec<_>>().join("\n");
        ds.add_recording_block(bref(9, "/muas/v3/gcs/recording/dash", 0), payload.as_bytes());
        ds.finish();
        assert_eq!(ds.of_kind(DatumKind::Telemetry).count(), 1);
        assert_eq!(ds.of_kind(DatumKind::Link).count(), 2);
        assert_eq!(ds.vehicles(), ["iuas-01", "wuas-01"]);
        let cites = ds.citations_where(|d| d.kind == DatumKind::Telemetry);
        assert_eq!(cites.len(), 1);
        assert_eq!(cites[0].chain, "/muas/v3/gcs/recording/dash");
    }
}
