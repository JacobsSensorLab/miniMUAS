//! Measurement: a spec (`specs/<name>.toml`, PROTOCOL.md "Measurement specs") run as repeats ×
//! arms of samples. Every sample is settled (I3), bracketed by stopped streams (I6) and counter
//! snapshots (I7), and written in full under `runs/<id>/` (I8); a run that switched cells puts the
//! starting cell back (I9). The caller holds the fleet lock (I1) and has done the armed check (I2).

use std::collections::BTreeMap;
use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};

use crate::cells;
use crate::config::Config;
use crate::counters::{self, NodeCounters};
use crate::jobs::{JobCtx, Jobs};
use crate::remote;
use crate::state::{self, Recorder};
use crate::status::{self, FleetStatus};

/// The settle wait reports what is left this often (I3: it waits and says so, never measures
/// early).
const SETTLE_LOG_EVERY_S: u64 = 30;
/// flightcheck's own budget beyond the window: hello (10 s) + stop-before (≤20 s) + stop-after
/// (≤10 s) + connect/report.
const FLIGHTCHECK_SLACK_S: u64 = 60;
/// fabric-bench beyond its flow duration: route setup, 100 ndnping probes, per-flow fetches.
const FABRIC_BENCH_SLACK_S: u64 = 300;
/// A stream-stop pass: hello (10 s) + quiet wait (≤20 s) + a 1 s window.
const STOP_STREAMS_TIMEOUT: Duration = Duration::from_secs(60);
/// `muas-fabric set` + health gate per cell change (PROTOCOL.md I9 re-verifies health).
const HEALTH_GATE_TIMEOUT: Duration = Duration::from_secs(300);

/// Peer-face counters a summary aggregates per node. LP repair (resent = the RTO path, fast-retx
/// = ack-ordering repair, dup-rx = duplicates a retransmission produced, gave-up = unrecoverable)
/// and reassembly are what rounds 4–7 of ndn-rs/docs/nfd-divergence-findings.md read by hand.
const PEER_COUNTERS: [&str; 10] = [
    "reliability.resent",
    "reliability.fast-retx",
    "reliability.dup-rx",
    "reliability.gave-up",
    "reliability.acks-tx",
    "reliability.acks-rx",
    "reassembly.completed",
    "reassembly.timed-out",
    "in.data",
    "out.data",
];

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(deny_unknown_fields)]
pub struct Spec {
    pub name: String,
    /// The question the spec answers.
    pub description: String,
    /// I3 minimum quiet time before each sample; `fleet.default_settle_s` when the file omits it
    /// (filled in on load, so `fleet_specs` shows the value in force).
    #[serde(default)]
    pub settle_s: Option<u64>,
    /// Workload window.
    pub duration_s: u64,
    #[serde(default = "one")]
    pub repeats: u32,
    /// Cells to compare, interleaved per repeat. Empty: measure the cell the fleet is on.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub arms: Vec<Arm>,
    /// Snapshot face/CS counters on every node around the workload (I7).
    #[serde(default = "yes")]
    pub counters: bool,
    pub workload: Workload,
    /// Optional pass/fail thresholds over `summary.json`, keyed by a dotted path in which `*`
    /// matches every key of an object (e.g. `"workload.video.*.fps" = { min = 12 }`).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub criteria: BTreeMap<String, Criterion>,
}

fn one() -> u32 {
    1
}
fn yes() -> bool {
    true
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Arm {
    pub label: String,
    /// `"<nfd|ndn-fwd> <wifi|radio>"`, as `muas-fabric` names cells.
    pub cell: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
pub enum Workload {
    /// `miniMUAS/tools/flightcheck.py` through the GCS dashboard: the operator path.
    Flightcheck {
        /// Vehicles to stream video from, all at once.
        #[serde(default)]
        video: Vec<String>,
        #[serde(default = "fc_width")]
        width: u32,
        #[serde(default = "fc_fps")]
        fps: f64,
        #[serde(default = "fc_quality")]
        quality: u32,
        #[serde(default = "fc_transport")]
        transport: String,
        /// Vehicle to task an audio capture on, or empty.
        #[serde(default)]
        audio: String,
        /// Vehicles that must be healthy (flightcheck's own default when empty).
        #[serde(default)]
        expect: Vec<String>,
    },
    /// `miniMUAS/tools/fabric-bench`: ndn-iperf with no application in the path.
    FabricBench {
        /// Node that produces.
        server: String,
        /// Node that consumes.
        client: String,
        #[serde(default = "fb_flows")]
        flows: u32,
        #[serde(default = "fb_size")]
        size: u32,
        #[serde(default = "fb_window")]
        window: u32,
        #[serde(default = "fb_ping_count")]
        ping_count: u32,
    },
    /// Nothing: counters over the window only.
    Idle {},
}

// flightcheck.py's own defaults, so a spec that omits a parameter means what the script means.
fn fc_width() -> u32 {
    320
}
fn fc_fps() -> f64 {
    10.0
}
fn fc_quality() -> u32 {
    40
}
fn fc_transport() -> String {
    "stream".into()
}
// fabric-bench.sh's defaults.
fn fb_flows() -> u32 {
    3
}
fn fb_size() -> u32 {
    8192
}
fn fb_window() -> u32 {
    64
}
fn fb_ping_count() -> u32 {
    100
}

impl Workload {
    fn kind(&self) -> &'static str {
        match self {
            Workload::Flightcheck { .. } => "flightcheck",
            Workload::FabricBench { .. } => "fabric-bench",
            Workload::Idle {} => "idle",
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Criterion {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub equals: Option<Value>,
}

impl Spec {
    fn validate(&self, cfg: &Config) -> Result<()> {
        if self.repeats == 0 || self.duration_s == 0 {
            bail!("spec `{}`: repeats and duration_s must be >= 1", self.name);
        }
        let mut labels = std::collections::BTreeSet::new();
        for arm in &self.arms {
            if arm.label.is_empty() || !labels.insert(arm.label.as_str()) {
                bail!(
                    "spec `{}`: arm labels must be non-empty and unique",
                    self.name
                );
            }
            cells::parse_cell(&arm.cell)
                .with_context(|| format!("spec `{}`: arm `{}`", self.name, arm.label))?;
        }
        match &self.workload {
            Workload::Flightcheck {
                video,
                audio,
                expect,
                transport,
                ..
            } => {
                let vehicles: Vec<&str> = cfg
                    .nodes
                    .iter()
                    .filter(|n| !n.is_gcs())
                    .map(|n| n.vehicle.as_str())
                    .collect();
                let named = video
                    .iter()
                    .chain(expect)
                    .map(String::as_str)
                    .chain((!audio.is_empty()).then_some(audio.as_str()));
                for v in named {
                    if !vehicles.contains(&v) {
                        bail!(
                            "spec `{}`: `{v}` is not a fleet vehicle ({})",
                            self.name,
                            vehicles.join(", ")
                        );
                    }
                }
                if !matches!(transport.as_str(), "stream" | "segmented") {
                    bail!("spec `{}`: transport must be stream|segmented", self.name);
                }
            }
            Workload::FabricBench { server, client, .. } => {
                for n in [server, client] {
                    if cfg.node(n).is_none() {
                        bail!("spec `{}`: `{n}` is not a fleet node", self.name);
                    }
                }
                if server == client {
                    bail!(
                        "spec `{}`: fabric-bench server and client must differ",
                        self.name
                    );
                }
            }
            Workload::Idle {} => {}
        }
        for (path, c) in &self.criteria {
            if c.min.is_none() && c.max.is_none() && c.equals.is_none() {
                bail!(
                    "spec `{}`: criterion `{path}` needs min, max or equals",
                    self.name
                );
            }
        }
        Ok(())
    }
}

/// Every spec in `specs/` next to `fleet.toml`, sorted by name.
pub fn load_specs(cfg: &Config) -> Result<Vec<Spec>> {
    let dir = cfg.root.join("specs");
    let mut paths = Vec::new();
    for entry in std::fs::read_dir(&dir).with_context(|| format!("reading {}", dir.display()))? {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "toml") {
            paths.push(path);
        }
    }
    paths.sort();
    paths.iter().map(|p| load_spec(cfg, p)).collect()
}

/// The spec `specs/<name>.toml`.
pub fn find_spec(cfg: &Config, name: &str) -> Result<Spec> {
    if name.is_empty()
        || !name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
    {
        bail!("`{name}` is not a spec name");
    }
    let path = cfg.root.join("specs").join(format!("{name}.toml"));
    if !path.exists() {
        let known: Vec<String> = load_specs(cfg)
            .map(|s| s.into_iter().map(|s| s.name).collect())
            .unwrap_or_default();
        bail!("no spec `{name}`; known: {}", known.join(", "));
    }
    load_spec(cfg, &path)
}

fn load_spec(cfg: &Config, path: &Path) -> Result<Spec> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let mut spec: Spec =
        toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
    // The file name is how callers address a spec; a mismatch would make two names for one
    // protocol and break comparability by name.
    if path.file_stem().and_then(|s| s.to_str()) != Some(spec.name.as_str()) {
        bail!(
            "{}: name `{}` must match the file name",
            path.display(),
            spec.name
        );
    }
    spec.settle_s.get_or_insert(cfg.fleet.default_settle_s);
    spec.validate(cfg)?;
    Ok(spec)
}

/// Apply ad-hoc `overrides` to `spec` and return the effective spec plus the overrides as
/// applied, flat (`"workload.fps": 30`), for the manifest. Allowed: `duration_s`, `repeats`,
/// `settle_s`, and `workload.<param>` either dotted or as a nested `workload` object. The
/// workload kind, arms and criteria are the spec's identity: changing them is a different spec.
pub fn apply_overrides(spec: &Spec, overrides: &Value) -> Result<(Spec, Map<String, Value>)> {
    let mut applied = Map::new();
    match overrides {
        Value::Null => return Ok((spec.clone(), applied)),
        Value::Object(o) => {
            for (k, v) in o {
                match (k.as_str(), v) {
                    ("workload", Value::Object(w)) => {
                        for (wk, wv) in w {
                            applied.insert(format!("workload.{wk}"), wv.clone());
                        }
                    }
                    _ => {
                        applied.insert(k.clone(), v.clone());
                    }
                }
            }
        }
        other => bail!("overrides must be an object, got {other}"),
    }
    let mut value = serde_json::to_value(spec)?;
    for (k, v) in &applied {
        match k.as_str() {
            "duration_s" | "repeats" | "settle_s" => {
                value[k.as_str()] = v.clone();
            }
            _ => match k.strip_prefix("workload.") {
                Some(param) if param != "kind" && !param.is_empty() => {
                    value["workload"][param] = v.clone();
                }
                _ => bail!(
                    "override `{k}` is not allowed: only duration_s, repeats, settle_s and \
                     workload.<param> (not kind) — a different workload is a different spec"
                ),
            },
        }
    }
    let out: Spec = serde_json::from_value(value)
        .with_context(|| format!("overrides {overrides} do not fit spec `{}`", spec.name))?;
    if out.repeats == 0 || out.duration_s == 0 {
        bail!("overrides: repeats and duration_s must be >= 1");
    }
    Ok((out, applied))
}

/// Seconds still to wait before a sample may start: I3 requires
/// `now − last_disturbance ≥ settle_s`. No recorded disturbance means nothing to wait for.
pub fn settle_remaining(since_disturbance_s: Option<u64>, settle_s: u64) -> u64 {
    since_disturbance_s.map_or(0, |s| settle_s.saturating_sub(s))
}

/// Arm order for repeat `rep` (1-based). The first repeat starts with the arm already in force
/// (no switch before the first sample); even repeats run the order reversed (ABBA), so arms are
/// interleaved within every repeat, order effects cancel across pairs, and each repeat boundary
/// costs no cell switch — every switch costs a disturbance plus a full settle.
pub fn arm_order(arms: &[Arm], rep: u32, start_cell: &str) -> Vec<usize> {
    let n = arms.len();
    let first = arms.iter().position(|a| a.cell == start_cell).unwrap_or(0);
    let mut order: Vec<usize> = (0..n).map(|i| (first + i) % n).collect();
    if rep.is_multiple_of(2) {
        order.reverse();
    }
    order
}

/// One sample's outcome, as `run()` reports and aggregates it.
#[derive(Serialize, Clone, Debug)]
pub struct SampleResult {
    pub run_id: String,
    pub arm: String,
    pub repeat: u32,
    pub attempt: u32,
    pub valid: bool,
    pub verdict: String,
    pub summary: Value,
}

struct RunCtx<'a> {
    cfg: &'a Config,
    rec: &'a Recorder,
    job: &'a JobCtx,
    spec: &'a Spec,
    overrides: &'a Map<String, Value>,
    settle_s: u64,
}

/// Run `spec` (with `overrides`) as a job. The caller holds the fleet lock.
pub async fn run(
    cfg: &Config,
    rec: &Recorder,
    job: &JobCtx,
    spec: Spec,
    overrides: Value,
) -> Result<Value> {
    let (spec, applied) = apply_overrides(&spec, &overrides)?;
    spec.validate(cfg)?;
    let settle_s = spec.settle_s.unwrap_or(cfg.fleet.default_settle_s);
    let status = status::fleet_status(cfg, rec).await?;
    let start_cell = uniform_cell(&status)?;
    let arms = if spec.arms.is_empty() {
        vec![Arm {
            label: start_cell.clone(),
            cell: start_cell.clone(),
        }]
    } else {
        spec.arms.clone()
    };
    job.log(format!(
        "measure `{}`: {} repeat(s) x {} arm(s), {} s window, settle {} s, from cell `{start_cell}`{}",
        spec.name,
        spec.repeats,
        arms.len(),
        spec.duration_s,
        settle_s,
        if applied.is_empty() {
            String::new()
        } else {
            format!(", overrides {}", Value::Object(applied.clone()))
        }
    ));
    rec.ledger(
        "measure.start",
        json!({"job": job.id, "spec": spec.name, "overrides": applied, "start_cell": start_cell}),
    );
    let cx = RunCtx {
        cfg,
        rec,
        job,
        spec: &spec,
        overrides: &applied,
        settle_s,
    };

    let mut current = start_cell.clone();
    let mut samples: Vec<SampleResult> = Vec::new();
    let mut cell_changes = Vec::new();
    let outcome: Result<()> = async {
        for rep in 1..=spec.repeats {
            for i in arm_order(&arms, rep, &start_cell) {
                let arm = &arms[i];
                if arm.cell != current {
                    job.log(format!(
                        "arm `{}`: switching cell `{current}` -> `{}`",
                        arm.label, arm.cell
                    ));
                    let r = cells::set_cell_all(cfg, rec, job, &arm.cell).await?;
                    cell_changes.push(json!({"from": current, "to": arm.cell, "result": r}));
                    current = arm.cell.clone();
                }
                let first = sample(&cx, arm, rep, 1, None).await?;
                let retry = (!first.valid).then(|| first.run_id.clone());
                samples.push(first);
                if let Some(id) = retry {
                    // An INVALID sample (streams would not stop, backlog, no counters) is not a
                    // result; one more attempt, then it is recorded as invalid and skipped.
                    job.log(format!("run {id} INVALID: repeating once"));
                    samples.push(sample(&cx, arm, rep, 2, Some(id)).await?);
                }
            }
        }
        Ok(())
    }
    .await;

    // I9: whatever happened, put the starting cell back.
    let mut restore_err = None;
    let restored = if current != start_cell {
        job.log(format!("restoring starting cell `{start_cell}` (I9)"));
        match cells::set_cell_all(cfg, rec, job, &start_cell).await {
            Ok(r) => json!({"cell": start_cell, "result": r}),
            Err(e) => {
                let msg = format!("{e:#}");
                restore_err = Some(msg.clone());
                json!({"cell": start_cell, "error": msg})
            }
        }
    } else {
        Value::Null
    };

    let run_ids: Vec<&str> = samples.iter().map(|s| s.run_id.as_str()).collect();
    match (outcome, restore_err) {
        (Err(e), None) => {
            return Err(e.context(format!(
                "measure `{}` stopped; runs so far: {run_ids:?}",
                spec.name
            )));
        }
        (Err(e), Some(r)) => bail!(
            "measure `{}` stopped ({e:#}) AND restoring cell `{start_cell}` failed ({r}); runs so far: {run_ids:?}",
            spec.name
        ),
        (Ok(()), Some(r)) => bail!(
            "measure `{}` completed (runs {run_ids:?}) but restoring cell `{start_cell}` failed: {r}",
            spec.name
        ),
        (Ok(()), None) => {}
    }

    let result = json!({
        "spec": spec.name,
        "overrides": applied,
        "start_cell": start_cell,
        "runs": samples.iter().map(|s| json!({
            "run_id": s.run_id, "arm": s.arm, "repeat": s.repeat, "attempt": s.attempt,
            "valid": s.valid, "verdict": s.verdict, "criteria_pass": s.summary["criteria_pass"],
        })).collect::<Vec<_>>(),
        "arms": aggregate_arms(&samples),
        "cell_changes": cell_changes,
        "restored": restored,
    });
    rec.ledger(
        "measure.done",
        json!({"job": job.id, "spec": spec.name, "runs": run_ids}),
    );
    Ok(result)
}

/// Back to the known-good cell on every node (health-gated), then the `flightcheck` spec once.
/// The caller holds the fleet lock.
pub async fn restore(cfg: &Config, rec: &Recorder, job: &JobCtx) -> Result<Value> {
    let good = cfg.fleet.known_good_cell.clone();
    let status = status::fleet_status(cfg, rec).await?;
    let off: Vec<String> = status
        .nodes
        .iter()
        .filter(|n| n.active.as_deref() != Some(good.as_str()))
        .map(|n| format!("{}={}", n.name, n.active.as_deref().unwrap_or("?")))
        .collect();
    let set_cell = if off.is_empty() {
        job.log(format!("every node already on `{good}`; health gate only"));
        status::health_gate(cfg, rec, &good, HEALTH_GATE_TIMEOUT, &|l: &str| job.log(l)).await?;
        Value::Null
    } else {
        job.log(format!("restoring `{good}` (was {})", off.join(", ")));
        cells::set_cell_all(cfg, rec, job, &good).await?
    };
    let spec = find_spec(cfg, "flightcheck")?;
    let settle_s = spec.settle_s.unwrap_or(cfg.fleet.default_settle_s);
    let no_overrides = Map::new();
    let cx = RunCtx {
        cfg,
        rec,
        job,
        spec: &spec,
        overrides: &no_overrides,
        settle_s,
    };
    let arm = Arm {
        label: "restore".into(),
        cell: good.clone(),
    };
    let s = sample(&cx, &arm, 1, 1, None).await?;
    job.log(format!("restore flight check {}: {}", s.run_id, s.verdict));
    Ok(json!({
        "cell": good,
        "was": off,
        "set_cell": set_cell,
        "run_id": s.run_id,
        "verdict": s.verdict,
        "valid": s.valid,
        "summary": s.summary,
    }))
}

/// The one cell every node is on; measuring a fleet split across cells (or with a node down)
/// measures nothing reproducible.
fn uniform_cell(status: &FleetStatus) -> Result<String> {
    let cells: Vec<(&str, Option<&str>)> = status
        .nodes
        .iter()
        .map(|n| {
            (
                n.name.as_str(),
                n.reachable.then_some(n.active.as_deref()).flatten(),
            )
        })
        .collect();
    match cells.first() {
        Some((_, Some(c))) if cells.iter().all(|(_, x)| *x == Some(*c)) => Ok(c.to_string()),
        _ => bail!(
            "the fleet is not on one cell ({}); fleet_set_cell or fleet_restore first",
            cells
                .iter()
                .map(|(n, c)| format!("{n}={}", c.unwrap_or("unreachable/unknown")))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    }
}

/// Checks right before a sample: nobody armed (I2 holds for the whole job, not just its start)
/// and every node on the arm's cell.
fn preflight(status: &FleetStatus, cell: &str) -> Result<()> {
    if !status.armed.armed.is_empty() {
        bail!(
            "vehicle(s) armed: {} — not measuring under an armed vehicle (I2)",
            status.armed.armed.join(", ")
        );
    }
    let c = uniform_cell(status)?;
    if c != cell {
        bail!("fleet is on `{c}`, the arm needs `{cell}`");
    }
    Ok(())
}

async fn settle_gate(rec: &Recorder, job: &JobCtx, settle_s: u64) -> u64 {
    let started = Instant::now();
    loop {
        let remaining = settle_remaining(rec.seconds_since_disturbance(), settle_s);
        if remaining == 0 {
            return started.elapsed().as_secs();
        }
        let why = rec
            .state()
            .last_disturbance
            .map(|d| format!("{} {}", d.kind, d.detail))
            .unwrap_or_default();
        job.log(format!(
            "settle (I3): {remaining} s left of {settle_s} s since the last disturbance ({why})"
        ));
        tokio::time::sleep(Duration::from_secs(remaining.min(SETTLE_LOG_EVERY_S))).await;
    }
}

/// What one sample produced; filled step by step so a failure part-way still records what ran.
#[derive(Default)]
struct Measured {
    t0: Value,
    streams_before: Value,
    streams_after: Value,
    workload: Value,
    deltas: Option<Value>,
}

/// One sample of `cx.spec` on `arm`: settle, identity, stop streams, counters before, workload,
/// counters after, stop streams, write `runs/<id>/`. Errors that make the fleet unsafe or wrong
/// to measure (armed, wrong cell, unreachable) abort; errors inside the sample make it INVALID.
async fn sample(
    cx: &RunCtx<'_>,
    arm: &Arm,
    repeat: u32,
    attempt: u32,
    retry_of: Option<String>,
) -> Result<SampleResult> {
    let settle_waited_s = settle_gate(cx.rec, cx.job, cx.settle_s).await;
    let fleet = status::fleet_status(cx.cfg, cx.rec).await?;
    preflight(&fleet, &arm.cell)?;
    let run_id = cx
        .rec
        .new_id(&format!("{}-{}-r{repeat}", cx.spec.name, slug(&arm.label)));
    let rel = format!("runs/{run_id}");
    let last_deploy = cx.rec.state().last_deploy;
    let mut manifest = json!({
        "run_id": run_id,
        "job_id": cx.job.id,
        "spec": cx.spec,
        "overrides": cx.overrides,
        "arm": arm,
        "repeat": repeat,
        "repeats": cx.spec.repeats,
        "attempt": attempt,
        "retry_of": retry_of,
        "settle": {"settle_s": cx.settle_s, "waited_s": settle_waited_s,
                   "since_disturbance_s": fleet.since_disturbance_s},
        "started_unix_ms": state::now_ms(),
        "identity": identity(&fleet),
        "deploy": deploy_in_force(cx.rec, last_deploy.as_deref()),
        "fleet_status": fleet,
    });
    cx.rec
        .write_json(&format!("{rel}/manifest.json"), &manifest)?;
    cx.job.log(format!(
        "run {run_id}: arm `{}` repeat {repeat}/{} attempt {attempt}",
        arm.label, cx.spec.repeats
    ));

    let mut m = Measured::default();
    let error = measure_into(cx, &rel, &run_id, &mut m)
        .await
        .err()
        .map(|e| format!("{e:#}"));
    if let Some(e) = &error {
        cx.job.log(format!("run {run_id}: {e}"));
        m.workload = json!({"verdict": "INVALID", "error": e, "partial": m.workload});
        cx.rec
            .write_json(&format!("{rel}/workload.json"), &m.workload)?;
    }
    let mut summary = summarize(cx.spec, &m.workload, m.deltas.as_ref());
    summary["run_id"] = json!(run_id);
    summary["arm"] = json!(arm.label);
    summary["repeat"] = json!(repeat);
    summary["attempt"] = json!(attempt);
    if let Some(e) = &error {
        summary["error"] = json!(e);
    }
    cx.rec
        .write_json(&format!("{rel}/summary.json"), &summary)?;
    manifest["t0"] = m.t0;
    manifest["streams"] = json!({"before": m.streams_before, "after": m.streams_after});
    manifest["finished_unix_ms"] = json!(state::now_ms());
    manifest["error"] = json!(error);
    cx.rec
        .write_json(&format!("{rel}/manifest.json"), &manifest)?;

    let verdict = summary["verdict"].as_str().unwrap_or("INVALID").to_string();
    let valid = summary["valid"].as_bool().unwrap_or(false);
    cx.job.log(format!(
        "run {run_id}: {verdict}{}",
        match summary["criteria_pass"].as_bool() {
            Some(true) => ", criteria pass",
            Some(false) => ", criteria FAIL",
            None => "",
        }
    ));
    cx.rec.ledger(
        "measure.run",
        json!({"run": run_id, "spec": cx.spec.name, "arm": arm.label, "repeat": repeat,
               "attempt": attempt, "verdict": verdict, "valid": valid}),
    );
    Ok(SampleResult {
        run_id,
        arm: arm.label.clone(),
        repeat,
        attempt,
        valid,
        verdict,
        summary,
    })
}

async fn measure_into(cx: &RunCtx<'_>, rel: &str, run_id: &str, m: &mut Measured) -> Result<()> {
    m.t0 = gcs_clock(cx.cfg, cx.rec, rel).await;
    m.streams_before = stop_streams(cx.cfg, cx.rec, cx.job, rel, "stop-before").await?;
    if m.streams_before["quiet"] != json!(true) {
        m.workload = json!({"verdict": "INVALID",
                            "error": "video would not stop before the sample (I6)"});
        cx.rec
            .write_json(&format!("{rel}/workload.json"), &m.workload)?;
        return Ok(());
    }
    let before = if cx.spec.counters {
        Some(snapshot(cx, rel, "before").await?)
    } else {
        None
    };
    m.workload = run_workload(cx, rel, run_id).await?;
    let after = if cx.spec.counters {
        Some(snapshot(cx, rel, "after").await?)
    } else {
        None
    };
    // flightcheck --stop-after already stopped (and confirmed) its streams; anything else, or a
    // flightcheck that did not get that far, gets its own stop pass (I6).
    m.streams_after = match &m.workload["stop_after"] {
        s if s["quiet"] == json!(true) => s.clone(),
        _ => stop_streams(cx.cfg, cx.rec, cx.job, rel, "stop-after").await?,
    };
    if let (Some(b), Some(a)) = (before, after) {
        let d = counters::delta(&b, &a);
        cx.rec.write_json(&format!("{rel}/deltas.json"), &d)?;
        m.deltas = Some(d);
    }
    Ok(())
}

/// Per-node identity for the manifest (I8): what code ran where, on which cell.
fn identity(status: &FleetStatus) -> Value {
    let nodes: Map<String, Value> = status
        .nodes
        .iter()
        .map(|n| {
            (
                n.name.clone(),
                json!({"system": n.system, "ndn_fwd_pkg": n.ndn_fwd_pkg, "active": n.active}),
            )
        })
        .collect();
    Value::Object(nodes)
}

/// The deploy in force and its pins (every pinned repo's rev, plus the miniMUAS rev).
fn deploy_in_force(rec: &Recorder, id: Option<&str>) -> Value {
    let Some(id) = id else {
        return Value::Null;
    };
    match rec.read_json::<Value>(&format!("deploys/{id}.json")) {
        Ok(d) => json!({
            "id": id,
            "outcome": d["outcome"],
            "pins": d["plan"]["pins"],
            "pins_commit": d["pins_commit"],
        }),
        Err(e) => json!({"id": id, "error": format!("{e:#}")}),
    }
}

/// The sample's t0 on the GCS clock (node t0, PROTOCOL.md results layout): logs on the nodes
/// are read against it, not against this machine's clock.
async fn gcs_clock(cfg: &Config, rec: &Recorder, rel: &str) -> Value {
    let local_ms = state::now_ms();
    let out = remote::ssh(cfg, cfg.gcs(), "date +%s%3N", Duration::from_secs(20)).await;
    let (gcs_ms, err) = match &out {
        Ok(o) => match o.stdout_ok().map(|s| s.trim().parse::<u64>()) {
            Ok(Ok(ms)) => (Some(ms), None),
            Ok(Err(e)) => (
                None,
                Some(format!("unparseable `{}`: {e}", o.stdout.trim())),
            ),
            Err(e) => (None, Some(format!("{e:#}"))),
        },
        Err(e) => (None, Some(format!("{e:#}"))),
    };
    if let Ok(o) = &out {
        write_raw(rec, rel, "t0", o);
    }
    json!({
        "gcs_unix_ms": gcs_ms,
        "gcs_iso": gcs_ms.map(state::iso8601),
        "local_unix_ms": local_ms,
        "error": err,
    })
}

async fn snapshot(cx: &RunCtx<'_>, rel: &str, tag: &str) -> Result<Vec<NodeCounters>> {
    let snap = counters::snapshot_fleet(cx.cfg)
        .await
        .with_context(|| format!("counters {tag}"))?;
    for n in &snap {
        cx.rec
            .write_text(&format!("{rel}/raw/counters-{tag}-{}.txt", n.node), &n.raw)?;
    }
    cx.rec
        .write_json(&format!("{rel}/counters-{tag}.json"), &snap)?;
    Ok(snap)
}

/// `raw/<name>.txt`: the command, its exit, stdout and stderr, verbatim (I8).
fn write_raw(rec: &Recorder, rel: &str, name: &str, o: &remote::Output) {
    let status = o
        .status
        .map_or_else(|| "timed out".to_string(), |s| format!("exit {s}"));
    let text = format!(
        "$ {}\n# {status}, {} ms\n--- stdout ---\n{}\n--- stderr ---\n{}\n",
        o.command, o.elapsed_ms, o.stdout, o.stderr
    );
    if let Err(e) = rec.write_text(&format!("{rel}/raw/{name}.txt"), &text) {
        eprintln!("ndn-fleet: writing raw/{name}.txt: {e:#}");
    }
}

/// Repair what jobs abandoned by a dead process left behind, before a new mutation starts.
///
/// Runs that will never get a summary (owning job no longer live) are marked incomplete, so they
/// are never read as results. For abandoned jobs, the fleet's state is unknown, so the settle clock
/// restarts (I3); a measurement or restore that died mid-sample may have left video enabled (its
/// stop-after never ran), so streams are stopped (I6). A job is recorded abandoned only after
/// this repair; if the repair is itself cut short, the next mutation redoes it. Found in the
/// field: a CLI measurement killed by its caller's timeout during repeat 3 of `lp-reliability`,
/// then four recoveries killed between marking and repairing, which lost the repair.
pub async fn recover_abandoned(
    cfg: &Config,
    rec: &Recorder,
    job: &JobCtx,
    jobs: &Jobs,
) -> Result<Value> {
    let incomplete = mark_incomplete_runs(rec, job, jobs)?;
    let abandoned = jobs.abandoned();
    if abandoned.is_empty() {
        return Ok(json!({"abandoned": [], "incomplete_runs": incomplete}));
    }
    let ids: Vec<&str> = abandoned.iter().map(|j| j.id.as_str()).collect();
    job.log(format!("recovering abandoned job(s) {}", ids.join(", ")));
    rec.disturb("abandoned", &format!("job(s) {}", ids.join(", ")));

    let sampling = abandoned
        .iter()
        .any(|j| j.kind == "measure" || j.kind == "restore");
    let streams = if sampling {
        let rel = format!("jobs/{}-recovery", job.id);
        stop_streams(cfg, rec, job, &rel, "stop-abandoned").await?
    } else {
        Value::Null
    };
    // Streams not confirmed off: leave the jobs unrecorded so the next mutation retries. A
    // measurement's own stop-before still guards its samples meanwhile.
    if sampling && streams["quiet"] != json!(true) {
        return Ok(
            json!({"abandoned": ids, "repaired": false, "streams": streams,
                         "incomplete_runs": incomplete}),
        );
    }
    for j in &abandoned {
        jobs.mark_abandoned(j)?;
        rec.ledger(
            "job_abandoned",
            json!({"job": j.id, "kind": j.kind, "recovered_by": job.id}),
        );
    }
    Ok(
        json!({"abandoned": ids, "repaired": true, "streams": streams,
              "incomplete_runs": incomplete}),
    )
}

/// Mark every run without a summary whose owning job is no longer live as incomplete.
fn mark_incomplete_runs(rec: &Recorder, job: &JobCtx, jobs: &Jobs) -> Result<Vec<String>> {
    let Ok(dir) = std::fs::read_dir(rec.dir().join("runs")) else {
        return Ok(Vec::new());
    };
    let mut marked = Vec::new();
    for entry in dir.flatten() {
        let run = entry.file_name().to_string_lossy().into_owned();
        let base = format!("runs/{run}");
        let path = rec.dir().join(&base);
        if path.join("summary.json").exists() || path.join("incomplete.json").exists() {
            continue;
        }
        let Ok(manifest) = rec.read_json::<Value>(&format!("{base}/manifest.json")) else {
            continue;
        };
        let Some(owner) = manifest["job_id"].as_str() else {
            continue;
        };
        if jobs.is_live(owner) {
            continue;
        }
        rec.write_json(
            &format!("{base}/incomplete.json"),
            &json!({
                "job": owner,
                "detected_by": job.id,
                "reason": "its job ended before this sample finished: no summary, never compare it",
            }),
        )?;
        job.log(format!("run {run} marked incomplete (job {owner} ended)"));
        marked.push(run);
    }
    marked.sort();
    Ok(marked)
}

fn path_str(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

/// Disable video on every vehicle and wait until the dashboard is quiet (I6), via
/// `flightcheck --stop-before` over a 1 s window. Returns flightcheck's stop record
/// (`quiet: true` on success).
pub async fn stop_streams(
    cfg: &Config,
    rec: &Recorder,
    job: &JobCtx,
    rel: &str,
    tag: &str,
) -> Result<Value> {
    let json_path = rec.dir().join(rel).join(format!("raw/{tag}.json"));
    // flightcheck writes --json without creating directories; a recovery dir is fresh.
    if let Some(parent) = json_path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating {}", parent.display()))?;
    }
    let fc = path_str(&cfg.workloads.flightcheck);
    let port = cfg.dashboard.port.to_string();
    let jp = path_str(&json_path);
    let args = [
        fc.as_str(),
        "--host",
        cfg.dashboard.host.as_str(),
        "--port",
        port.as_str(),
        "--seconds",
        "1",
        "--stop-before",
        "--label",
        tag,
        "--json",
        jp.as_str(),
    ];
    let out = remote::local(
        &cfg.workloads.python,
        &args,
        None,
        &[],
        STOP_STREAMS_TIMEOUT,
    )
    .await?;
    write_raw(rec, rel, tag, &out);
    let rec = read_json_file(&json_path)
        .map(|v| v["stop_before"].clone())
        .unwrap_or_else(|e| json!({"quiet": false, "error": format!("{e:#}")}));
    if rec["quiet"] != json!(true) {
        job.log(format!("{tag}: dashboard NOT quiet: {}", rec["error"]));
    }
    Ok(rec)
}

fn read_json_file(path: &Path) -> Result<Value> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))
}

/// Run the spec's workload over its window. Returns what goes in `workload.json`, which always
/// carries a `verdict` (`INVALID` when the workload could not produce a trustworthy sample).
async fn run_workload(cx: &RunCtx<'_>, rel: &str, run_id: &str) -> Result<Value> {
    let dir = cx.rec.dir().join(rel);
    let duration = cx.spec.duration_s;
    let v = match &cx.spec.workload {
        Workload::Flightcheck {
            video,
            width,
            fps,
            quality,
            transport,
            audio,
            expect,
        } => {
            let json_path = dir.join("workload.json");
            let mut args: Vec<String> = vec![
                path_str(&cx.cfg.workloads.flightcheck),
                "--host".into(),
                cx.cfg.dashboard.host.clone(),
                "--port".into(),
                cx.cfg.dashboard.port.to_string(),
                "--seconds".into(),
                duration.to_string(),
                "--label".into(),
                run_id.into(),
                "--json".into(),
                path_str(&json_path),
                // I6, inside the workload too: its own stop brackets the exact window it
                // measures, and it rejects a stream whose first frame precedes its enable.
                "--stop-before".into(),
                "--stop-after".into(),
                // Report any stream that ran in the window, not just the requested ones: a
                // stream nobody asked for is load on the sample.
                "--video-all".into(),
            ];
            if !video.is_empty() {
                args.extend([
                    "--video".into(),
                    video.join(","),
                    "--width".into(),
                    width.to_string(),
                    "--fps".into(),
                    fps.to_string(),
                    "--quality".into(),
                    quality.to_string(),
                    "--transport".into(),
                    transport.clone(),
                ]);
            }
            if !audio.is_empty() {
                args.extend(["--audio".into(), audio.clone()]);
            }
            if !expect.is_empty() {
                args.extend(["--expect".into(), expect.join(",")]);
            }
            let argv: Vec<&str> = args.iter().map(String::as_str).collect();
            cx.job.log(format!("flightcheck: {duration} s window"));
            let out = remote::local(
                &cx.cfg.workloads.python,
                &argv,
                None,
                &[],
                Duration::from_secs(duration + FLIGHTCHECK_SLACK_S),
            )
            .await?;
            write_raw(cx.rec, rel, "flightcheck", &out);
            match read_json_file(&json_path) {
                Ok(v) => v,
                Err(e) => {
                    let v = json!({"verdict": "INVALID", "exit": out.status,
                                   "error": format!("flightcheck wrote no result: {e:#}")});
                    cx.rec.write_json(&format!("{rel}/workload.json"), &v)?;
                    v
                }
            }
        }
        Workload::FabricBench {
            server,
            client,
            flows,
            size,
            window,
            ping_count,
        } => {
            let target = |name: &str| -> Result<String> {
                let n = cx
                    .cfg
                    .node(name)
                    .ok_or_else(|| anyhow!("fabric-bench: no node `{name}`"))?;
                Ok(format!("{}@{}", cx.cfg.fleet.ssh_user, n.host))
            };
            let out_dir = dir.join("raw/fabric-bench");
            std::fs::create_dir_all(&out_dir)
                .with_context(|| format!("creating {}", out_dir.display()))?;
            let args: Vec<String> = vec![
                path_str(&cx.cfg.workloads.fabric_bench),
                // ndn-fleet owns cell changes and the settle clock (I3); fabric-bench only
                // measures the cell it finds.
                "--no-switch".into(),
                "--duration".into(),
                duration.to_string(),
                "--flows".into(),
                flows.to_string(),
                "--size".into(),
                size.to_string(),
                "--window".into(),
                window.to_string(),
                "--ping-count".into(),
                ping_count.to_string(),
                "--server".into(),
                target(server)?,
                "--client".into(),
                target(client)?,
            ];
            let argv: Vec<&str> = args.iter().map(String::as_str).collect();
            let out_env = path_str(&out_dir);
            cx.job.log(format!(
                "fabric-bench: {flows} flow(s), {duration} s, {server} -> {client}"
            ));
            let out = remote::local(
                "bash",
                &argv,
                None,
                &[("BENCH_OUT", out_env.as_str())],
                Duration::from_secs(duration + FABRIC_BENCH_SLACK_S),
            )
            .await?;
            write_raw(cx.rec, rel, "fabric-bench", &out);
            let v = fabric_bench_result(&out_dir, &out);
            cx.rec.write_json(&format!("{rel}/workload.json"), &v)?;
            v
        }
        Workload::Idle {} => {
            cx.job.log(format!("idle: {duration} s"));
            tokio::time::sleep(Duration::from_secs(duration)).await;
            let v = json!({"verdict": "COMPLETE", "slept_s": duration});
            cx.rec.write_json(&format!("{rel}/workload.json"), &v)?;
            v
        }
    };
    Ok(v)
}

/// fabric-bench writes `$BENCH_OUT/<UTC stamp>/` with `summary.jsonl` (one record per stack;
/// `current` under --no-switch), `current-ping.txt` and raw per-flow client output. No summary
/// record means the stack was skipped (no route: every number would have been a Nack) — INVALID,
/// not a result.
fn fabric_bench_result(out_dir: &Path, out: &remote::Output) -> Value {
    let run_dir = std::fs::read_dir(out_dir).ok().and_then(|rd| {
        rd.filter_map(|e| e.ok().map(|e| e.path()))
            .find(|p| p.is_dir())
    });
    let Some(run_dir) = run_dir else {
        return json!({"verdict": "INVALID", "exit": out.status,
                      "error": "fabric-bench produced no run directory"});
    };
    let read = |name: &str| std::fs::read_to_string(run_dir.join(name)).ok();
    // The records are pretty-printed over several lines: read a JSON stream, not lines.
    let summary = read("summary.jsonl").and_then(|t| {
        serde_json::Deserializer::from_str(&t)
            .into_iter::<Value>()
            .map_while(Result::ok)
            .last()
    });
    let flows: Vec<Value> = (1..)
        .map(|f| run_dir.join(format!("current-flow{f}-client.txt")))
        .take_while(|p| p.exists())
        .map(|p| std::fs::read_to_string(p).map_or(Value::Null, |t| parse_flow(&t)))
        .collect();
    let ping = read("current-ping.txt").map_or(Value::Null, |t| parse_ping(&t));
    let complete = summary.is_some() && out.ok();
    let mut v = json!({
        "verdict": if complete { "COMPLETE" } else { "INVALID" },
        "exit": out.status,
        "run_dir": path_str(&run_dir),
        "summary": summary,
        "flows": flows,
        "ping": ping,
    });
    if !complete {
        v["error"] = json!("fabric-bench did not complete (see raw/fabric-bench.txt)");
    }
    v
}

/// The number at the start of `s` (after blanks), e.g. `12.61` of ` 12.61 Mbps`.
fn num_prefix(s: &str) -> Option<f64> {
    let s = s.trim_start();
    let end = s
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(s.len());
    s[..end].parse().ok()
}

fn num_after(text: &str, key: &str) -> Option<f64> {
    text.find(key)
        .and_then(|i| num_prefix(&text[i + key.len()..]))
}

/// One ndn-iperf client report: `throughput:  12.61 Mbps`, `11 lost (0.3% loss)`,
/// `retransmits: 98`, `p50=42055us p95=683120us p99=960472us`.
fn parse_flow(text: &str) -> Value {
    let ms = |key: &str| num_after(text, key).map(|us| us / 1000.0);
    json!({
        "mbps": num_after(text, "throughput:"),
        "loss_pct": num_after(text, "lost ("),
        "retransmits": num_after(text, "retransmits:"),
        "rtt_p50_ms": ms("p50="),
        "rtt_p95_ms": ms("p95="),
        "rtt_p99_ms": ms("p99="),
    })
}

/// ndnping's statistics: `100 packets transmitted, 100 received, 0 nacked, 0% lost, 0% nacked,
/// …` and `rtt min/avg/max/mdev = 3.56/4.27/21.52/1.82 ms`. Run unloaded, before the flows, so
/// it separates path latency from the queueing the flows' RTTs include.
fn parse_ping(text: &str) -> Value {
    let stats = text
        .lines()
        .find(|l| l.contains("packets transmitted"))
        .unwrap_or("");
    let pct = |what: &str| {
        stats
            .split(", ")
            .find_map(|p| p.trim().strip_suffix(what))
            .and_then(num_prefix)
    };
    let rtt: Vec<f64> = text
        .lines()
        .find_map(|l| l.trim().strip_prefix("rtt min/avg/max/mdev = "))
        .map(|r| r.split('/').filter_map(num_prefix).collect())
        .unwrap_or_default();
    json!({
        "transmitted": num_prefix(stats),
        "lost_pct": pct("% lost"),
        "nacked_pct": pct("% nacked"),
        "rtt_min_ms": rtt.first(),
        "rtt_avg_ms": rtt.get(1),
        "rtt_max_ms": rtt.get(2),
    })
}

/// `summary.json`: the metrics a comparison reads. Workload key metrics per vehicle, per-node
/// aggregates over peer UDP faces, derived ratios, and the spec's criteria.
pub fn summarize(spec: &Spec, workload: &Value, deltas: Option<&Value>) -> Value {
    let verdict = workload["verdict"]
        .as_str()
        .unwrap_or("INVALID")
        .to_string();
    let valid = verdict != "INVALID";
    let key = match &spec.workload {
        Workload::Flightcheck { .. } => flightcheck_metrics(workload),
        Workload::FabricBench { .. } => fabric_bench_metrics(workload),
        Workload::Idle {} => json!({}),
    };
    let mut summary = json!({
        "spec": spec.name,
        "kind": spec.workload.kind(),
        "verdict": verdict,
        "valid": valid,
        "workload": key,
        "nodes": deltas.map_or(Value::Null, node_aggregates),
    });
    if let Some(missing) = deltas.and_then(|d| d.get("missing")) {
        summary["counters_missing"] = missing.clone();
    }
    // Why a sample was rejected must be readable from the summary, not only from workload.json.
    if let Some(e) = workload.get("error") {
        summary["workload_error"] = e.clone();
    }
    if !spec.criteria.is_empty() {
        let (results, pass) = evaluate_criteria(&spec.criteria, &summary);
        summary["criteria"] = results;
        summary["criteria_pass"] = json!(pass);
    } else {
        summary["criteria_pass"] = Value::Null;
    }
    summary
}

fn pick(v: &Value, fields: &[&str]) -> Value {
    Value::Object(
        fields
            .iter()
            .filter_map(|f| v.get(*f).map(|x| (f.to_string(), x.clone())))
            .collect(),
    )
}

fn per_key(v: &Value, fields: &[&str]) -> Value {
    match v.as_object() {
        Some(o) => Value::Object(
            o.iter()
                .map(|(k, x)| (k.clone(), pick(x, fields)))
                .collect(),
        ),
        None => json!({}),
    }
}

fn flightcheck_metrics(w: &Value) -> Value {
    json!({
        "telemetry": per_key(&w["telemetry"], &["n", "rate", "gaps_p95", "gaps_max", "gaps_over_2s", "status"]),
        "video": per_key(&w["video"], &["n", "fps", "kbps", "first_frame_s", "gap_p50", "gap_p95",
                                         "gap_max", "stutters_over_1s", "warmup_stutters", "valid", "status"]),
        "aggregate": w["aggregate"],
        "audio": pick(&w["audio"], &["vehicle", "status", "seconds", "ok"]),
        "invalid_reasons": w["invalid_reasons"],
    })
}

fn fabric_bench_metrics(w: &Value) -> Value {
    let s = &w["summary"];
    let flows = w["flows"].as_array().map(Vec::as_slice).unwrap_or_default();
    let col = |k: &str| -> Vec<f64> { flows.iter().filter_map(|f| f[k].as_f64()).collect() };
    let max = |xs: &[f64]| xs.iter().copied().reduce(f64::max);
    let min = |xs: &[f64]| xs.iter().copied().reduce(f64::min);
    let mbps = col("mbps");
    let retx = col("retransmits");
    json!({
        "aggregate_mbps": s["aggregate_mbps"],
        "failed_flows": s["failed_flows"],
        "flows": s["flows"],
        "flow_min_mbps": min(&mbps),
        "flow_max_mbps": max(&mbps),
        // The fairness signal: a forwarder that does not share capacity shows a wide spread.
        "flow_spread_mbps": max(&mbps).zip(min(&mbps)).map(|(a, b)| a - b),
        "loaded_rtt_p95_ms_worst": max(&col("rtt_p95_ms")),
        "retransmits": (!retx.is_empty()).then(|| retx.iter().sum::<f64>()),
        "unloaded_rtt_avg_ms": w["ping"]["rtt_avg_ms"],
        "ping_lost_pct": w["ping"]["lost_pct"],
        "ping_nacked_pct": w["ping"]["nacked_pct"],
        "client_in_data_delta": s["client_in_data_delta"],
        "client_out_data_delta": s["client_out_data_delta"],
    })
}

/// A peer face: unicast UDP to another node. Multicast discovery faces and internal/app faces
/// are not links a Data packet crosses between nodes.
fn is_peer_udp(key: &str) -> bool {
    let Some(rest) = key
        .strip_prefix("udp4://")
        .or_else(|| key.strip_prefix("udp6://"))
        .or_else(|| key.strip_prefix("udp://"))
    else {
        return false;
    };
    let host = rest.trim_start_matches('[');
    let first_octet = host
        .split(['.', ':'])
        .next()
        .and_then(|o| o.parse::<u16>().ok());
    let multicast_v4 = first_octet.is_some_and(|o| (224..=239).contains(&o));
    let multicast_v6 = host.to_ascii_lowercase().starts_with("ff");
    !(multicast_v4 || multicast_v6)
}

fn ratio(num: Option<f64>, den: Option<f64>) -> Value {
    match (num, den) {
        (Some(n), Some(d)) if d > 0.0 => json!(n / d),
        _ => Value::Null,
    }
}

/// Per node: peer-UDP-face sums of `PEER_COUNTERS` (null when no face reports a counter — NFD
/// does not expose LP reliability), CS hits/misses/hit rate, and the derived ratios.
fn node_aggregates(deltas: &Value) -> Value {
    let Some(nodes) = deltas["nodes"].as_object() else {
        return json!({});
    };
    let mut out = Map::new();
    for (name, n) in nodes {
        let mut sums: BTreeMap<&str, f64> = BTreeMap::new();
        let (mut peer, mut reset, mut new) = (0usize, Vec::new(), Vec::new());
        for (key, face) in n["faces"].as_object().into_iter().flatten() {
            if !is_peer_udp(key) {
                continue;
            }
            peer += 1;
            if face["reset"] == json!(true) {
                reset.push(key.clone());
            }
            if face["new"] == json!(true) {
                new.push(key.clone());
            }
            for c in PEER_COUNTERS {
                if let Some(x) = face["counters"][c].as_f64() {
                    *sums.entry(c).or_insert(0.0) += x;
                }
            }
        }
        let get = |c: &str| sums.get(c).copied();
        let mut groups: BTreeMap<&str, Map<String, Value>> = BTreeMap::new();
        for c in PEER_COUNTERS {
            let (group, field) = c.split_once('.').expect("dotted counter name");
            groups
                .entry(group)
                .or_default()
                .insert(field.to_string(), get(c).map_or(Value::Null, |x| json!(x)));
        }
        let mut node = Map::new();
        for (g, fields) in groups {
            node.insert(g.to_string(), Value::Object(fields));
        }
        node.insert(
            "cs".into(),
            json!({"hits": n["cs"]["hits"], "misses": n["cs"]["misses"], "hit_rate": n["cs_hit_rate"]}),
        );
        node.insert(
            "derived".into(),
            json!({
                // Duplicates a retransmission produced per ack-ordering repair.
                "dup_rx_per_fast_retx": ratio(get("reliability.dup-rx"), get("reliability.fast-retx")),
                // Unrecoverable LP losses per Data sent: each is a Data that never arrives.
                "gave_up_per_out_data": ratio(get("reliability.gave-up"), get("out.data")),
            }),
        );
        node.insert("forwarder".into(), n["forwarder"].clone());
        node.insert("forwarder_changed".into(), n["forwarder_changed"].clone());
        node.insert("window_ms".into(), n["window_ms"].clone());
        node.insert(
            "peer_faces".into(),
            json!({"count": peer, "reset": reset, "new": new}),
        );
        out.insert(name.clone(), Value::Object(node));
    }
    Value::Object(out)
}

/// Every value at `path` (dotted; `*` = every key of an object), with its concrete path.
fn select<'a>(v: &'a Value, path: &[&str], at: String, out: &mut Vec<(String, &'a Value)>) {
    let Some((head, rest)) = path.split_first() else {
        out.push((at, v));
        return;
    };
    let join = |k: &str| {
        if at.is_empty() {
            k.to_string()
        } else {
            format!("{at}.{k}")
        }
    };
    if *head == "*" {
        for (k, x) in v.as_object().into_iter().flatten() {
            select(x, rest, join(k), out);
        }
    } else if let Some(x) = v.get(*head) {
        select(x, rest, join(head), out);
    }
}

fn evaluate_criteria(criteria: &BTreeMap<String, Criterion>, summary: &Value) -> (Value, bool) {
    let mut results = Map::new();
    let mut all = true;
    for (path, c) in criteria {
        let parts: Vec<&str> = path.split('.').collect();
        let mut hits = Vec::new();
        select(summary, &parts, String::new(), &mut hits);
        let mut failed = Vec::new();
        let mut values = Map::new();
        for (at, v) in &hits {
            let ok = c.min.is_none_or(|m| v.as_f64().is_some_and(|x| x >= m))
                && c.max.is_none_or(|m| v.as_f64().is_some_and(|x| x <= m))
                && c.equals.as_ref().is_none_or(|e| values_equal(e, v));
            if !ok {
                failed.push(at.clone());
            }
            values.insert(at.clone(), (*v).clone());
        }
        // A criterion that matches nothing did not pass: the metric it guards was not measured.
        let pass = !hits.is_empty() && failed.is_empty();
        all &= pass;
        results.insert(
            path.clone(),
            json!({"rule": c, "pass": pass, "values": values, "failed": failed}),
        );
    }
    (Value::Object(results), all)
}

fn values_equal(a: &Value, b: &Value) -> bool {
    match (a.as_f64(), b.as_f64()) {
        (Some(x), Some(y)) => x == y,
        _ => a == b,
    }
}

fn flatten_numbers(prefix: &str, v: &Value, out: &mut BTreeMap<String, f64>) {
    match v {
        Value::Number(n) => {
            if let Some(x) = n.as_f64() {
                out.insert(prefix.to_string(), x);
            }
        }
        Value::Object(o) => {
            for (k, x) in o {
                flatten_numbers(&format!("{prefix}.{k}"), x, out);
            }
        }
        _ => {}
    }
}

/// Per arm, across its valid samples: every numeric metric of the summaries' `workload` and
/// `nodes` as mean/min/max. Invalid samples are listed, never averaged in.
pub fn aggregate_arms(samples: &[SampleResult]) -> Value {
    let mut arms: BTreeMap<&str, Vec<&SampleResult>> = BTreeMap::new();
    for s in samples {
        arms.entry(s.arm.as_str()).or_default().push(s);
    }
    let mut out = Map::new();
    for (arm, runs) in arms {
        let valid: Vec<&SampleResult> = runs.iter().copied().filter(|s| s.valid).collect();
        let mut series: BTreeMap<String, Vec<f64>> = BTreeMap::new();
        for s in &valid {
            let mut flat = BTreeMap::new();
            flatten_numbers("workload", &s.summary["workload"], &mut flat);
            flatten_numbers("nodes", &s.summary["nodes"], &mut flat);
            for (k, x) in flat {
                series.entry(k).or_default().push(x);
            }
        }
        let metrics: Map<String, Value> = series
            .into_iter()
            .map(|(k, xs)| {
                let n = xs.len() as f64;
                let mean = xs.iter().sum::<f64>() / n;
                let min = xs.iter().copied().fold(f64::INFINITY, f64::min);
                let max = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                (
                    k,
                    json!({"mean": mean, "min": min, "max": max, "n": xs.len()}),
                )
            })
            .collect();
        let mut verdicts: BTreeMap<&str, u32> = BTreeMap::new();
        for s in &runs {
            *verdicts.entry(s.verdict.as_str()).or_insert(0) += 1;
        }
        out.insert(
            arm.to_string(),
            json!({
                "runs": runs.iter().map(|s| &s.run_id).collect::<Vec<_>>(),
                "valid_runs": valid.len(),
                "invalid": runs.iter().filter(|s| !s.valid).map(|s| &s.run_id).collect::<Vec<_>>(),
                "verdicts": verdicts,
                "metrics": metrics,
            }),
        );
    }
    Value::Object(out)
}

fn slug(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config::load(&Config::default_path()).expect("fleet.toml loads")
    }

    #[test]
    fn the_committed_specs_load() {
        let cfg = cfg();
        let specs = load_specs(&cfg).expect("every spec in specs/ parses and validates");
        assert!(!specs.is_empty());
        assert!(specs.iter().all(|s| s.settle_s.is_some()));
        // restore() runs this one by name.
        assert!(matches!(
            find_spec(&cfg, "flightcheck").unwrap().workload,
            Workload::Flightcheck { .. }
        ));
        assert!(find_spec(&cfg, "../fleet").is_err());
    }

    #[test]
    fn settle_waits_only_for_the_remainder() {
        assert_eq!(settle_remaining(None, 150), 0, "nothing disturbed yet");
        assert_eq!(settle_remaining(Some(0), 150), 150);
        assert_eq!(settle_remaining(Some(100), 150), 50);
        assert_eq!(
            settle_remaining(Some(150), 150),
            0,
            "exactly settled may start"
        );
        assert_eq!(settle_remaining(Some(10_000), 150), 0);
    }

    #[test]
    fn overrides_apply_and_are_recorded_flat() {
        let cfg = cfg();
        let spec = find_spec(&cfg, "video-3stream").unwrap();
        let (s, applied) = apply_overrides(
            &spec,
            &json!({"duration_s": 30, "workload": {"fps": 30}, "workload.width": 960}),
        )
        .unwrap();
        assert_eq!(s.duration_s, 30);
        assert_eq!(
            s.repeats, spec.repeats,
            "untouched fields keep the spec's value"
        );
        let Workload::Flightcheck {
            fps,
            width,
            quality,
            ..
        } = &s.workload
        else {
            panic!("kind preserved")
        };
        assert_eq!((*fps, *width, *quality), (30.0, 960, 50));
        assert_eq!(
            Value::Object(applied),
            json!({"duration_s": 30, "workload.fps": 30, "workload.width": 960})
        );

        let (same, none) = apply_overrides(&spec, &Value::Null).unwrap();
        assert!(none.is_empty());
        assert_eq!(same.workload, spec.workload);

        // Anything that would silently measure something else is refused.
        for bad in [
            json!({"workload.kind": "idle"}),
            json!({"arms": []}),
            json!({"workload": {"fsp": 30}}),
            json!({"repeats": 0}),
            json!({"duration_s": "long"}),
            json!(["duration_s", 30]),
        ] {
            assert!(apply_overrides(&spec, &bad).is_err(), "{bad} accepted");
        }
        let idle = find_spec(&cfg, "idle-baseline").unwrap();
        assert!(apply_overrides(&idle, &json!({"workload.fps": 30})).is_err());
    }

    #[test]
    fn arms_interleave_abba_from_the_cell_in_force() {
        let arms = [
            Arm {
                label: "ndn-fwd".into(),
                cell: "ndn-fwd wifi".into(),
            },
            Arm {
                label: "nfd".into(),
                cell: "nfd wifi".into(),
            },
        ];
        assert_eq!(arm_order(&arms, 1, "nfd wifi"), [1, 0]);
        assert_eq!(arm_order(&arms, 2, "nfd wifi"), [0, 1]);
        assert_eq!(arm_order(&arms, 1, "ndn-fwd wifi"), [0, 1]);
        // Two repeats from either cell: one switch out, one back, ending where it began (ABAB
        // would take three switches plus an I9 restore, each with a full settle).
        for start in ["nfd wifi", "ndn-fwd wifi"] {
            let (mut cur, mut switches) = (start, 0);
            for rep in 1..=2 {
                for i in arm_order(&arms, rep, start) {
                    if arms[i].cell != cur {
                        switches += 1;
                        cur = &arms[i].cell;
                    }
                }
            }
            assert_eq!((switches, cur), (2, start));
        }
    }

    /// flightcheck.py --json, as written for a 3-stream run (trimmed to what a summary reads).
    fn flightcheck_fixture(first_frame_wuas: f64) -> Value {
        let video = |fps: f64, first: f64| {
            json!({"n": 900, "fps": fps, "kbps": 1200.5, "first_frame_s": first, "gap_p50": 0.07,
                   "gap_p95": 0.3, "gap_max": 1.4, "stutters_over_1s": 1, "warmup_stutters": 0,
                   "valid": first >= 0.0, "status": "OK"})
        };
        let backlog = first_frame_wuas < 0.0;
        json!({
            "label": "t", "seconds": 90.0, "vehicles": ["wuas-01", "iuas-01", "iuas-02"],
            "telemetry": {"iuas-01": {"n": 300, "rate": 3.3, "gaps_p50": 0.3, "gaps_p95": 0.4,
                                      "gaps_max": 0.9, "gaps_over_2s": 0, "status": "OK"}},
            "video": {"iuas-01": video(10.0, 0.4), "iuas-02": video(14.0, 0.5),
                      "wuas-01": video(13.0, first_frame_wuas)},
            "aggregate": {"streams": 3, "fps": 37.0, "kbps": 3601.5, "worst_gap": 1.4},
            "audio": {}, "events": {"video.control": 3},
            "armed": {"iuas-01": false, "iuas-02": false, "wuas-01": false},
            "stop_before": {"quiet": true}, "stop_after": {"quiet": true},
            "invalid_reasons": if backlog { json!(["video wuas-01: backlog"]) } else { json!([]) },
            "verdict": if backlog { "INVALID" } else { "FLIGHT-READY" },
        })
    }

    /// counters::delta for two nodes: ndn-fwd with LP reliability, NFD without it.
    fn deltas_fixture() -> Value {
        let face = |c: Value| {
            json!({"faceid": 4, "faceid_before": 4, "header": "UDP permanent non-local point-to-point",
                                     "new": false, "reset": false, "counters": c, "gauges": {}})
        };
        let mut reset_face = face(
            json!({"reliability.resent": 1.0, "reliability.fast-retx": 20.0,
                                         "reliability.dup-rx": 5.0, "reliability.gave-up": 1.0,
                                         "in.data": 100.0, "out.data": 100.0}),
        );
        reset_face["reset"] = json!(true);
        json!({
            "nodes": {
                "minidronesys-03": {
                    "forwarder": "ndn-fwd", "forwarder_changed": null, "window_ms": 95000,
                    "faces": {
                        "udp4://192.168.1.14:6363": face(json!({
                            "reliability.resent": 10.0, "reliability.fast-retx": 60.0,
                            "reliability.dup-rx": 25.0, "reliability.gave-up": 2.0,
                            "reliability.acks-tx": 500.0, "reliability.acks-rx": 400.0,
                            "reassembly.completed": 800.0, "reassembly.timed-out": 3.0,
                            "in.data": 5000.0, "out.data": 200.0})),
                        "udp4://192.168.1.11:6363": reset_face,
                        // Multicast discovery and the app's local face are not peer links.
                        "udp4://224.0.23.170:56363": face(json!({"out.data": 99999.0, "reliability.gave-up": 999.0})),
                        "unix:///run/ndn/fwd.sock#262": face(json!({"in.data": 77777.0})),
                    },
                    "gone": [], "non_local_totals": {},
                    "cs": {"hits": 30.0, "misses": 70.0}, "cs_gauges": {}, "cs_hit_rate": 0.3, "cs_reset": false,
                },
                "minidronesys-01": {
                    "forwarder": "nfd", "forwarder_changed": null, "window_ms": 95000,
                    "faces": {"udp4://192.168.1.13:6363": face(json!({"in.data": 40.0, "out.data": 900.0}))},
                    "gone": [], "non_local_totals": {},
                    "cs": {"hits": 0.0, "misses": 0.0}, "cs_gauges": {}, "cs_hit_rate": null, "cs_reset": false,
                },
            },
            "missing": [],
        })
    }

    fn spec_with_criteria() -> Spec {
        let mut spec = find_spec(&cfg(), "video-3stream").unwrap();
        spec.criteria = toml::from_str(
            r#"
            verdict = { equals = "FLIGHT-READY" }
            "workload.video.*.fps" = { min = 12.0 }
            "nodes.*.derived.gave_up_per_out_data" = { max = 0.05 }
            "workload.not.measured" = { max = 1 }
            "#,
        )
        .unwrap();
        spec
    }

    #[test]
    fn summary_aggregates_peer_faces_and_derives_ratios() {
        let s = summarize(
            &spec_with_criteria(),
            &flightcheck_fixture(0.6),
            Some(&deltas_fixture()),
        );
        assert_eq!(
            (s["verdict"].as_str(), s["valid"].as_bool()),
            (Some("FLIGHT-READY"), Some(true))
        );
        assert_eq!(s["workload"]["video"]["iuas-02"]["fps"], json!(14.0));
        assert_eq!(s["workload"]["aggregate"]["fps"], json!(37.0));

        let gcs = &s["nodes"]["minidronesys-03"];
        // Two unicast peers summed; multicast and the local app face excluded.
        assert_eq!(gcs["reliability"]["fast-retx"], json!(80.0));
        assert_eq!(gcs["reliability"]["gave-up"], json!(3.0));
        assert_eq!(gcs["out"]["data"], json!(300.0));
        assert_eq!(gcs["in"]["data"], json!(5100.0));
        assert_eq!(gcs["reassembly"]["timed-out"], json!(3.0));
        assert_eq!(gcs["derived"]["dup_rx_per_fast_retx"], json!(30.0 / 80.0));
        assert_eq!(gcs["derived"]["gave_up_per_out_data"], json!(0.01));
        assert_eq!(
            gcs["cs"],
            json!({"hits": 30.0, "misses": 70.0, "hit_rate": 0.3})
        );
        assert_eq!(gcs["peer_faces"]["count"], json!(2));
        assert_eq!(
            gcs["peer_faces"]["reset"],
            json!(["udp4://192.168.1.11:6363"])
        );
        // NFD exposes no LP reliability: absent, not zero, and no ratio made up from it.
        let nfd = &s["nodes"]["minidronesys-01"];
        assert_eq!(nfd["reliability"]["gave-up"], Value::Null);
        assert_eq!(nfd["derived"]["gave_up_per_out_data"], Value::Null);
        assert_eq!(nfd["out"]["data"], json!(900.0));

        let c = &s["criteria"];
        assert_eq!(c["verdict"]["pass"], json!(true));
        assert_eq!(c["workload.video.*.fps"]["pass"], json!(false));
        assert_eq!(
            c["workload.video.*.fps"]["failed"],
            json!(["workload.video.iuas-01.fps"])
        );
        // The NFD node's null ratio is a failure to measure, not a pass.
        assert_eq!(
            c["nodes.*.derived.gave_up_per_out_data"]["failed"],
            json!(["nodes.minidronesys-01.derived.gave_up_per_out_data"])
        );
        assert_eq!(
            c["workload.not.measured"]["pass"],
            json!(false),
            "no value is not a pass"
        );
        assert_eq!(s["criteria_pass"], json!(false));
    }

    #[test]
    fn a_backlog_sample_summarises_as_invalid() {
        let s = summarize(&spec_with_criteria(), &flightcheck_fixture(-2.5), None);
        assert_eq!(
            (s["verdict"].as_str(), s["valid"].as_bool()),
            (Some("INVALID"), Some(false))
        );
        assert_eq!(s["workload"]["video"]["wuas-01"]["valid"], json!(false));
        assert_eq!(s["nodes"], Value::Null);
    }

    fn sample(arm: &str, repeat: u32, valid: bool, fps: f64, resent: f64) -> SampleResult {
        SampleResult {
            run_id: format!("{arm}-{repeat}-{valid}"),
            arm: arm.into(),
            repeat,
            attempt: 1,
            valid,
            verdict: if valid { "FLIGHT-READY" } else { "INVALID" }.into(),
            summary: json!({
                "workload": {"aggregate": {"fps": fps}, "video": {"iuas-01": {"valid": valid}}},
                "nodes": {"minidronesys-03": {"reliability": {"resent": resent}, "cs": {"hit_rate": null}}},
            }),
        }
    }

    #[test]
    fn arms_aggregate_valid_samples_only() {
        let samples = [
            sample("ndn-fwd", 1, true, 30.0, 100.0),
            sample("nfd", 1, true, 40.0, 7.0),
            sample("nfd", 2, false, 400.0, 9999.0),
            sample("nfd", 2, true, 42.0, 9.0),
            sample("ndn-fwd", 2, true, 33.0, 50.0),
        ];
        let a = aggregate_arms(&samples);
        let fps = &a["ndn-fwd"]["metrics"]["workload.aggregate.fps"];
        assert_eq!(
            *fps,
            json!({"mean": 31.5, "min": 30.0, "max": 33.0, "n": 2})
        );
        assert_eq!(
            a["nfd"]["metrics"]["nodes.minidronesys-03.reliability.resent"],
            json!({"mean": 8.0, "min": 7.0, "max": 9.0, "n": 2})
        );
        assert_eq!(a["nfd"]["valid_runs"], json!(2));
        assert_eq!(a["nfd"]["invalid"], json!(["nfd-2-false"]));
        assert_eq!(
            a["nfd"]["verdicts"],
            json!({"FLIGHT-READY": 2, "INVALID": 1})
        );
        assert!(
            a["nfd"]["metrics"]
                .as_object()
                .unwrap()
                .keys()
                .all(|k| !k.contains("hit_rate") && !k.contains("valid"))
        );
    }

    #[test]
    fn fabric_bench_reports_parse() {
        // Verbatim from miniMUAS/tools/fabric-bench/results/20260918T162359Z.
        let flow = parse_flow(
            "--- ndn-iperf results ---\n  mode:        forward (client→server)\n  duration:    21.41s\n  \
             transferred: 32.19 MB (33749640 bytes)\n  throughput:  12.61 Mbps\n  \
             packets:     4102 sent, 4091 received, 11 lost (0.3% loss)\n  retransmits: 98\n  \
             RTT:         avg=96633us min=6245us max=1214417us\n               p50=42055us p95=683120us p99=960472us\n",
        );
        assert_eq!(
            flow,
            json!({"mbps": 12.61, "loss_pct": 0.3, "retransmits": 98.0,
                   "rtt_p50_ms": 42.055, "rtt_p95_ms": 683.12, "rtt_p99_ms": 960.472})
        );
        let ping = parse_ping(
            "--- /fabricbench/57754 ping statistics ---\n\
             100 packets transmitted, 97 received, 1 nacked, 2% lost, 1% nacked, time 427.09 ms\n\
             rtt min/avg/max/mdev = 3.56506/4.2709/21.5244/1.82137 ms\n",
        );
        assert_eq!(
            ping,
            json!({"transmitted": 100.0, "lost_pct": 2.0, "nacked_pct": 1.0,
                   "rtt_min_ms": 3.56506, "rtt_avg_ms": 4.2709, "rtt_max_ms": 21.5244})
        );
    }
}
