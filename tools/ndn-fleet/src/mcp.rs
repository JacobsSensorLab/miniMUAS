//! The MCP server: JSON-RPC 2.0 over stdio (newline-delimited), `initialize` / `tools/list` /
//! `tools/call` / `ping`, the same hand-rolled shape as `ndn-sim`'s server.
//!
//! [`Fleet::call_tool`] is the single entry to every operation; the CLI calls it too, so both
//! fronts run the identical, ledgered path (I8). Mutating tools start jobs whose first act is
//! `status::begin_mutation` (lock + armed check, I1/I2) and which hold the lock until they end.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::config::Config;
use crate::counters::{self, NodeCounters};
use crate::jobs::{JobCtx, Jobs};
use crate::remote::{self, sh_quote};
use crate::state::Recorder;
use crate::{cells, deploy, measure, results, status};

const PROTOCOL_VERSION: &str = "2024-11-05";
const JOB_WAIT_DEFAULT_S: u64 = 30;
/// Below common MCP client request timeouts (60 s).
const JOB_WAIT_MAX_S: u64 = 55;

const INSTRUCTIONS: &str = "\
ndn-fleet is the only way to deploy to or measure on the miniMUAS fleet (4 ODROID nodes: three \
airframes + GCS). It enforces PROTOCOL.md: one mutation at a time (fleet lock), never under an \
armed vehicle, a settle clock after every disturbance, only pushed+clean+pinned code, canary \
first / GCS last, streams stopped around every sample, counters reported as deltas, everything \
recorded under tools/ndn-fleet/results/, and the fleet left on its known-good cell.

Canonical sequence:
1. fleet_status — preflight; proceed only on READY (or understand every DEGRADED reason).
2. fleet_deploy {} — returns a PLAN (revs old->new, commits, nodes); read it.
3. fleet_deploy {plan_id} — executes that plan as a job; poll fleet_job until done.
4. fleet_measure {spec} — runs a spec from fleet_specs as a job (settle, stop streams, counters \
before/after, workload); poll fleet_job.
5. fleet_results {} / {id} / {compare:[old,new]} — read and compare runs.
6. fleet_restore {} — known-good cell + health gate + flight check before you leave.
Long operations return {job_id}; fleet_job {job_id, wait_s} long-polls (<= 55 s per call). \
Never ssh to the nodes to mutate them yourself.";

pub struct Fleet {
    pub cfg: Arc<Config>,
    pub rec: Arc<Recorder>,
    pub jobs: Arc<Jobs>,
    /// `mcp` | `cli`, recorded with every call.
    source: &'static str,
}

#[derive(Deserialize)]
struct RpcRequest {
    id: Option<Value>,
    method: String,
    #[serde(default)]
    params: Value,
}

#[derive(Deserialize)]
struct ToolCall {
    name: String,
    #[serde(default)]
    arguments: Value,
}

type ToolResult = Result<Value, String>;

fn err(e: anyhow::Error) -> String {
    format!("{e:#}")
}

fn opt_str<'a>(args: &'a Value, key: &str) -> Result<Option<&'a str>, String> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s)),
        Some(_) => Err(format!("'{key}' must be a string")),
    }
}

fn req_str<'a>(args: &'a Value, key: &str) -> Result<&'a str, String> {
    opt_str(args, key)?.ok_or_else(|| format!("missing required '{key}'"))
}

fn opt_bool(args: &Value, key: &str) -> Result<bool, String> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(false),
        Some(Value::Bool(b)) => Ok(*b),
        Some(_) => Err(format!("'{key}' must be a boolean")),
    }
}

fn opt_u64(args: &Value, key: &str) -> Result<Option<u64>, String> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v
            .as_u64()
            .map(Some)
            .ok_or_else(|| format!("'{key}' must be a non-negative integer")),
    }
}

/// Snapshot ids are file stems under `state/snapshots/`; nothing path-like.
fn check_id(id: &str) -> Result<&str, String> {
    if id.is_empty() || id.contains('/') || id.contains("..") {
        return Err(format!("invalid id '{id}'"));
    }
    Ok(id)
}

const ASSUME_DISARMED: &str = "Proceed when the dashboard cannot report every vehicle's armed \
flag. Only set true if you KNOW every vehicle is on the ground; it is recorded in the ledger. \
An armed vehicle always refuses, regardless.";

impl Fleet {
    pub fn new(cfg: Config, source: &'static str) -> anyhow::Result<Arc<Fleet>> {
        let rec = Recorder::open(&cfg)?;
        Ok(Arc::new(Fleet {
            cfg: Arc::new(cfg),
            jobs: Jobs::new(rec.clone()),
            rec,
            source,
        }))
    }

    /// The `tools/list` payload.
    pub fn tool_catalog() -> Value {
        let cell_enum = json!(["ndn-fwd wifi", "ndn-fwd radio", "nfd wifi", "nfd radio"]);
        json!([
            {
                "name": "fleet_status",
                "description": "Read-only preflight (no lock). Per node: reachable, /run/current-system, ndn-fwd package, fabric cell desired/good/active, fabric + role units, /muas strategy (must be multicast) and nexthop count (must be >= nodes-1), oversize journals, uptime, clock offset. Fleet: vehicles' armed flags from the dashboard, seconds since the last disturbance (settle clock), last deploy id, and a verdict READY / DEGRADED / UNREACHABLE with every reason. Call first; mutate only from READY or with each DEGRADED reason understood.",
                "inputSchema": { "type": "object", "properties": {}, "additionalProperties": false }
            },
            {
                "name": "fleet_counters",
                "description": "Read-only. Snapshots face and content-store counters on every node (parsed from ndn-ctl, or nfdc under an NFD cell), saves it as state/snapshots/<snapshot_id>.json and returns the id. With `since` (an earlier snapshot_id) returns the DELTA over the window instead of lifetime totals: per node, per face keyed by remote URI (faceids change on restart), faces new/gone/reset, CS hits/misses delta and the window's CS hit rate. Lifetime totals are not comparable across runs; use deltas.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "since": { "type": "string", "description": "snapshot_id returned by an earlier fleet_counters call; the result is the delta from it" }
                    },
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_logs",
                "description": "Read-only. `journalctl -u <unit>` on one node since a time, newest `lines` lines, optionally filtered by an extended regex.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "node": { "type": "string", "description": "node name (minidronesys-03) or vehicle (iuas-01, gcs)" },
                        "unit": { "type": "string", "description": "systemd unit, e.g. muas-fabric-ndn-fwd, muas-fabric-apply, muas-v2-agent, muas-v2-dashboard" },
                        "since": { "type": "string", "description": "journalctl --since value; default \"15 min ago\"" },
                        "lines": { "type": "integer", "minimum": 1, "maximum": 2000, "description": "default 200" },
                        "grep": { "type": "string", "description": "extended regex applied on the node before taking the last `lines`" }
                    },
                    "required": ["node", "unit"],
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_specs",
                "description": "Read-only. The measurement protocols (specs/*.toml): name, description, settle_s, duration_s, repeats, arms (cells compared), workload. A spec is the unit of comparability: runs of the same spec at different builds compare; ad-hoc changes go in fleet_measure `overrides` and are recorded.",
                "inputSchema": { "type": "object", "properties": {}, "additionalProperties": false }
            },
            {
                "name": "fleet_results",
                "description": "Read-only. No arguments: list runs and deploys/plans, newest first. `id`: show one run (manifest with fleet identity + summary + files) or deploy (record + plan). `compare`: [old_run_id, new_run_id] flattens both summary.json to dotted numeric keys and reports old/new/delta/pct per shared metric, with warnings when the specs or overrides differ.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "id": { "type": "string" },
                        "compare": { "type": "array", "items": { "type": "string" }, "minItems": 2, "maxItems": 2 },
                        "limit": { "type": "integer", "minimum": 1, "description": "list length per kind; default 20" }
                    },
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_job",
                "description": "Status (running|succeeded|failed), result, error and log tail of a background job. Long-polls: returns as soon as the job finishes or after wait_s. Call repeatedly until status is not running.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "job_id": { "type": "string" },
                        "wait_s": { "type": "integer", "minimum": 0, "maximum": JOB_WAIT_MAX_S, "description": "default 30" }
                    },
                    "required": ["job_id"],
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_deploy",
                "description": "Deploy ndn-fwd (and optionally miniMUAS) to the fleet. TWO-PHASE. (1) Without plan_id: read-only PLAN — resolves revs (default: current HEAD of every pinned repo's local checkout), verifies each is clean and pushed, computes old->new revs, commit lists and SRI hashes, and returns the plan with its plan_id. Nothing is changed. (2) With plan_id: EXECUTES that plan as a job (returns job_id): fleet lock + armed check, edit pins (rev+hash together), commit + push the config repo, build every closure, canary airframe -> verify -> other airframes -> GCS, verify /run/current-system and one ndn-fwd path fleet-wide, truncate oversize journals, restart roles in order (forwarder -> agents -> controller/gcs -> dashboard), health gate. Stamps the settle clock. Always read the plan before executing it.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "plan_id": { "type": "string", "description": "execute this plan (from a previous plan call)" },
                        "revs": { "type": "object", "additionalProperties": { "type": "string" }, "description": "plan only: repo name -> rev (commit/branch/tag) overriding the local HEAD" },
                        "minimuas_rev": { "type": "string", "description": "plan only: also move the miniMUAS input to this rev" },
                        "assume_disarmed": { "type": "boolean", "description": ASSUME_DISARMED }
                    },
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_set_cell",
                "description": "Switch every node's fabric cell (`muas-fabric set`): airframes first, GCS last; then a health gate (every node on the cell, /muas multicast toward every other node, forwarder unit active, dashboard reporting every vehicle's armed flag; 300 s). Runs as a job (fleet lock + armed check); stamps the settle clock. Prefer fleet_measure arms for A/B comparisons: they restore the starting cell themselves.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "cell": { "type": "string", "enum": cell_enum },
                        "assume_disarmed": { "type": "boolean", "description": ASSUME_DISARMED }
                    },
                    "required": ["cell"],
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_measure",
                "description": "Run a measurement spec (see fleet_specs) as a job (fleet lock + armed check). For each repeat, arms interleaved so each comparison shares a window: set the arm's cell if needed; WAIT for the settle clock (now - last disturbance >= settle_s; never measures inside churn); STOP all video streams and wait for a quiet dashboard; snapshot counters on every node; run the workload; snapshot counters again; stop streams again; write runs/<id>/ (manifest with fleet identity + deploy in force, counters before/after, DELTAS, workload JSON, raw outputs, summary). Samples draining a backlog are rejected. Restores the starting cell at the end. Returns the run ids in the job result.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "spec": { "type": "string", "description": "spec name from fleet_specs" },
                        "overrides": {
                            "type": "object",
                            "description": "ad-hoc changes for this run, recorded in the manifest. Allowed: duration_s, repeats, settle_s, and workload parameters as {\"workload\": {\"fps\": 10}} or \"workload.fps\": 10. Arms, workload kind and criteria are the spec's identity and cannot be overridden (use another spec).",
                            "properties": {
                                "duration_s": { "type": "integer", "minimum": 1 },
                                "repeats": { "type": "integer", "minimum": 1 },
                                "settle_s": { "type": "integer", "minimum": 0 },
                                "workload": { "type": "object" }
                            }
                        },
                        "assume_disarmed": { "type": "boolean", "description": ASSUME_DISARMED }
                    },
                    "required": ["spec"],
                    "additionalProperties": false
                }
            },
            {
                "name": "fleet_restore",
                "description": "Return the fleet to its known-good cell on every node, health gate, and run the flight check (operator-path verdict). Runs as a job (fleet lock + armed check); stamps the settle clock. Call before ending a session that changed anything.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "assume_disarmed": { "type": "boolean", "description": ASSUME_DISARMED }
                    },
                    "additionalProperties": false
                }
            }
        ])
    }

    /// Execute one tool. `Ok` is structured JSON; `Err` a message surfaced as an `isError` result.
    /// Every call and its outcome are ledgered (I8).
    pub async fn call_tool(&self, name: &str, args: &Value) -> ToolResult {
        self.rec.ledger(
            "tool_call",
            json!({ "source": self.source, "tool": name, "args": args }),
        );
        let out = self.dispatch(name, args).await;
        match &out {
            Ok(v) => self.rec.ledger(
                "tool_ok",
                json!({ "source": self.source, "tool": name, "job_id": v.get("job_id") }),
            ),
            Err(e) => self.rec.ledger(
                "tool_error",
                json!({ "source": self.source, "tool": name, "error": e }),
            ),
        }
        out
    }

    async fn dispatch(&self, name: &str, args: &Value) -> ToolResult {
        let (cfg, rec) = (&*self.cfg, &*self.rec);
        match name {
            "fleet_status" => status::fleet_status(cfg, rec)
                .await
                .map(to_value)
                .map_err(err),
            "fleet_counters" => self.counters(opt_str(args, "since")?).await,
            "fleet_logs" => self.logs(args).await,
            "fleet_specs" => measure::load_specs(cfg).map(to_value).map_err(err),
            "fleet_results" => self.results(args),
            "fleet_job" => {
                let id = req_str(args, "job_id")?;
                let wait = opt_u64(args, "wait_s")?
                    .unwrap_or(JOB_WAIT_DEFAULT_S)
                    .min(JOB_WAIT_MAX_S);
                self.jobs
                    .view(id, Duration::from_secs(wait))
                    .await
                    .map(to_value)
                    .ok_or_else(|| format!("no job '{id}'"))
            }
            "fleet_deploy" => self.deploy(args).await,
            "fleet_set_cell" => {
                let cell = req_str(args, "cell")?.to_string();
                cells::parse_cell(&cell).map_err(err)?;
                self.start_mutation(
                    "cell",
                    opt_bool(args, "assume_disarmed")?,
                    move |cfg, rec, job| async move {
                        cells::set_cell_all(&cfg, &rec, &job, &cell).await
                    },
                )
            }
            "fleet_measure" => {
                let spec = measure::find_spec(cfg, req_str(args, "spec")?).map_err(err)?;
                let overrides = match args.get("overrides") {
                    None | Some(Value::Null) => json!({}),
                    Some(o @ Value::Object(_)) => o.clone(),
                    Some(_) => return Err("'overrides' must be an object".into()),
                };
                // Reject a bad override now, not after the lock and armed probe.
                measure::apply_overrides(&spec, &overrides).map_err(err)?;
                self.start_mutation(
                    "measure",
                    opt_bool(args, "assume_disarmed")?,
                    move |cfg, rec, job| async move {
                        measure::run(&cfg, &rec, &job, spec, overrides).await
                    },
                )
            }
            "fleet_restore" => self.start_mutation(
                "restore",
                opt_bool(args, "assume_disarmed")?,
                |cfg, rec, job| async move { measure::restore(&cfg, &rec, &job).await },
            ),
            other => Err(format!("unknown tool '{other}'")),
        }
    }

    /// Start a mutating op as a job. A live lock refuses up front (I1); inside the job,
    /// `begin_mutation` takes the lock for real (closing the race) and checks armed flags (I2),
    /// and the lock is held until `op` returns.
    fn start_mutation<F, Fut>(&self, kind: &str, assume_disarmed: bool, op: F) -> ToolResult
    where
        F: FnOnce(Arc<Config>, Arc<Recorder>, JobCtx) -> Fut + Send + 'static,
        Fut: Future<Output = anyhow::Result<Value>> + Send + 'static,
    {
        if let Some(h) = self.rec.lock_holder() {
            return Err(format!(
                "refused: fleet is locked by {} (pid {}); one mutation at a time (I1). Poll \
                 fleet_job {{\"job_id\": \"{}\"}} and retry when it finishes",
                h.holder, h.pid, h.holder
            ));
        }
        let (cfg, rec, jobs) = (self.cfg.clone(), self.rec.clone(), self.jobs.clone());
        let id = self.jobs.spawn(kind, move |job| async move {
            let (lock, armed) =
                status::begin_mutation(&cfg, &rec, &job.id, assume_disarmed).await?;
            job.log(format!(
                "fleet lock taken; armed probe: {} (armed: [{}])",
                armed.detail,
                armed.armed.join(", ")
            ));
            // Under the lock, before touching anything: repair what a job whose process died
            // left behind (streams on, half-written runs, unknown settle state).
            let r = measure::recover_abandoned(&cfg, &rec, &job, &jobs).await?;
            if r["abandoned"] != json!([]) || r["incomplete_runs"] != json!([]) {
                job.log(format!("recovered: {r}"));
            }
            let result = op(cfg, rec, job).await;
            drop(lock);
            result
        });
        Ok(json!({
            "job_id": id,
            "status": "running",
            "next": format!("poll fleet_job {{\"job_id\": \"{id}\"}} until status is not running"),
        }))
    }

    async fn deploy(&self, args: &Value) -> ToolResult {
        let assume = opt_bool(args, "assume_disarmed")?;
        if let Some(plan_id) = opt_str(args, "plan_id")? {
            if args.get("revs").is_some() || args.get("minimuas_rev").is_some() {
                return Err(
                    "revs/minimuas_rev belong to the plan call; a plan_id executes the \
                            plan exactly as recorded"
                        .into(),
                );
            }
            let plan_id = check_id(plan_id)?.to_string();
            if !self
                .rec
                .dir()
                .join(format!("deploys/{plan_id}.plan.json"))
                .exists()
            {
                return Err(format!(
                    "no plan '{plan_id}': call fleet_deploy without plan_id first"
                ));
            }
            return self.start_mutation("deploy", assume, move |cfg, rec, job| async move {
                deploy::execute(&cfg, &rec, &job, &plan_id).await
            });
        }
        let revs: BTreeMap<String, String> = match args.get("revs") {
            None | Some(Value::Null) => BTreeMap::new(),
            Some(v) => serde_json::from_value(v.clone())
                .map_err(|e| format!("'revs' must map repo names to rev strings: {e}"))?,
        };
        let minimuas = opt_str(args, "minimuas_rev")?.map(String::from);
        let plan = deploy::plan(&self.cfg, &self.rec, revs, minimuas)
            .await
            .map_err(err)?;
        let id = plan.id.clone();
        let mut v = to_value(plan);
        v["next"] = json!(format!(
            "review this plan; to execute it call fleet_deploy {{\"plan_id\": \"{id}\"}}"
        ));
        Ok(v)
    }

    async fn counters(&self, since: Option<&str>) -> ToolResult {
        let before: Option<Vec<NodeCounters>> = match since {
            Some(s) => Some(
                self.rec
                    .read_json(&format!("state/snapshots/{}.json", check_id(s)?))
                    .map_err(err)?,
            ),
            None => None,
        };
        let snap = counters::snapshot_fleet(&self.cfg).await.map_err(err)?;
        let id = self.rec.new_id("counters");
        self.rec
            .write_json(&format!("state/snapshots/{id}.json"), &snap)
            .map_err(err)?;
        match before {
            Some(b) => Ok(json!({
                "snapshot_id": id,
                "since": since,
                "delta": counters::delta(&b, &snap),
            })),
            None => {
                // `raw` stays in the saved snapshot; the response carries the parsed form.
                let nodes: Vec<Value> = snap
                    .iter()
                    .map(|n| {
                        let mut v = to_value(n);
                        if let Some(o) = v.as_object_mut() {
                            o.remove("raw");
                        }
                        v
                    })
                    .collect();
                Ok(json!({ "snapshot_id": id, "nodes": nodes }))
            }
        }
    }

    async fn logs(&self, args: &Value) -> ToolResult {
        let want = req_str(args, "node")?;
        let node = self
            .cfg
            .nodes
            .iter()
            .find(|n| n.name == want || n.vehicle == want)
            .ok_or_else(|| format!("unknown node '{want}'"))?;
        let unit = req_str(args, "unit")?;
        let since = opt_str(args, "since")?.unwrap_or("15 min ago");
        let lines = opt_u64(args, "lines")?.unwrap_or(200).clamp(1, 2000);
        let mut cmd = format!(
            "journalctl -u {} --since {} --no-pager -o short-iso",
            sh_quote(unit),
            sh_quote(since)
        );
        match opt_str(args, "grep")? {
            Some(re) => cmd += &format!(" | grep -E -e {} | tail -n {lines}", sh_quote(re)),
            None => cmd += &format!(" -n {lines}"),
        }
        let out = remote::ssh(&self.cfg, node, &cmd, Duration::from_secs(60))
            .await
            .map_err(err)?;
        // grep exits 1 on no match: that is an empty answer, not a failure.
        if !out.ok() && !(out.status == Some(1) && out.stderr.trim().is_empty()) {
            return Err(err(out.stdout_ok().map(|_| ()).unwrap_err()));
        }
        Ok(json!({ "node": node.name, "unit": unit, "command": cmd, "log": out.stdout }))
    }

    fn results(&self, args: &Value) -> ToolResult {
        if let Some(c) = args.get("compare") {
            let ids: Vec<String> = serde_json::from_value(c.clone())
                .map_err(|_| "'compare' must be [old_run_id, new_run_id]".to_string())?;
            let [a, b] = ids.as_slice() else {
                return Err("'compare' must be [old_run_id, new_run_id]".into());
            };
            return results::compare(&self.rec, check_id(a)?, check_id(b)?).map_err(err);
        }
        if let Some(id) = opt_str(args, "id")? {
            return results::show(&self.rec, check_id(id)?).map_err(err);
        }
        let limit = opt_u64(args, "limit")?.unwrap_or(20) as usize;
        results::list(&self.rec, limit.max(1)).map_err(err)
    }

    /// Handle one JSON-RPC 2.0 request; empty string for notifications (no `id`).
    pub async fn handle_rpc(&self, request: &str) -> String {
        let req: RpcRequest = match serde_json::from_str(request) {
            Ok(r) => r,
            Err(e) => return rpc_error(Value::Null, -32700, &format!("parse error: {e}")),
        };
        let Some(id) = req.id.clone() else {
            return String::new();
        };
        match req.method.as_str() {
            "initialize" => rpc_ok(
                id,
                json!({
                    "protocolVersion": PROTOCOL_VERSION,
                    "capabilities": { "tools": {} },
                    "serverInfo": { "name": "ndn-fleet", "version": env!("CARGO_PKG_VERSION") },
                    "instructions": INSTRUCTIONS,
                }),
            ),
            "ping" => rpc_ok(id, json!({})),
            "tools/list" => rpc_ok(id, json!({ "tools": Self::tool_catalog() })),
            "tools/call" => {
                let call: ToolCall = match serde_json::from_value(req.params) {
                    Ok(c) => c,
                    Err(e) => return rpc_error(id, -32602, &format!("invalid params: {e}")),
                };
                let args = if call.arguments.is_null() {
                    json!({})
                } else {
                    call.arguments
                };
                let (text, is_error) = match self.call_tool(&call.name, &args).await {
                    Ok(v) => (serde_json::to_string_pretty(&v).unwrap_or_default(), false),
                    Err(e) => (e, true),
                };
                rpc_ok(
                    id,
                    json!({ "content": [ { "type": "text", "text": text } ], "isError": is_error }),
                )
            }
            other => rpc_error(id, -32601, &format!("method not found: {other}")),
        }
    }

    /// Serve newline-delimited JSON-RPC on stdin/stdout. Requests are handled concurrently (a
    /// `fleet_job` long-poll must not stall `fleet_status`). On EOF, answers in-flight requests
    /// and waits for running jobs before returning.
    pub async fn serve_stdio(self: Arc<Self>) -> anyhow::Result<()> {
        use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<String>();
        let writer = tokio::spawn(async move {
            let mut stdout = tokio::io::stdout();
            while let Some(line) = rx.recv().await {
                stdout.write_all(line.as_bytes()).await?;
                stdout.write_all(b"\n").await?;
                stdout.flush().await?;
            }
            std::io::Result::Ok(())
        });
        let mut lines = BufReader::new(tokio::io::stdin()).lines();
        while let Some(line) = lines.next_line().await.context("reading stdin")? {
            if line.trim().is_empty() {
                continue;
            }
            let (me, tx) = (self.clone(), tx.clone());
            tokio::spawn(async move {
                let response = me.handle_rpc(&line).await;
                if !response.is_empty() {
                    let _ = tx.send(response);
                }
            });
        }
        drop(tx);
        writer.await.context("stdout writer")??;
        let running = self.jobs.running();
        if !running.is_empty() {
            eprintln!(
                "ndn-fleet: client gone; finishing running job(s) {} before exit",
                running.join(", ")
            );
            self.jobs.wait_idle().await;
        }
        Ok(())
    }
}

fn to_value<T: serde::Serialize>(v: T) -> Value {
    serde_json::to_value(v).unwrap_or(Value::Null)
}

fn rpc_ok(id: Value, result: Value) -> String {
    json!({ "jsonrpc": "2.0", "id": id, "result": result }).to_string()
}

fn rpc_error(id: Value, code: i64, message: &str) -> String {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message } }).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fleet() -> Arc<Fleet> {
        let mut cfg = Config::load(&Config::default_path()).unwrap();
        cfg.fleet.results_dir =
            std::env::temp_dir().join(format!("ndn-fleet-mcp-{}", std::process::id()));
        Fleet::new(cfg, "test").unwrap()
    }

    async fn rpc(f: &Fleet, req: Value) -> Value {
        serde_json::from_str(&f.handle_rpc(&req.to_string()).await).unwrap()
    }

    #[tokio::test]
    async fn rpc_shape_initialize_list_errors_and_notifications() {
        let f = fleet();
        let init = rpc(
            &f,
            json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{}}),
        )
        .await;
        assert_eq!(init["result"]["protocolVersion"], PROTOCOL_VERSION);
        assert_eq!(init["result"]["serverInfo"]["name"], "ndn-fleet");

        let list = rpc(&f, json!({"jsonrpc":"2.0","id":"a","method":"tools/list"})).await;
        assert_eq!(list["id"], "a", "string ids echo back");
        let tools = list["result"]["tools"].as_array().unwrap();
        for t in tools {
            assert_eq!(t["inputSchema"]["type"], "object", "{}", t["name"]);
        }
        assert_eq!(tools.len(), 10);

        // A bad tool argument is a tool error (isError), not a JSON-RPC error.
        let bad = rpc(
            &f,
            json!({"jsonrpc":"2.0","id":2,"method":"tools/call",
                   "params":{"name":"fleet_set_cell","arguments":{"cell":"nfd ethernet"}}}),
        )
        .await;
        assert_eq!(bad["result"]["isError"], true);
        assert!(bad.get("error").is_none());

        let missing = rpc(&f, json!({"jsonrpc":"2.0","id":3,"method":"nope"})).await;
        assert_eq!(missing["error"]["code"], -32601);

        let note = f
            .handle_rpc(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#)
            .await;
        assert!(note.is_empty());
        let ledger = std::fs::read_to_string(f.rec.dir().join("ledger.jsonl")).unwrap();
        assert!(ledger.contains("\"tool_error\"") && ledger.contains("fleet_set_cell"));
    }

    #[tokio::test]
    async fn a_mutation_is_refused_up_front_while_the_lock_is_held() {
        let f = fleet();
        let _held = f.rec.lock("20260923T000000Z-deploy").unwrap();
        let e = f
            .call_tool("fleet_restore", &json!({}))
            .await
            .expect_err("locked fleet refuses");
        assert!(e.contains("20260923T000000Z-deploy"), "{e}");
    }
}
