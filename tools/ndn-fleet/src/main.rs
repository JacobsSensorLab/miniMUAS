//! `ndn-fleet`: the MCP server (`ndn-fleet mcp`) and a CLI over the same tool calls. Every CLI
//! command is a `Fleet::call_tool`, so it is ledgered and enforced exactly like an MCP call.

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use ndn_fleet::config::Config;
use ndn_fleet::mcp::Fleet;
use serde_json::{Map, Value, json};

const USAGE: &str = "\
usage: ndn-fleet [--config <fleet.toml>] <command>

  mcp                                     serve MCP (JSON-RPC 2.0) on stdio
  status                                  preflight: nodes, cells, units, armed flags, verdict
  counters [since=<snapshot_id>]          counter snapshot (or delta since a snapshot)
  logs <node|vehicle> <unit> [since=..] [lines=N] [grep=RE]
  specs                                   measurement specs
  results [<id> | compare <old> <new>]    list / show / compare runs and deploys
  deploy plan [<repo>=<rev> ...] [minimuas=<rev>]
  deploy run <plan_id> [--assume-disarmed]
  cell <nfd|ndn-fwd> <wifi|radio> [--assume-disarmed]
  measure <spec> [key=value ...] [--assume-disarmed]   (dotted keys nest: workload.fps=10)
  restore [--assume-disarmed]
  job <id> [--follow]

Mutating commands run as jobs (lock + armed check) and stream the job log to stderr until done.";

#[tokio::main]
async fn main() -> ExitCode {
    match run().await {
        Ok(code) => code,
        Err(e) => {
            eprintln!("ndn-fleet: {e:#}");
            ExitCode::from(2)
        }
    }
}

/// `k=v` → (k, JSON value): valid JSON literals (numbers, bools, arrays) stay typed, anything
/// else is a string.
fn kv(arg: &str) -> Result<(&str, Value)> {
    let (k, v) = arg
        .split_once('=')
        .with_context(|| format!("expected key=value, got '{arg}'"))?;
    Ok((
        k,
        serde_json::from_str(v).unwrap_or_else(|_| Value::String(v.into())),
    ))
}

/// `k=v` arguments kept verbatim as strings (ids, regexes, journalctl times).
fn str_args(args: &[&str]) -> Result<Map<String, Value>> {
    args.iter()
        .map(|a| {
            let (k, v) = a
                .split_once('=')
                .with_context(|| format!("expected key=value, got '{a}'"))?;
            Ok((k.to_string(), json!(v)))
        })
        .collect()
}

/// `a.b.c=v` into nested objects.
fn insert_dotted(obj: &mut Map<String, Value>, key: &str, v: Value) -> Result<()> {
    match key.split_once('.') {
        None => {
            obj.insert(key.into(), v);
        }
        Some((head, rest)) => {
            let child = obj.entry(head).or_insert_with(|| json!({}));
            let Some(child) = child.as_object_mut() else {
                bail!("override '{head}' is both a value and an object");
            };
            insert_dotted(child, rest, v)?;
        }
    }
    Ok(())
}

async fn run() -> Result<ExitCode> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut config = Config::default_path();
    if let Some(i) = args.iter().position(|a| a == "--config") {
        let path = args.get(i + 1).context("--config needs a path")?;
        config = PathBuf::from(path);
        args.drain(i..=i + 1);
    }
    let assume_disarmed = match args.iter().position(|a| a == "--assume-disarmed") {
        Some(i) => {
            args.remove(i);
            true
        }
        None => false,
    };
    let follow = match args.iter().position(|a| a == "--follow") {
        Some(i) => {
            args.remove(i);
            true
        }
        None => false,
    };
    let argv: Vec<&str> = args.iter().map(String::as_str).collect();
    if matches!(argv.as_slice(), [] | ["-h" | "--help" | "help", ..]) {
        println!("{USAGE}");
        return Ok(ExitCode::SUCCESS);
    }

    let cfg = Config::load(&config)?;
    let source = if argv[0] == "mcp" { "mcp" } else { "cli" };
    let fleet = Fleet::new(cfg, source)?;

    let (tool, targs) = match argv.as_slice() {
        ["mcp"] => {
            fleet.serve_stdio().await?;
            return Ok(ExitCode::SUCCESS);
        }
        ["status"] => ("fleet_status", json!({})),
        ["counters", rest @ ..] => ("fleet_counters", Value::Object(str_args(rest)?)),
        ["logs", node, unit, rest @ ..] => {
            let mut a = str_args(rest)?;
            a.insert("node".into(), json!(node));
            a.insert("unit".into(), json!(unit));
            if let Some(n) = a.get("lines").and_then(Value::as_str) {
                let n: u64 = n.parse().context("lines=N needs an integer")?;
                a.insert("lines".into(), json!(n));
            }
            ("fleet_logs", Value::Object(a))
        }
        ["specs"] => ("fleet_specs", json!({})),
        ["results"] => ("fleet_results", json!({})),
        ["results", "compare", a, b] => ("fleet_results", json!({ "compare": [a, b] })),
        ["results", id] => ("fleet_results", json!({ "id": id })),
        ["deploy", "plan", rest @ ..] => {
            let mut revs = Map::new();
            let mut a = Map::new();
            for r in rest {
                let (k, v) = r
                    .split_once('=')
                    .with_context(|| format!("expected <repo>=<rev>, got '{r}'"))?;
                if k == "minimuas" {
                    a.insert("minimuas_rev".into(), json!(v));
                } else {
                    revs.insert(k.into(), json!(v));
                }
            }
            if !revs.is_empty() {
                a.insert("revs".into(), Value::Object(revs));
            }
            ("fleet_deploy", Value::Object(a))
        }
        ["deploy", "run", plan_id] => (
            "fleet_deploy",
            json!({ "plan_id": plan_id, "assume_disarmed": assume_disarmed }),
        ),
        ["cell", fwd, link] => (
            "fleet_set_cell",
            json!({ "cell": format!("{fwd} {link}"), "assume_disarmed": assume_disarmed }),
        ),
        ["measure", spec, rest @ ..] => {
            let mut overrides = Map::new();
            for r in rest {
                let (k, v) = kv(r)?;
                insert_dotted(&mut overrides, k, v)?;
            }
            (
                "fleet_measure",
                json!({ "spec": spec, "overrides": overrides, "assume_disarmed": assume_disarmed }),
            )
        }
        ["restore"] => (
            "fleet_restore",
            json!({ "assume_disarmed": assume_disarmed }),
        ),
        ["job", id] => {
            let wait = if follow { 55 } else { 0 };
            let mut v = fleet
                .call_tool("fleet_job", &json!({ "job_id": id, "wait_s": wait }))
                .await
                .map_err(anyhow::Error::msg)?;
            while follow && v["status"] == "running" {
                v = fleet
                    .call_tool("fleet_job", &json!({ "job_id": id, "wait_s": wait }))
                    .await
                    .map_err(anyhow::Error::msg)?;
            }
            return Ok(print_job(v, false));
        }
        _ => bail!("unrecognised command\n\n{USAGE}"),
    };

    let v = match fleet.call_tool(tool, &targs).await {
        Ok(v) => v,
        Err(e) => {
            eprintln!("ndn-fleet: {e}");
            return Ok(ExitCode::FAILURE);
        }
    };
    match v.get("job_id").and_then(Value::as_str) {
        Some(job_id) if v.get("status").and_then(Value::as_str) == Some("running") => {
            Ok(follow_job(&fleet, job_id).await)
        }
        _ => {
            println!("{}", serde_json::to_string_pretty(&v)?);
            Ok(ExitCode::SUCCESS)
        }
    }
}

/// Wait for a job this process started; its log already streams to stderr via `JobCtx::log`.
async fn follow_job(fleet: &Arc<Fleet>, id: &str) -> ExitCode {
    loop {
        match fleet.jobs.view(id, Duration::from_secs(55)).await {
            Some(v) if v.finished() => {
                return print_job(serde_json::to_value(&v).unwrap_or_default(), true);
            }
            Some(_) => {}
            None => {
                eprintln!("ndn-fleet: job {id} vanished");
                return ExitCode::FAILURE;
            }
        }
    }
}

/// Print a job view; `own` = this process ran it, so its log already went to stderr.
fn print_job(mut v: Value, own: bool) -> ExitCode {
    if own && let Some(o) = v.as_object_mut() {
        o.remove("log_tail");
    }
    println!("{}", serde_json::to_string_pretty(&v).unwrap_or_default());
    if v["status"] == "succeeded" || v["status"] == "running" {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
