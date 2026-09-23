//! Preflight and health: one batched ssh per node, the dashboard's armed flags (I2), the verdict,
//! the post-mutation health gate, and `begin_mutation` (I1 + I2 in one call).

use std::collections::BTreeMap;
use std::pin::Pin;
use std::task::Poll;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use serde::Serialize;
use serde_json::{Value, json};

use crate::cells;
use crate::config::{Config, Node};
use crate::remote;
use crate::state::{FleetLock, Recorder, now_ms};

/// Clock disagreement beyond which a node is flagged: node timestamps in logs and captures are
/// correlated across the fleet, and GPS/mavlink-set clocks have drifted before.
const CLOCK_LIMIT_MS: i64 = 500;

#[derive(Serialize, Clone, Debug)]
pub struct ArmedProbe {
    pub answered: bool,
    pub vehicles: Vec<String>,
    pub armed: Vec<String>,
    pub detail: String,
}

#[derive(Serialize, Clone, Debug)]
pub struct NodeStatus {
    pub name: String,
    pub reachable: bool,
    pub system: Option<String>,
    pub ndn_fwd_pkg: Option<String>,
    pub desired: Option<String>,
    pub good: Option<String>,
    pub active: Option<String>,
    /// Loaded units only (fabric + role units) → `ActiveState`.
    pub units: BTreeMap<String, String>,
    pub muas_strategy: Option<String>,
    pub muas_nexthops: usize,
    pub oversize_journals: Vec<(String, u64)>,
    pub uptime_s: Option<u64>,
    /// Node clock − local clock (ms), midpoint estimate.
    pub clock_offset_ms: Option<i64>,
    /// Half-width of the interval the true offset lies in (ssh setup time makes it wide).
    pub clock_uncertainty_ms: Option<i64>,
    pub problems: Vec<String>,
}

#[derive(Serialize, Clone, Debug)]
pub struct FleetStatus {
    /// `READY` | `DEGRADED` | `UNREACHABLE`
    pub verdict: String,
    pub reasons: Vec<String>,
    pub nodes: Vec<NodeStatus>,
    pub armed: ArmedProbe,
    pub since_disturbance_s: Option<u64>,
    pub last_deploy: Option<String>,
}

/// Await every future concurrently, results in input order. Borrow-friendly (no `'static`
/// spawn), which is what per-node ssh over a `&Config` needs.
pub async fn join_all<F: Future>(futs: impl IntoIterator<Item = F>) -> Vec<F::Output> {
    let mut futs: Vec<Pin<Box<F>>> = futs.into_iter().map(Box::pin).collect();
    let mut out: Vec<Option<F::Output>> = futs.iter().map(|_| None).collect();
    std::future::poll_fn(|cx| {
        let mut pending = false;
        for (f, o) in futs.iter_mut().zip(out.iter_mut()) {
            if o.is_none() {
                match f.as_mut().poll(cx) {
                    Poll::Ready(v) => *o = Some(v),
                    Poll::Pending => pending = true,
                }
            }
        }
        if pending {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    })
    .await;
    out.into_iter()
        .map(|o| o.expect("polled to completion"))
        .collect()
}

/// Fabric units plus every role unit of the restart order, deduplicated, in order.
fn watched_units(cfg: &Config) -> Vec<String> {
    let mut units: Vec<String> = [
        "nfd",
        "muas-fabric-ndn-fwd",
        "muas-fabric-ndn-fwd-radio",
        "muas-fabric-watchdog",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    for u in role_units(cfg) {
        if !units.contains(u) {
            units.push(u.clone());
        }
    }
    units
}

fn role_units(cfg: &Config) -> impl Iterator<Item = &String> {
    let r = &cfg.restart;
    r.forwarder
        .iter()
        .chain(&r.agents)
        .chain(&r.control)
        .chain(&r.dashboard)
}

/// One script, one ssh: tagged `@key value` lines. `@t0`/`@t1` bracket it for the clock offset.
fn status_script(cfg: &Config) -> String {
    let units = watched_units(cfg).join(" ");
    let glob = &cfg.fleet.journal_glob;
    let limit = cfg.fleet.journal_truncate_bytes;
    format!(
        r#"echo "@t0 $(date +%s%3N)"
echo "@host $(hostname)"
echo "@system $(readlink /run/current-system)"
echo "@fwdpkg $(systemctl show -p ExecStart --value muas-fabric-ndn-fwd.service 2>/dev/null | grep -oE '/nix/store/[a-z0-9]+-ndn-fwd[^ /]*' | head -n1)"
for f in desired good active; do echo "@$f $(cat /var/lib/minimuas/fabric/$f 2>/dev/null)"; done
systemctl show -p Id,LoadState,ActiveState {units} 2>/dev/null | sed 's/^/@unit /'
case "$(cat /var/lib/minimuas/fabric/active 2>/dev/null)" in
  nfd*) nfdc strategy list 2>/dev/null | grep -E 'prefix=/muas[[:space:]]' | sed 's/^/@strategy /'
        nfdc route list 2>/dev/null | grep -E 'prefix=/muas[[:space:]]' | sed 's/^/@route /' ;;
  *) ndn-ctl strategy list 2>/dev/null | grep -E '^/muas[[:space:]]' | sed 's/^/@strategy /'
     ndn-ctl route list 2>/dev/null | grep -E '^/muas[[:space:]]' | sed 's/^/@route /' ;;
esac
for j in {glob}; do [ -f "$j" ] || continue; s=$(stat -c %s "$j"); [ "$s" -gt {limit} ] && echo "@journal $j $s"; done
echo "@uptime $(cut -d' ' -f1 /proc/uptime)"
echo "@t1 $(date +%s%3N)"
"#
    )
}

/// Parse the tagged output of [`status_script`]. `local_ms` = local clock before/after the ssh.
fn parse_status(cfg: &Config, node: &Node, text: &str, local_ms: (u64, u64)) -> NodeStatus {
    let mut st = NodeStatus {
        name: node.name.clone(),
        reachable: true,
        system: None,
        ndn_fwd_pkg: None,
        desired: None,
        good: None,
        active: None,
        units: BTreeMap::new(),
        muas_strategy: None,
        muas_nexthops: 0,
        oversize_journals: Vec::new(),
        uptime_s: None,
        clock_offset_ms: None,
        clock_uncertainty_ms: None,
        problems: Vec::new(),
    };
    let nonempty = |s: &str| (!s.is_empty()).then(|| s.to_string());
    let (mut t0, mut t1, mut host) = (None::<i64>, None::<i64>, None::<String>);
    let mut unit: (Option<String>, Option<String>, Option<String>) = (None, None, None);
    let (mut strategy_rows, mut route_rows) = (String::new(), String::new());
    let flush_unit = |unit: &mut (Option<String>, Option<String>, Option<String>),
                      units: &mut BTreeMap<String, String>| {
        if let (Some(id), Some(load), Some(active)) = (unit.0.take(), unit.1.take(), unit.2.take())
            && load != "not-found"
        {
            units.insert(id.trim_end_matches(".service").to_string(), active);
        }
    };
    for line in text.lines() {
        let Some(tagged) = line.strip_prefix('@') else {
            continue;
        };
        let (tag, rest) = tagged.split_once(' ').unwrap_or((tagged, ""));
        let rest = rest.trim();
        match tag {
            "t0" => t0 = rest.parse().ok(),
            "t1" => t1 = rest.parse().ok(),
            "host" => host = nonempty(rest),
            "system" => st.system = nonempty(rest),
            "fwdpkg" => st.ndn_fwd_pkg = nonempty(rest),
            "desired" => st.desired = nonempty(rest),
            "good" => st.good = nonempty(rest),
            "active" => st.active = nonempty(rest),
            "unit" => match rest.split_once('=') {
                Some(("Id", v)) => {
                    if unit.0.is_some() {
                        flush_unit(&mut unit, &mut st.units);
                    }
                    unit.0 = Some(v.to_string());
                }
                Some(("LoadState", v)) => unit.1 = Some(v.to_string()),
                Some(("ActiveState", v)) => unit.2 = Some(v.to_string()),
                _ => flush_unit(&mut unit, &mut st.units),
            },
            "strategy" => {
                strategy_rows.push_str(rest);
                strategy_rows.push('\n');
            }
            "route" => {
                route_rows.push_str(rest);
                route_rows.push('\n');
            }
            "journal" => {
                if let Some((path, size)) = rest.rsplit_once(' ') {
                    st.oversize_journals
                        .push((path.to_string(), size.parse().unwrap_or(0)));
                }
            }
            "uptime" => st.uptime_s = rest.parse::<f64>().ok().map(|s| s as u64),
            _ => {}
        }
    }
    flush_unit(&mut unit, &mut st.units);
    st.muas_strategy = cells::muas_strategy(&strategy_rows);
    st.muas_nexthops = cells::muas_nexthops(&route_rows);

    // The node stamped t0 after local l0 and t1 before local l1, so the true offset lies in
    // [t1 − l1, t0 − l0]; report its midpoint and half-width, flag only a certain violation.
    let (l0, l1) = (local_ms.0 as i64, local_ms.1 as i64);
    if let (Some(t0), Some(t1)) = (t0, t1) {
        let (lo, hi) = (t1 - l1, t0 - l0);
        st.clock_offset_ms = Some((lo + hi) / 2);
        st.clock_uncertainty_ms = Some((hi - lo).max(0) / 2);
        if lo > CLOCK_LIMIT_MS || hi < -CLOCK_LIMIT_MS {
            st.problems.push(format!(
                "clock off by {}±{} ms (limit {CLOCK_LIMIT_MS})",
                (lo + hi) / 2,
                (hi - lo).max(0) / 2
            ));
        }
    }

    if host.as_deref() != Some(node.name.as_str()) {
        st.problems.push(format!(
            "hostname {:?} is not the inventory name",
            host.unwrap_or_default()
        ));
    }
    match (&st.desired, &st.active) {
        (_, None) => st.problems.push("no active fabric cell".into()),
        (Some(d), Some(a)) if d != a => st
            .problems
            .push(format!("active cell '{a}' != desired '{d}'")),
        _ => {}
    }
    st.problems.extend(fabric_problems(cfg, &st));
    for (path, size) in &st.oversize_journals {
        st.problems.push(format!(
            "journal {path} is {size} B (> {} B; blocks agent startup)",
            cfg.fleet.journal_truncate_bytes
        ));
    }
    for u in role_units(cfg) {
        if let Some(state) = st.units.get(u)
            && state != "active"
        {
            st.problems.push(format!("role unit {u} is {state}"));
        }
    }
    st
}

/// The fabric conditions the health gate waits for: `/muas` multicast toward every other node,
/// and the cell's forwarder unit running.
fn fabric_problems(cfg: &Config, st: &NodeStatus) -> Vec<String> {
    let mut p = Vec::new();
    match &st.muas_strategy {
        Some(s) if s.contains("/multicast/") => {}
        Some(s) => p.push(format!("/muas strategy is {s}, not multicast")),
        None => p.push("/muas has no strategy entry".into()),
    }
    let want = cfg.nodes.len().saturating_sub(1);
    if st.muas_nexthops < want {
        p.push(format!(
            "/muas has {} nexthops (want >= {want})",
            st.muas_nexthops
        ));
    }
    if let Some(cell) = &st.active {
        match cells::forwarder_unit(cell) {
            Ok(unit) => match st.units.get(unit).map(String::as_str) {
                Some("active") => {}
                Some(state) => p.push(format!("forwarder unit {unit} is {state}")),
                None => p.push(format!("forwarder unit {unit} is not installed")),
            },
            Err(e) => p.push(format!("{e:#}")),
        }
    }
    p
}

async fn probe_node(cfg: &Config, node: &Node) -> NodeStatus {
    let script = status_script(cfg);
    let l0 = now_ms();
    let out = remote::ssh(cfg, node, &script, Duration::from_secs(30)).await;
    let l1 = now_ms();
    match out {
        Ok(o) if o.stdout.contains("@t1 ") => parse_status(cfg, node, &o.stdout, (l0, l1)),
        other => {
            let why = match other {
                Ok(o) => o.stdout_ok().err().map_or_else(
                    || "status script did not finish".into(),
                    |e| format!("{e:#}"),
                ),
                Err(e) => format!("{e:#}"),
            };
            NodeStatus {
                name: node.name.clone(),
                reachable: false,
                system: None,
                ndn_fwd_pkg: None,
                desired: None,
                good: None,
                active: None,
                units: BTreeMap::new(),
                muas_strategy: None,
                muas_nexthops: 0,
                oversize_journals: Vec::new(),
                uptime_s: None,
                clock_offset_ms: None,
                clock_uncertainty_ms: None,
                problems: vec![format!("unreachable: {why}")],
            }
        }
    }
}

/// Every node's status, in inventory order, probed in parallel.
pub async fn probe_nodes(cfg: &Config) -> Vec<NodeStatus> {
    join_all(cfg.nodes.iter().map(|n| probe_node(cfg, n))).await
}

/// Airframe vehicles from the inventory: I2 needs an answer for each of them.
fn fleet_vehicles(cfg: &Config) -> Vec<String> {
    cfg.nodes
        .iter()
        .filter(|n| !n.is_gcs())
        .map(|n| n.vehicle.clone())
        .collect()
}

/// Interpret flightcheck `--probe --json` output. Answered only if every inventory airframe
/// (and every vehicle the dashboard advertises) reported a boolean `armed`.
fn read_probe(cfg: &Config, v: &Value) -> ArmedProbe {
    let mut vehicles: Vec<String> = v["vehicles"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|x| x.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();
    for fv in fleet_vehicles(cfg) {
        if !vehicles.contains(&fv) {
            vehicles.push(fv);
        }
    }
    let mut armed = Vec::new();
    let mut unknown = Vec::new();
    for veh in &vehicles {
        match v["armed"].get(veh).and_then(Value::as_bool) {
            Some(true) => armed.push(veh.clone()),
            Some(false) => {}
            None => unknown.push(veh.clone()),
        }
    }
    let mut detail = if unknown.is_empty() {
        format!("{} vehicle(s) reported armed state", vehicles.len())
    } else {
        format!("no armed flag from: {}", unknown.join(", "))
    };
    if let Some(e) = v["error"].as_str() {
        detail = format!("{detail}; flightcheck: {e}");
    }
    ArmedProbe {
        answered: unknown.is_empty(),
        vehicles,
        armed,
        detail,
    }
}

/// Ask the GCS dashboard for every vehicle's armed flag (flightcheck `--probe`, passive).
pub async fn probe_armed(cfg: &Config) -> ArmedProbe {
    let tmp = std::env::temp_dir().join(format!(
        "ndn-fleet-probe-{}-{}.json",
        std::process::id(),
        now_ms()
    ));
    let (fc, tmp_s, port) = (
        cfg.workloads.flightcheck.to_string_lossy().into_owned(),
        tmp.to_string_lossy().into_owned(),
        cfg.dashboard.port.to_string(),
    );
    let args = [
        fc.as_str(),
        "--host",
        cfg.dashboard.host.as_str(),
        "--port",
        port.as_str(),
        "--probe",
        "--seconds",
        "5",
        "--json",
        tmp_s.as_str(),
    ];
    let out = remote::local(
        &cfg.workloads.python,
        &args,
        None,
        &[],
        Duration::from_secs(30),
    )
    .await;
    let parsed = std::fs::read(&tmp)
        .ok()
        .and_then(|b| serde_json::from_slice::<Value>(&b).ok());
    let _ = std::fs::remove_file(&tmp);
    match parsed {
        Some(v) => read_probe(cfg, &v),
        None => ArmedProbe {
            answered: false,
            vehicles: fleet_vehicles(cfg),
            armed: Vec::new(),
            detail: match out {
                Ok(o) => format!(
                    "flightcheck --probe wrote no JSON ({}): {}",
                    o.status
                        .map_or_else(|| "timed out".into(), |s| format!("exit {s}")),
                    last_line(&o.stderr)
                ),
                Err(e) => format!("flightcheck --probe did not run: {e:#}"),
            },
        },
    }
}

fn last_line(s: &str) -> &str {
    s.trim().lines().last().unwrap_or("")
}

/// Fleet-level verdict over per-node statuses. `armed` is `None` for the health gate, which does
/// not probe the dashboard.
fn assemble(rec: &Recorder, nodes: Vec<NodeStatus>, armed: Option<ArmedProbe>) -> FleetStatus {
    let mut reasons: Vec<String> = Vec::new();
    for n in &nodes {
        for p in &n.problems {
            reasons.push(format!("{}: {p}", n.name));
        }
    }
    let reachable: Vec<&NodeStatus> = nodes.iter().filter(|n| n.reachable).collect();
    let distinct = |f: fn(&NodeStatus) -> Option<&String>| {
        let mut v: Vec<&str> = reachable
            .iter()
            .filter_map(|n| f(n).map(String::as_str))
            .collect();
        v.sort_unstable();
        v.dedup();
        v.len() > 1
    };
    if distinct(|n| n.active.as_ref()) {
        reasons.push(format!(
            "cells differ across nodes: {}",
            reachable
                .iter()
                .map(|n| format!("{}={}", n.name, n.active.as_deref().unwrap_or("?")))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    // I4: after a rollout every node runs the same ndn-fwd package.
    if distinct(|n| n.ndn_fwd_pkg.as_ref()) {
        reasons.push("nodes run different ndn-fwd packages".into());
    }
    if let Some(a) = &armed {
        if !a.armed.is_empty() {
            reasons.push(format!("ARMED: {}", a.armed.join(", ")));
        }
        if !a.answered {
            reasons.push(format!("armed probe unanswered: {}", a.detail));
        }
    }
    let verdict = if nodes.iter().any(|n| !n.reachable) {
        "UNREACHABLE"
    } else if reasons.is_empty() {
        "READY"
    } else {
        "DEGRADED"
    };
    let st = rec.state();
    FleetStatus {
        verdict: verdict.into(),
        reasons,
        nodes,
        armed: armed.unwrap_or_else(|| ArmedProbe {
            answered: false,
            vehicles: Vec::new(),
            armed: Vec::new(),
            detail: "not probed (health gate)".into(),
        }),
        since_disturbance_s: rec.seconds_since_disturbance(),
        last_deploy: st.last_deploy,
    }
}

/// Preflight: every node (parallel, one ssh each) and the armed flags, with a verdict.
pub async fn fleet_status(cfg: &Config, rec: &Recorder) -> Result<FleetStatus> {
    let (nodes, armed) = tokio::join!(probe_nodes(cfg), probe_armed(cfg));
    Ok(assemble(rec, nodes, Some(armed)))
}

/// Poll every 10 s until every node runs `expected_cell` with `/muas` multicast toward every
/// other node and the cell's forwarder unit active. Err with the outstanding reasons on timeout.
pub async fn health_gate(
    cfg: &Config,
    rec: &Recorder,
    expected_cell: &str,
    timeout: Duration,
    log: &(dyn Fn(&str) + Sync),
) -> Result<FleetStatus> {
    let started = Instant::now();
    loop {
        let nodes = probe_nodes(cfg).await;
        let mut outstanding = Vec::new();
        for n in &nodes {
            if !n.reachable {
                outstanding.push(format!("{}: unreachable", n.name));
                continue;
            }
            if n.active.as_deref() != Some(expected_cell) {
                outstanding.push(format!(
                    "{}: active cell {:?}, want '{expected_cell}'",
                    n.name,
                    n.active.as_deref().unwrap_or("none")
                ));
            }
            for p in fabric_problems(cfg, n) {
                outstanding.push(format!("{}: {p}", n.name));
            }
        }
        let waited = started.elapsed();
        if outstanding.is_empty() {
            log(&format!(
                "health gate '{expected_cell}': healthy after {}s",
                waited.as_secs()
            ));
            rec.ledger(
                "health_gate",
                json!({ "cell": expected_cell, "ok": true, "waited_s": waited.as_secs() }),
            );
            return Ok(assemble(rec, nodes, None));
        }
        if waited >= timeout {
            rec.ledger(
                "health_gate",
                json!({ "cell": expected_cell, "ok": false, "reasons": outstanding }),
            );
            bail!(
                "health gate '{expected_cell}' not met after {}s: {}",
                waited.as_secs(),
                outstanding.join("; ")
            );
        }
        log(&format!(
            "health gate '{expected_cell}' waiting ({}s): {}",
            waited.as_secs(),
            outstanding.join("; ")
        ));
        tokio::time::sleep(Duration::from_secs(10)).await;
    }
}

/// I1 then I2: take the fleet lock, then read the vehicles' armed flags. Refuses if any vehicle
/// is armed, or if the dashboard cannot answer and the caller did not assert `assume_disarmed`
/// (which is ledgered).
pub async fn begin_mutation(
    cfg: &Config,
    rec: &Recorder,
    holder: &str,
    assume_disarmed: bool,
) -> Result<(FleetLock, ArmedProbe)> {
    let lock = rec.lock(holder)?;
    let probe = probe_armed(cfg).await;
    rec.ledger("armed_probe", json!({ "holder": holder, "probe": probe }));
    if !probe.armed.is_empty() {
        bail!(
            "refused: vehicle(s) ARMED: {} — the fabric is never changed under an armed vehicle (I2)",
            probe.armed.join(", ")
        );
    }
    if !probe.answered {
        if !assume_disarmed {
            bail!(
                "refused: the dashboard could not confirm every vehicle is disarmed ({}). Pass \
                 assume_disarmed=true only if you know every vehicle is on the ground (I2)",
                probe.detail
            );
        }
        rec.ledger(
            "assume_disarmed",
            json!({ "holder": holder, "detail": probe.detail }),
        );
    }
    Ok((lock, probe))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config::load(&Config::default_path()).unwrap()
    }

    /// Tagged output shaped like the live GCS (minidronesys-03, ndn-fwd wifi, 2026-09-23).
    fn gcs_output(active: &str, strategy: &str, routes: &[&str], t: (i64, i64)) -> String {
        let mut s = format!(
            "@t0 {}\n@host minidronesys-03\n@system /nix/store/g2w6-nixos-system-minidronesys-03\n\
             @fwdpkg /nix/store/xvzq-ndn-fwd-unstable\n@desired ndn-fwd wifi\n@good ndn-fwd wifi\n@active {active}\n",
            t.0
        );
        for (id, load, st) in [
            ("nfd.service", "loaded", "inactive"),
            ("muas-fabric-ndn-fwd.service", "loaded", "active"),
            ("muas-fabric-apply.service", "loaded", "active"),
            ("muas-v2-agent.service", "not-found", "inactive"),
            ("muas-v2-dashboard.service", "loaded", "failed"),
        ] {
            s +=
                &format!("@unit Id={id}\n@unit LoadState={load}\n@unit ActiveState={st}\n@unit \n");
        }
        s += &format!("@strategy {strategy}\n");
        for r in routes {
            s += &format!("@route {r}\n");
        }
        s += "@journal /var/lib/minimuas/log/gcs-dashboard.jsonl 4698873\n@uptime 177625.57\n";
        s += &format!("@t1 {}\n", t.1);
        s
    }

    #[test]
    fn status_lines_parse_into_problems() {
        let cfg = cfg();
        let gcs = cfg.gcs().clone();
        let out = gcs_output(
            "ndn-fwd wifi",
            "/muas             /localhost/nfd/strategy/multicast/v=5",
            &[
                "/muas  4  100",
                "/muas  2  100",
                "/muas  3  100",
                "/muas  3  100",
            ],
            (10_100, 10_300),
        );
        let st = parse_status(&cfg, &gcs, &out, (10_000, 10_600));
        assert_eq!(
            st.muas_strategy.as_deref(),
            Some("/localhost/nfd/strategy/multicast/v=5")
        );
        assert_eq!(st.muas_nexthops, 3, "duplicate nexthop rows count once");
        assert!(
            !st.units.contains_key("muas-v2-agent"),
            "not-found units are absent"
        );
        assert_eq!(st.units["muas-fabric-ndn-fwd"], "active");
        assert_eq!(st.uptime_s, Some(177_625));
        // offset ∈ [10300−10600, 10100−10000] = [−300, 100]
        assert_eq!(st.clock_offset_ms, Some(-100));
        assert_eq!(st.clock_uncertainty_ms, Some(200));
        let p = st.problems.join(" | ");
        assert!(p.contains("role unit muas-v2-dashboard is failed"), "{p}");
        assert!(p.contains("gcs-dashboard.jsonl"), "{p}");
        assert!(
            !p.contains("clock"),
            "an uncertain offset is not flagged: {p}"
        );
        assert!(!p.contains("/muas"), "{p}");
    }

    #[test]
    fn fabric_problems_catch_wrong_strategy_few_nexthops_and_stopped_forwarder() {
        let cfg = cfg();
        let gcs = cfg.gcs().clone();
        let out = gcs_output(
            "nfd wifi",
            "prefix=/muas strategy=/localhost/nfd/strategy/best-route/v=5",
            &["prefix=/muas nexthop=262 origin=static cost=100"],
            (99_000, 99_100),
        );
        let st = parse_status(&cfg, &gcs, &out, (10_000, 10_600));
        let p = st.problems.join(" | ");
        assert!(p.contains("not multicast"), "{p}");
        assert!(p.contains("1 nexthops (want >= 3)"), "{p}");
        assert!(p.contains("forwarder unit nfd is inactive"), "{p}");
        assert!(
            p.contains("active cell 'nfd wifi' != desired 'ndn-fwd wifi'"),
            "{p}"
        );
        assert!(
            p.contains("clock off"),
            "a certain 88 s offset is flagged: {p}"
        );
    }

    #[test]
    fn armed_probe_needs_every_airframe_answered() {
        let cfg = cfg();
        let all = json!({"vehicles": ["iuas-01", "iuas-02", "wuas-01"],
                         "armed": {"iuas-01": false, "iuas-02": false, "wuas-01": false}});
        let p = read_probe(&cfg, &all);
        assert!(p.answered && p.armed.is_empty());

        let null_flag = json!({"vehicles": ["iuas-01", "iuas-02", "wuas-01"],
                               "armed": {"iuas-01": false, "iuas-02": null, "wuas-01": true}});
        let p = read_probe(&cfg, &null_flag);
        assert!(!p.answered);
        assert_eq!(p.armed, vec!["wuas-01"]);

        // The dashboard not advertising an inventory airframe is not an answer for it.
        let missing = json!({"vehicles": ["iuas-01"], "armed": {"iuas-01": false}});
        assert!(!read_probe(&cfg, &missing).answered);
    }
}
