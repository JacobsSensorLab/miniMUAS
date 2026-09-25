//! Fleet-wide forwarder cells and the role-service restart order (PROTOCOL.md I5).
//!
//! A cell is runtime state owned by `muas-fabric` on each node (`/var/lib/minimuas/fabric/…`);
//! this module only drives it, in the fleet's fixed order, and stamps the disturbance (I3).

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use serde::Serialize;
use serde_json::{Value, json};

use crate::config::{Config, Node};
use crate::jobs::JobCtx;
use crate::remote::{self, Output, sh_quote};
use crate::state::Recorder;
use crate::status;

const FABRIC_DIR: &str = "/var/lib/minimuas/fabric";
/// `muas-fabric set` is health-gated with auto-revert; `muas-fabric-apply` has
/// TimeoutStartSec=600 on the nodes, so both may legitimately take minutes.
const FABRIC_APPLY: Duration = Duration::from_secs(660);
const QUICK: Duration = Duration::from_secs(60);
const HEALTH_GATE: Duration = Duration::from_secs(300);
const FORWARDER_SETTLE: Duration = Duration::from_secs(180);
const POLL: Duration = Duration::from_secs(10);
/// Seconds a restarted role unit gets to report `active` (polled every 3 s on the node).
const UNIT_ACTIVE_S: u32 = 90;

/// Split a cell `"<nfd|ndn-fwd> <wifi|radio>"` into (forwarder, link).
pub fn parse_cell(cell: &str) -> Result<(&str, &str)> {
    let mut parts = cell.split_whitespace();
    match (parts.next(), parts.next(), parts.next()) {
        (Some(fwd @ ("nfd" | "ndn-fwd")), Some(link @ ("wifi" | "radio")), None) => Ok((fwd, link)),
        _ => bail!("invalid cell '{cell}': expected \"<nfd|ndn-fwd> <wifi|radio>\""),
    }
}

/// The systemd unit that runs the forwarder of `cell`.
pub fn forwarder_unit(cell: &str) -> Result<&'static str> {
    Ok(match parse_cell(cell)? {
        ("nfd", _) => "nfd",
        (_, "wifi") => "muas-fabric-ndn-fwd",
        _ => "muas-fabric-ndn-fwd-radio",
    })
}

/// Rows of a strategy/route listing for exactly `/muas` (not `/muas/v2/…`), in either the
/// `ndn-ctl` table form (`/muas  …`) or the `nfdc` key=value form (`prefix=/muas …`).
fn muas_rows(text: &str) -> impl Iterator<Item = &str> {
    text.lines()
        .filter(|l| matches!(l.split_whitespace().next(), Some("/muas" | "prefix=/muas")))
}

/// The strategy set on `/muas`, from `ndn-ctl strategy list` or `nfdc strategy list`.
pub fn muas_strategy(text: &str) -> Option<String> {
    muas_rows(text).next().and_then(|l| {
        l.split_whitespace()
            .skip(1)
            .map(|t| t.strip_prefix("strategy=").unwrap_or(t))
            .find(|t| t.starts_with("/localhost/"))
            .map(str::to_string)
    })
}

/// Distinct nexthop faces of `/muas`, from `ndn-ctl route list` (`/muas  <faceid>  <cost>`) or
/// `nfdc route list` (`prefix=/muas nexthop=<faceid> origin=…`). A RIB lists one face once per
/// origin (static + app), so rows would overcount and hide a missing neighbour.
pub fn muas_nexthops(text: &str) -> usize {
    muas_rows(text)
        .filter_map(|l| {
            let mut tokens = l.split_whitespace().skip(1);
            tokens
                .clone()
                .find_map(|t| t.strip_prefix("nexthop="))
                .or_else(|| tokens.next())
                .and_then(|f| f.parse::<u64>().ok())
        })
        .collect::<BTreeSet<_>>()
        .len()
}

/// One node's forwarder, as the canary check and the restart order need it.
#[derive(Debug, Clone, Serialize)]
pub struct ForwarderHealth {
    pub node: String,
    pub cell: Option<String>,
    pub unit: Option<String>,
    pub unit_state: Option<String>,
    pub muas_strategy: Option<String>,
    pub muas_nexthops: usize,
}

impl ForwarderHealth {
    /// The forwarder of the active cell runs and `/muas` is multicast — without multicast the
    /// NDNSF SVS sync group partitions (ndn-rs pin comment: `[[strategy]]` boot config).
    pub fn ok(&self) -> bool {
        self.unit_state.as_deref() == Some("active")
            && self
                .muas_strategy
                .as_deref()
                .is_some_and(|s| s.contains("/multicast"))
    }

    fn summary(&self) -> String {
        format!(
            "{}: cell={} {}={} /muas={} nexthops={}",
            self.node,
            self.cell.as_deref().unwrap_or("?"),
            self.unit.as_deref().unwrap_or("?"),
            self.unit_state.as_deref().unwrap_or("?"),
            self.muas_strategy.as_deref().unwrap_or("none"),
            self.muas_nexthops
        )
    }
}

/// Read one node's forwarder health in a single ssh.
pub async fn forwarder_health(cfg: &Config, node: &Node) -> Result<ForwarderHealth> {
    let script = format!(
        "a=$(cat {FABRIC_DIR}/active 2>/dev/null); echo \"active=$a\"; \
         for u in {FORWARDER_UNITS}; do \
           echo \"unit $u $(systemctl is-active $u 2>/dev/null)\"; done; \
         case \"$a\" in nfd*) t=nfdc;; *) t=ndn-ctl;; esac; \
         echo '--- strategy'; $t strategy list 2>&1; echo '--- routes'; $t route list 2>&1; true"
    );
    let out = remote::ssh(cfg, node, &script, QUICK).await?;
    let text = out.stdout_ok()?;
    let (head, rest) = text.split_once("--- strategy").unwrap_or((text, ""));
    let (strategy, routes) = rest.split_once("--- routes").unwrap_or((rest, ""));
    let cell = head
        .lines()
        .find_map(|l| l.strip_prefix("active="))
        .map(str::trim)
        .filter(|c| !c.is_empty())
        .map(str::to_string);
    let unit = cell.as_deref().and_then(|c| forwarder_unit(c).ok());
    let unit_state = unit.and_then(|u| {
        head.lines().find_map(|l| {
            let mut t = l.split_whitespace();
            (t.next() == Some("unit") && t.next() == Some(u))
                .then(|| t.next().unwrap_or("unknown").to_string())
        })
    });
    Ok(ForwarderHealth {
        node: node.name.clone(),
        cell,
        unit: unit.map(str::to_string),
        unit_state,
        muas_strategy: muas_strategy(strategy),
        muas_nexthops: muas_nexthops(routes),
    })
}

/// Poll `forwarder_health` every 10 s until it is ok or `timeout` elapses.
pub async fn wait_forwarder_healthy(
    cfg: &Config,
    node: &Node,
    timeout: Duration,
    log: &(dyn Fn(&str) + Sync),
) -> Result<ForwarderHealth> {
    let started = Instant::now();
    loop {
        let last = forwarder_health(cfg, node).await;
        match &last {
            Ok(h) if h.ok() => {
                log(&format!("forwarder healthy: {}", h.summary()));
                return last;
            }
            Ok(h) => log(&format!("forwarder not healthy yet: {}", h.summary())),
            Err(e) => log(&format!("{}: forwarder check failed: {e:#}", node.name)),
        }
        if started.elapsed() + POLL > timeout {
            return match last {
                Ok(h) => bail!(
                    "forwarder unhealthy after {}s: {}",
                    timeout.as_secs(),
                    h.summary()
                ),
                Err(e) => Err(e.context(format!("{}: forwarder check", node.name))),
            };
        }
        tokio::time::sleep(POLL).await;
    }
}

/// An `Output` for the record: stdout/stderr bounded so one chatty command cannot bloat it.
pub(crate) fn out_json(o: &Output) -> Value {
    json!({
        "command": o.command,
        "status": o.status,
        "elapsed_ms": o.elapsed_ms,
        "stdout": tail(&o.stdout, 4000),
        "stderr": tail(&o.stderr, 4000),
    })
}

/// The last `max` bytes of `s` (on a char boundary).
pub(crate) fn tail(s: &str, max: usize) -> &str {
    if s.len() <= max {
        return s;
    }
    let mut start = s.len() - max;
    while !s.is_char_boundary(start) {
        start += 1;
    }
    &s[start..]
}

/// `muas-fabric set <cell>` on every node: canary, other airframes, GCS last (the GCS carries
/// the dashboard the operator watches). Nodes already on `cell` are left alone so an arm that
/// needs no switch adds no churn (I3). Caller holds the lock.
pub async fn set_cell_all(cfg: &Config, rec: &Recorder, job: &JobCtx, cell: &str) -> Result<Value> {
    let (fwd, link) = parse_cell(cell)?;
    let mut nodes = Vec::new();
    let mut switched = Vec::new();
    for node in cfg.rollout_order() {
        let read = format!(
            "printf '%s\\n%s\\n' \"$(cat {FABRIC_DIR}/desired 2>/dev/null)\" \"$(cat {FABRIC_DIR}/active 2>/dev/null)\""
        );
        let current = remote::ssh(cfg, node, &read, QUICK).await?;
        let text = current
            .stdout_ok()
            .with_context(|| format!("{}: reading fabric state", node.name))?;
        let mut lines = text.lines().map(str::trim);
        let (desired, active) = (lines.next().unwrap_or(""), lines.next().unwrap_or(""));
        if desired == cell && active == cell {
            job.log(format!("{}: already on '{cell}'", node.name));
            nodes.push(json!({ "node": node.name, "skipped": "already on cell" }));
            continue;
        }
        job.log(format!(
            "{}: muas-fabric set {fwd} {link} (was desired='{desired}' active='{active}')",
            node.name
        ));
        let out = remote::ssh(
            cfg,
            node,
            &format!("sudo -n muas-fabric set {fwd} {link}"),
            FABRIC_APPLY,
        )
        .await?;
        switched.push(node.name.clone());
        nodes.push(json!({ "node": node.name, "from": active, "set": out_json(&out) }));
        if let Err(e) = out.stdout_ok() {
            // Nodes already switched stay switched: the fabric is disturbed either way.
            rec.disturb(
                "cell",
                &format!("{cell} (failed at {}; switched: {switched:?})", node.name),
            );
            return Err(e.context(format!(
                "{}: muas-fabric set {cell} failed; already switched: {switched:?}",
                node.name
            )));
        }
        job.log(format!("{}: set ok in {} ms", node.name, out.elapsed_ms));
    }
    if !switched.is_empty() {
        rec.disturb("cell", cell);
    }
    let health = status::health_gate(cfg, rec, cell, HEALTH_GATE, &|l| job.log(l)).await?;
    Ok(json!({ "cell": cell, "switched": switched, "nodes": nodes, "health": health }))
}

/// What `restart_roles` finds on one node before touching anything.
struct Survey {
    units: BTreeSet<String>,
    /// (path, bytes) of journals over the truncation limit.
    journals: Vec<(String, u64)>,
}

fn parse_survey(text: &str) -> Survey {
    let mut units = BTreeSet::new();
    let mut journals = Vec::new();
    for line in text.lines() {
        if let Some(u) = line.strip_prefix("unit ") {
            units.insert(u.trim().to_string());
        } else if let Some(rest) = line.strip_prefix("journal ")
            && let Some((size, path)) = rest.split_once(' ')
            && let Ok(size) = size.parse()
        {
            journals.push((path.to_string(), size));
        }
    }
    Survey { units, journals }
}

/// Bring the role services across the whole fleet into the I5 order: oversize journals truncated
/// first (they blocked agent startup), then forwarder (fabric apply + health) on every node, then
/// agents, then controller/gcs, then the dashboard. A group starts only when the previous one is
/// up everywhere — the wrong order produced "Targeted ProviderToken is unknown or expired" and
/// wiped prefix registrations. Only units present on a node are touched.
///
/// Each role unit restarts at most once per deploy: a unit that already (re)started after the
/// node's activation and after its forwarder's last start is left alone (see `refresh_script`).
/// Restarting everything unconditionally bounced each role two or three times per deploy
/// (2026-09-23 17:07–17:12 journals), and every bounce is a window where commands are lost.
/// Caller holds the lock.
pub async fn restart_roles(cfg: &Config, rec: &Recorder, job: &JobCtx) -> Result<Value> {
    let mut restarted = false;
    let result = restart_roles_inner(cfg, job, &mut restarted).await;
    if restarted {
        let detail = match &result {
            Ok(_) => "role services restarted (I5 order)".to_string(),
            Err(e) => format!("role restart failed: {e:#}"),
        };
        rec.disturb("restart", &detail);
    }
    result
}

async fn restart_roles_inner(cfg: &Config, job: &JobCtx, restarted: &mut bool) -> Result<Value> {
    let groups: [(&str, &[String]); 4] = [
        ("forwarder", &cfg.restart.forwarder),
        ("agents", &cfg.restart.agents),
        ("control", &cfg.restart.control),
        ("dashboard", &cfg.restart.dashboard),
    ];
    let nodes = cfg.rollout_order();

    let unit_list = groups
        .iter()
        .flat_map(|(_, units)| units.iter())
        .map(|u| sh_quote(u))
        .collect::<Vec<_>>()
        .join(" ");
    // The glob is left unquoted so the node's shell expands it.
    let survey_cmd = format!(
        "for u in {unit_list}; do systemctl cat \"$u\" >/dev/null 2>&1 && echo \"unit $u\"; done; \
         for f in {glob}; do [ -f \"$f\" ] || continue; s=$(stat -c %s \"$f\") || continue; \
           [ \"$s\" -gt {limit} ] && echo \"journal $s $f\"; done; true",
        glob = cfg.fleet.journal_glob,
        limit = cfg.fleet.journal_truncate_bytes,
    );
    let surveyed = status::join_all(
        nodes
            .iter()
            .map(|n| remote::ssh(cfg, n, &survey_cmd, QUICK)),
    )
    .await;
    let mut surveys = BTreeMap::new();
    for (node, out) in nodes.iter().zip(surveyed) {
        let out = out?;
        let text = out
            .stdout_ok()
            .with_context(|| format!("{}: surveying role units", node.name))?;
        surveys.insert(node.name.clone(), parse_survey(text));
    }

    let mut truncated = Vec::new();
    let with_journals: Vec<&&Node> = nodes
        .iter()
        .filter(|n| !surveys[&n.name].journals.is_empty())
        .collect();
    let truncate_cmds: Vec<String> = with_journals
        .iter()
        .map(|n| {
            let files = surveys[&n.name]
                .journals
                .iter()
                .map(|(p, _)| sh_quote(p))
                .collect::<Vec<_>>()
                .join(" ");
            format!("sudo -n truncate -s 0 -- {files}")
        })
        .collect();
    let truncations = status::join_all(
        with_journals
            .iter()
            .zip(&truncate_cmds)
            .map(|(n, cmd)| remote::ssh(cfg, n, cmd, QUICK)),
    )
    .await;
    for (node, out) in with_journals.iter().zip(truncations) {
        let out = out?;
        let journals = &surveys[&node.name].journals;
        out.stdout_ok()
            .with_context(|| format!("{}: truncating journals", node.name))?;
        for (path, size) in journals {
            job.log(format!("{}: truncated {path} ({size} bytes)", node.name));
        }
        truncated.push(json!({ "node": node.name, "journals": journals }));
    }

    let mut done = Vec::new();
    for (group, units) in groups {
        let targets: Vec<(&Node, Vec<&String>)> = nodes
            .iter()
            .map(|n| {
                let present = &surveys[&n.name].units;
                (*n, units.iter().filter(|u| present.contains(*u)).collect())
            })
            .filter(|(_, u): &(&Node, Vec<&String>)| !u.is_empty())
            .collect();
        if targets.is_empty() {
            job.log(format!("{group}: no units present on any node"));
            continue;
        }
        let forwarder = group == "forwarder";
        for (node, units) in &targets {
            let verb = if forwarder { "ensuring" } else { "refreshing" };
            job.log(format!("{group}: {verb} {units:?} on {}", node.name));
        }
        let scripts: Vec<String> = targets
            .iter()
            .map(|(_, u)| {
                if forwarder {
                    ensure_script(u)
                } else {
                    refresh_script(u)
                }
            })
            .collect();
        let outs = status::join_all(
            targets
                .iter()
                .zip(&scripts)
                .map(|((n, _), s)| remote::ssh(cfg, n, s, FABRIC_APPLY)),
        )
        .await;
        let mut group_nodes = Vec::new();
        let mut failures = Vec::new();
        for ((node, units), out) in targets.iter().zip(outs) {
            let out = out?;
            let (bounced, fresh) = parse_refresh(&out.stdout);
            match out.stdout_ok() {
                Ok(_) => job.log(format!(
                    "{group}: {} up in {} ms (restarted {bounced:?}, already fresh {fresh:?})",
                    node.name, out.elapsed_ms
                )),
                Err(e) => failures.push(format!("{}: {e:#}", node.name)),
            }
            *restarted |= !bounced.is_empty();
            group_nodes.push(json!({
                "node": node.name,
                "units": units,
                "restarted": bounced,
                "fresh": fresh,
                "restart": out_json(&out),
            }));
        }
        if !failures.is_empty() {
            bail!("{group} restart failed: {}", failures.join("; "));
        }
        let mut entry = json!({ "group": group, "nodes": group_nodes });
        if forwarder {
            // Fabric apply returning is not enough: the next groups register prefixes and join
            // the /muas sync group, which needs the forwarder up with multicast on /muas.
            let log = |l: &str| job.log(l);
            let healths = status::join_all(
                targets
                    .iter()
                    .map(|(n, _)| wait_forwarder_healthy(cfg, n, FORWARDER_SETTLE, &log)),
            )
            .await;
            let mut ok = Vec::new();
            for h in healths {
                ok.push(h.context("forwarder health after fabric apply")?);
            }
            entry["health"] = json!(ok);
        }
        done.push(entry);
    }
    Ok(json!({ "truncated": truncated, "groups": done }))
}

/// Forwarder daemons a role unit's app face lives on; which one runs depends on the cell.
const FORWARDER_UNITS: &str = "nfd muas-fabric-ndn-fwd muas-fabric-ndn-fwd-radio";

/// Start (never restart) the forwarder-group units, then wait for each to report active.
///
/// `systemctl restart muas-fabric-apply` is not a forwarder restart: muas-fabric.target
/// Requires= apply and every role unit Requires= the target, so systemd propagates the restart
/// to all role units on the node at once — on every node in parallel, which is the opposite of
/// the I5 order (2026-09-23 17:11:43: agents, controller, gcs and dashboard all stopped in the
/// same second as apply). And apply on a healthy cell only asserts it (its fast path), so the
/// restart bought nothing. A switch that changed the forwarder already restarted it, and its
/// ExecStartPost re-applies the /muas setup; `start` still revives an apply that failed.
fn ensure_script(units: &[&String]) -> String {
    let list = shell_list(units);
    format!(
        "for u in {list}; do sudo -n systemctl start \"$u\" || {{ echo \"start $u failed\" >&2; exit 1; }}; done; {}",
        wait_active_script(&list)
    )
}

/// Restart each unit in order unless it is already fresh, then wait for each to report active.
///
/// Fresh = active and entered activation at or after both the node's activation
/// (`/run/current-system` is re-linked by switch-to-configuration before it starts changed
/// units) and the last start of any running forwarder (a forwarder restart drops the unit's app
/// face and every prefix it registered). The check runs on the node right before each unit, after
/// any job queued on it has finished, so restarts cascaded by an earlier unit in this or a
/// previous group count: restarting muas-v2-controller restarts gcs and dashboard via PartOf=.
/// Prints `reference <t>`, then `fresh <unit> <t>` or `restarted <unit>` per unit.
fn refresh_script(units: &[&String]) -> String {
    let list = shell_list(units);
    format!(
        "ref=$(stat -c %Y /run/current-system) || exit 1; \
         for f in {FORWARDER_UNITS}; do systemctl is-active -q \"$f\" || continue; \
           t=$(systemctl show --timestamp=unix -P InactiveExitTimestamp \"$f\"); t=${{t#@}}; \
           [ -n \"$t\" ] && [ \"$t\" -gt \"$ref\" ] && ref=$t; done; \
         echo \"reference $ref\"; \
         for u in {list}; do i=0; while [ -n \"$(systemctl show -P Job \"$u\")\" ]; do \
             i=$((i+1)); [ $i -ge {UNIT_ACTIVE_S} ] && break; sleep 1; done; \
           t=$(systemctl show --timestamp=unix -P InactiveExitTimestamp \"$u\"); t=${{t#@}}; \
           if systemctl is-active -q \"$u\" && [ -n \"$t\" ] && [ \"$t\" -ge \"$ref\" ]; then \
             echo \"fresh $u $t\"; continue; fi; \
           sudo -n systemctl restart \"$u\" || {{ echo \"restart $u failed\" >&2; exit 1; }}; \
           echo \"restarted $u\"; done; {}",
        wait_active_script(&list)
    )
}

fn shell_list(units: &[&String]) -> String {
    units
        .iter()
        .map(|u| sh_quote(u))
        .collect::<Vec<_>>()
        .join(" ")
}

fn wait_active_script(list: &str) -> String {
    let tries = UNIT_ACTIVE_S / 3;
    format!(
        "for u in {list}; do i=0; until systemctl is-active -q \"$u\"; do i=$((i+1)); \
           if [ $i -ge {tries} ]; then echo \"$u not active: $(systemctl is-active \"$u\")\" >&2; exit 1; fi; \
           sleep 3; done; echo \"$u active\"; done"
    )
}

/// (restarted, already fresh) unit names from `refresh_script` output.
fn parse_refresh(stdout: &str) -> (Vec<String>, Vec<String>) {
    let mut restarted = Vec::new();
    let mut fresh = Vec::new();
    for line in stdout.lines() {
        let mut words = line.split_whitespace();
        match (words.next(), words.next()) {
            (Some("restarted"), Some(u)) => restarted.push(u.to_string()),
            (Some("fresh"), Some(u)) => fresh.push(u.to_string()),
            _ => {}
        }
    }
    (restarted, fresh)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn muas_rows_match_only_the_exact_prefix_in_both_tools_formats() {
        let ndnctl_strategy = "Prefix            Strategy\n\
            ──────────────────────────────────────────────────\n\
            /                 /localhost/nfd/strategy/best-route/v=5\n\
            /muas             /localhost/nfd/strategy/multicast/v=5\n\
            /muas/v2/iuas-01  /localhost/nfd/strategy/best-route/v=5\n";
        assert_eq!(
            muas_strategy(ndnctl_strategy).as_deref(),
            Some("/localhost/nfd/strategy/multicast/v=5")
        );
        let ndnctl_routes = "/muas                                                 4     100\n\
            /muas                                                 2     100\n\
            /muas                                                 3     100\n\
            /muas                                                 3       0\n\
            /muas/v2/wuas-01                                      3      10\n\
            /muas/v2/controller                                 257       0\n";
        assert_eq!(muas_nexthops(ndnctl_routes), 3);

        let nfdc_strategy = "prefix=/ strategy=/localhost/nfd/strategy/best-route/v=5\n\
            prefix=/muas/v2 strategy=/localhost/nfd/strategy/best-route/v=5\n\
            prefix=/muas strategy=/localhost/nfd/strategy/multicast/v=5\n";
        assert_eq!(
            muas_strategy(nfdc_strategy).as_deref(),
            Some("/localhost/nfd/strategy/multicast/v=5")
        );
        let nfdc_routes = "prefix=/muas nexthop=262 origin=static cost=0 flags=child-inherit expires=never\n\
            prefix=/muas nexthop=262 origin=app cost=0 flags=child-inherit expires=never\n\
            prefix=/muas nexthop=263 origin=static cost=100 flags=child-inherit expires=never\n\
            prefix=/muas/v2 nexthop=264 origin=app cost=0 flags=child-inherit expires=never\n";
        assert_eq!(
            muas_nexthops(nfdc_routes),
            2,
            "one face listed per origin counts once"
        );
    }

    #[test]
    fn cells_map_to_their_forwarder_unit_and_reject_garbage() {
        assert_eq!(
            forwarder_unit("ndn-fwd wifi").unwrap(),
            "muas-fabric-ndn-fwd"
        );
        assert_eq!(
            forwarder_unit("ndn-fwd radio").unwrap(),
            "muas-fabric-ndn-fwd-radio"
        );
        assert_eq!(forwarder_unit("nfd radio").unwrap(), "nfd");
        assert!(parse_cell("ndn-fwd").is_err());
        assert!(parse_cell("nfd wifi; reboot").is_err());
    }
}
