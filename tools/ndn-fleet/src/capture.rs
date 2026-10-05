//! Per-sample evidence capture (PROTOCOL.md I10): packet headers on the fabric interface, a
//! periodic link/PHY/host sampler, and journal slices, on every node, for the exact sample
//! window; plus optional trace levels for NFD and the role services.
//!
//! Trace levels need a restart to take effect (NFD reads its log section at start, NDNSF and
//! the agents read their environment at start). They are installed as RUNTIME drop-ins under
//! `/run/systemd/system` before the cell switch into the capture arm, so that switch -- which
//! starts the forwarder and restarts every role unit anyway -- is the only restart, and are
//! removed before the I9 restore switch, which restarts them clean. A reboot also clears them.
//!
//! Collectors run on the node as a transient systemd unit, not inside an ssh session: the
//! sample must not depend on the control link (minidronesys-02 is reachable only over the Wi-Fi
//! being measured), and nothing is pulled until the sample is over.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::cells::out_json;
use crate::config::{Config, Node};
use crate::jobs::JobCtx;
use crate::remote::{self, sh_quote};
use crate::state::Recorder;
use crate::status;

const QUICK: Duration = Duration::from_secs(60);
/// Journal slicing, compression and tarring on a node after a sample.
const PACK: Duration = Duration::from_secs(300);
/// Pulling one node's bundle; 02's rides the Wi-Fi via the jump host.
const PULL: Duration = Duration::from_secs(600);
/// Drop-in file name, the same for every unit, so removal needs no record of what was installed.
const DROP_IN: &str = "50-ndn-fleet-capture.conf";
/// Runtime state: the generated nfd.conf and the install stamp.
const RUN_DIR: &str = "/run/ndn-fleet-capture";
/// Per-sample collector output on the node.
const NODE_DIR: &str = "/var/tmp/ndn-fleet-capture";
const UNIT: &str = "ndn-fleet-capture";
/// Seconds of journal kept either side of the capture window.
const JOURNAL_PAD_S: u64 = 2;
/// Refuse to start a capture sample with less than this free under the results directory.
const MIN_FREE_BYTES: u64 = 1 << 30;

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Capture {
    /// Interface the fabric's UDP faces ride.
    #[serde(default = "default_iface")]
    pub iface: String,
    /// Bytes kept per packet by dumpcap (IP/UDP/NDNLP headers and the start of the NDN packet,
    /// i.e. the name of an unfragmented packet or first fragment). 0 = no packet capture.
    #[serde(default = "default_snaplen")]
    pub snaplen: u32,
    /// Link/PHY/host sampler period. 0 = no sampler.
    #[serde(default = "default_interval")]
    pub link_interval_ms: u64,
    /// NFD log levels by module as nfd.conf names them (`*` = `default_level`), in force while
    /// NFD runs during the capture job. Empty = NFD's own config.
    #[serde(default)]
    pub nfd_log: BTreeMap<String, String>,
    /// Environment set on every role unit (`[restart]` agents/control/dashboard) for the job.
    #[serde(default)]
    pub role_env: BTreeMap<String, String>,
}

fn default_iface() -> String {
    "mesh0".into()
}
fn default_snaplen() -> u32 {
    200
}
fn default_interval() -> u64 {
    1000
}

/// Drops nfd.conf's top-level `log { ... }` section (NFD's INFO format: `log` alone on a line,
/// the section closed by a `}` at column 0).
const NFD_LOG_STRIP_AWK: &str =
    "BEGIN{skip=0} /^log[[:space:]]*$/{skip=1; next} skip&&/^}/{skip=0; next} !skip";

const NFD_LEVELS: [&str; 7] = ["NONE", "ERROR", "WARN", "INFO", "DEBUG", "TRACE", "ALL"];

impl Capture {
    /// Whether the capture changes what NFD or the role units log (needs their restart).
    pub fn traces(&self) -> bool {
        !self.nfd_log.is_empty() || !self.role_env.is_empty()
    }

    pub fn validate(&self) -> Result<()> {
        let word = |s: &str, extra: &str| {
            !s.is_empty()
                && s.chars()
                    .all(|c| c.is_ascii_alphanumeric() || extra.contains(c))
        };
        if !word(&self.iface, "_.-") {
            bail!("capture.iface `{}` is not an interface name", self.iface);
        }
        if self.snaplen != 0 && !(64..=65535).contains(&self.snaplen) {
            bail!("capture.snaplen must be 0 or 64..=65535");
        }
        for (module, level) in &self.nfd_log {
            if module != "*" && !word(module, "_.") {
                bail!("capture.nfd_log: `{module}` is not an NFD log module");
            }
            if !NFD_LEVELS.contains(&level.as_str()) {
                bail!(
                    "capture.nfd_log.{module}: `{level}` is not one of {}",
                    NFD_LEVELS.join("|")
                );
            }
        }
        for (k, v) in &self.role_env {
            if !word(k, "_") || k.starts_with(|c: char| c.is_ascii_digit()) {
                bail!("capture.role_env: `{k}` is not an environment variable name");
            }
            // Written into a systemd Environment="K=V" line: '%' is a specifier, '"' and '\'
            // would need escaping, and a newline would end the directive.
            if v.chars()
                .any(|c| matches!(c, '%' | '"' | '\\' | '\n' | '\r'))
            {
                bail!("capture.role_env.{k}: value may not contain % \" \\ or newlines");
            }
        }
        Ok(())
    }
}

/// Role units the trace environment goes on.
fn role_units(cfg: &Config) -> Vec<String> {
    cfg.restart
        .agents
        .iter()
        .chain(&cfg.restart.control)
        .chain(&cfg.restart.dashboard)
        .cloned()
        .collect()
}

/// Run `script` as root on every node in parallel; every node must succeed.
async fn on_every_node(cfg: &Config, what: &str, script: &str, timeout: Duration) -> Result<Value> {
    let cmd = format!("sudo -n sh -c {}", sh_quote(script));
    let outs = status::join_all(cfg.nodes.iter().map(|n| remote::ssh(cfg, n, &cmd, timeout))).await;
    let mut per_node = serde_json::Map::new();
    let mut failures = Vec::new();
    for (node, out) in cfg.nodes.iter().zip(outs) {
        let out = out?;
        if let Err(e) = out.stdout_ok() {
            failures.push(format!("{}: {e:#}", node.name));
        }
        per_node.insert(node.name.clone(), out_json(&out));
    }
    if !failures.is_empty() {
        bail!("{what} failed: {}", failures.join("; "));
    }
    Ok(Value::Object(per_node))
}

fn install_script(cfg: &Config, cap: &Capture) -> String {
    let mut s = format!("set -eu\nmkdir -p {RUN_DIR}\n");
    if !cap.nfd_log.is_empty() {
        let mut block = String::from("log\n{\n");
        if let Some(level) = cap.nfd_log.get("*") {
            block.push_str(&format!("  default_level {level}\n"));
        }
        for (module, level) in cap.nfd_log.iter().filter(|(m, _)| *m != "*") {
            block.push_str(&format!("  {module} {level}\n"));
        }
        block.push_str("}\n");
        // Same binary and config NFD runs with, the `log` section replaced: everything else
        // (faces, strategies, CS) must be exactly what the cell normally runs.
        s.push_str(&format!(
            r#"argv=$(systemctl show -P ExecStart nfd.service | sed -n 's/.*argv\[\]=\([^;]*\) ;.*/\1/p' | head -n1)
bin=${{argv%% *}}
conf=$(printf '%s\n' "$argv" | sed -n 's/.* -c \([^ ]*\).*/\1/p')
if [ ! -x "$bin" ] || [ ! -r "$conf" ]; then echo "cannot resolve nfd ExecStart: $argv" >&2; exit 1; fi
awk {awk} "$conf" > {RUN_DIR}/nfd.conf
cat >> {RUN_DIR}/nfd.conf <<'NDNFLEETLOG'
{block}NDNFLEETLOG
mkdir -p /run/systemd/system/nfd.service.d
printf '[Service]\nExecStart=\nExecStart=%s -c %s\n' "$bin" {RUN_DIR}/nfd.conf > /run/systemd/system/nfd.service.d/{DROP_IN}
echo "nfd $bin -c {RUN_DIR}/nfd.conf (from $conf)"
"#,
            awk = sh_quote(NFD_LOG_STRIP_AWK)
        ));
    }
    if !cap.role_env.is_empty() {
        let lines: String = cap
            .role_env
            .iter()
            .map(|(k, v)| format!(" {}", sh_quote(&format!("Environment=\"{k}={v}\""))))
            .collect();
        for unit in role_units(cfg) {
            let u = sh_quote(&format!("{unit}.service"));
            s.push_str(&format!(
                "if systemctl cat {u} >/dev/null 2>&1; then \
                 mkdir -p /run/systemd/system/{u}.d; \
                 printf '%s\\n' '[Service]'{lines} > /run/systemd/system/{u}.d/{DROP_IN}; \
                 echo \"env {unit}\"; fi\n"
            ));
        }
    }
    s.push_str(&format!(
        "systemctl daemon-reload\ndate +%s > {RUN_DIR}/installed\n"
    ));
    s
}

fn remove_script(cfg: &Config) -> String {
    let units: Vec<String> = std::iter::once("nfd".to_string())
        .chain(role_units(cfg))
        .map(|u| format!("/run/systemd/system/{u}.service.d"))
        .collect();
    format!(
        "n=0; for d in {dirs}; do if [ -f \"$d/{DROP_IN}\" ]; then rm -f \"$d/{DROP_IN}\"; \
         rmdir \"$d\" 2>/dev/null || true; n=$((n+1)); fi; done; rm -rf {RUN_DIR}; \
         systemctl daemon-reload; echo \"removed $n drop-in(s)\"",
        dirs = units.join(" ")
    )
}

/// Install the trace drop-ins on every node. They take effect at the next start of each unit:
/// the caller's cell switch.
pub async fn install_trace(cfg: &Config, job: &JobCtx, cap: &Capture) -> Result<Value> {
    job.log(format!(
        "capture: installing trace drop-ins (nfd_log {:?}, role_env {:?})",
        cap.nfd_log, cap.role_env
    ));
    on_every_node(
        cfg,
        "installing capture trace drop-ins",
        &install_script(cfg, cap),
        QUICK,
    )
    .await
}

/// Remove every capture drop-in (idempotent). Running units keep the trace until they restart.
pub async fn remove_trace(cfg: &Config, job: &JobCtx) -> Result<Value> {
    let r = on_every_node(
        cfg,
        "removing capture trace drop-ins",
        &remove_script(cfg),
        QUICK,
    )
    .await?;
    job.log("capture: trace drop-ins removed");
    Ok(r)
}

/// After the switch into the capture cell: every traced unit that runs on a node must have
/// (re)started after the drop-ins were installed, or the capture would record untraced
/// processes while claiming the trace.
pub async fn verify_trace(cfg: &Config, cap: &Capture) -> Result<Value> {
    let mut units: Vec<String> = Vec::new();
    if !cap.nfd_log.is_empty() {
        units.push("nfd".into());
    }
    if !cap.role_env.is_empty() {
        units.extend(role_units(cfg));
    }
    let list = units
        .iter()
        .map(|u| sh_quote(u))
        .collect::<Vec<_>>()
        .join(" ");
    let script = format!(
        "i=$(cat {RUN_DIR}/installed) || {{ echo 'capture drop-ins not installed' >&2; exit 1; }}; \
         bad=0; for u in {list}; do systemctl cat \"$u.service\" >/dev/null 2>&1 || continue; \
         t=$(systemctl show --timestamp=unix -P ActiveEnterTimestamp \"$u\"); t=${{t#@}}; \
         s=$(systemctl is-active \"$u\"); \
         if [ \"$s\" = active ] && [ -n \"$t\" ] && [ \"$t\" -ge \"$i\" ]; then echo \"traced $u since $t\"; \
         else echo \"NOT traced: $u ($s since ${{t:-never}}, drop-ins at $i)\" >&2; bad=1; fi; done; exit $bad"
    );
    on_every_node(
        cfg,
        "verifying the capture trace is in force",
        &script,
        QUICK,
    )
    .await
}

/// A running capture: what `stop` and `collect` need.
pub struct Running {
    pub run_id: String,
    pub started_unix_s: u64,
    pub stopped_unix_s: Option<u64>,
}

fn collector_script(cap: &Capture, dir: &str) -> String {
    let iface = &cap.iface;
    let pcap = if cap.snaplen > 0 {
        format!(
            // -B 32 (MiB): with libpcap's 2 MiB default ring the kernel dropped 4-12% of the
            // packets (dumpcap's own `pcap:` drops, first dry run 2026-10-05), which would
            // read as wire loss. 32 MiB holds ~10 s of the GCS's ~7 kpkt/s at 200 B.
            "dumpcap -q -B 32 -i {iface} -s {snap} -f 'udp port 6363' -w \"$D/wire.pcapng\" 2>>\"$D/dumpcap.err\" &\n\
             echo $! > \"$D/dumpcap.pid\"\n",
            snap = cap.snaplen
        )
    } else {
        String::new()
    };
    let sampler = if cap.link_interval_ms > 0 {
        format!(
            r#"PIDS=$(for u in nfd muas-fabric-ndn-fwd muas-fabric-ndn-fwd-radio muas-v2-agent muas-v2-dashboard muas-v2-gcs muas-v2-controller; do systemctl show -P MainPID "$u" 2>/dev/null; done | grep -v '^0$' | tr '\n' ' ')
echo "$PIDS" > "$D/pids"
# One `cat` for every thread of the forwarder and role processes: a cat per thread (~100 on
# the GCS) took 0.77 s of every tick. The thread set is fixed for the sample.
TASKS=$(for p in $PIDS; do ls -d /proc/$p/task/*/stat 2>/dev/null; done | tr '\n' ' ')
DBG=$(ls -d /sys/kernel/debug/ieee80211/*/netdev:{iface} 2>/dev/null | head -n1)
PHY=${{DBG%/netdev:*}}
RTL=/proc/net/rtl88x2eu/{iface}
s() {{ echo "@ $1"; shift; "$@" 2>&1; }}
while :; do
  t0=$(date +%s%N)
  {{
    echo "@@ $t0"
    s station iw dev {iface} station dump
    s survey iw dev {iface} survey dump
    s netdev grep -H . /sys/class/net/{iface}/statistics/rx_packets /sys/class/net/{iface}/statistics/tx_packets /sys/class/net/{iface}/statistics/rx_dropped /sys/class/net/{iface}/statistics/tx_dropped /sys/class/net/{iface}/statistics/rx_errors /sys/class/net/{iface}/statistics/tx_errors
    s qdisc tc -s qdisc show dev {iface}
    s snmp grep -E '^(Ip|Udp):' /proc/net/snmp
    s udp grep -hi ':18EB ' /proc/net/udp /proc/net/udp6
    s softnet cat /proc/net/softnet_stat
    s cpu grep '^cpu' /proc/stat
    s tasks cat $TASKS
    if [ -d "$RTL" ]; then for f in trx_info rx_stat rx_signal sta_tp_info trx_info_debug; do [ -r "$RTL/$f" ] && s "rtl.$f" cat "$RTL/$f"; done; fi
    if [ -n "$PHY" ]; then
      [ -r "$PHY/aqm" ] && s mac80211.aqm cat "$PHY/aqm"
      [ -r "$PHY/mt76/xmit-queues" ] && s mt76.xmit-queues cat "$PHY/mt76/xmit-queues"
      for st in "$DBG"/stations/*; do [ -d "$st" ] || continue; m=${{st##*/}}
        [ -r "$st/aqm" ] && s "sta.$m.aqm" cat "$st/aqm"
        [ -r "$st/airtime" ] && s "sta.$m.airtime" cat "$st/airtime"
      done
    fi
    if systemctl is-active -q nfd; then s nfd.faces nfdc face list; s nfd.status nfdc status; fi
    if systemctl is-active -q muas-fabric-ndn-fwd; then s fwd.faces ndn-ctl face list; fi
  }} >> "$D/link.txt"
  # Paced to the period, not period + work: a tick costs ~0.3 s on the GCS.
  left=$(( {period_ms} - ($(date +%s%N) - t0) / 1000000 ))
  [ "$left" -gt 0 ] && sleep "$(awk -v ms="$left" 'BEGIN{{printf "%.3f", ms/1000}}')"
done
"#,
            period_ms = cap.link_interval_ms
        )
    } else {
        "while :; do sleep 1; done\n".into()
    };
    format!(
        // A transient unit gets systemd's minimal PATH, not the login one: dumpcap, iw, nfdc and
        // chronyc live in the NixOS system profile (the first dry run found no dumpcap).
        "PATH=/run/wrappers/bin:/run/current-system/sw/bin:$PATH\nexport PATH\n\
         D={dir}\ncd \"$D\" || exit 1\n\
         date +%s%N > started_ns\nchronyc -n tracking > chrony-start.txt 2>&1\n\
         stop() {{ [ -f dumpcap.pid ] && kill \"$(cat dumpcap.pid)\" 2>/dev/null; wait; \
         date +%s%N > stopped_ns; chronyc -n tracking > chrony-stop.txt 2>&1; exit 0; }}\n\
         trap stop TERM INT\n{pcap}{sampler}"
    )
}

/// Start the collectors on every node. Fails if any node cannot start (a capture missing a node
/// cannot say where a packet was lost).
pub async fn start(
    cfg: &Config,
    rec: &Recorder,
    job: &JobCtx,
    cap: &Capture,
    run_id: &str,
) -> Result<Running> {
    let free = local_free_bytes(rec.dir()).await?;
    if free < MIN_FREE_BYTES {
        bail!(
            "capture: only {} MB free under {}; refusing to capture (needs >= {} MB)",
            free >> 20,
            rec.dir().display(),
            MIN_FREE_BYTES >> 20
        );
    }
    let dir = format!("{NODE_DIR}/{run_id}");
    let script = collector_script(cap, &dir);
    let launch = format!(
        "set -eu; systemctl stop {UNIT} 2>/dev/null || true; systemctl reset-failed {UNIT} 2>/dev/null || true; \
         mkdir -p {dir}; cat > {dir}/collect.sh <<'NDNFLEETCOLLECT'\n{script}NDNFLEETCOLLECT\n\
         systemd-run --unit={UNIT} --collect --quiet --property=KillMode=control-group sh {dir}/collect.sh; \
         sleep 1; systemctl is-active -q {UNIT} || {{ journalctl -u {UNIT} -n 20 --no-pager >&2; exit 1; }}; \
         [ {snap} -eq 0 ] || kill -0 \"$(cat {dir}/dumpcap.pid 2>/dev/null)\" 2>/dev/null || \
           {{ echo 'dumpcap is not running' >&2; cat {dir}/dumpcap.err >&2; exit 1; }}; \
         echo started",
        snap = cap.snaplen
    );
    let started_unix_s = crate::state::now_ms() / 1000;
    let r = on_every_node(cfg, "starting capture collectors", &launch, QUICK).await;
    if let Err(e) = r {
        // Leave nothing running behind a failed start.
        let _ = on_every_node(
            cfg,
            "stopping capture collectors",
            &format!("systemctl stop {UNIT} 2>/dev/null || true; rm -rf {dir}"),
            QUICK,
        )
        .await;
        return Err(e);
    }
    job.log(format!("capture: collectors running on every node ({dir})"));
    Ok(Running {
        run_id: run_id.to_string(),
        started_unix_s,
        stopped_unix_s: None,
    })
}

/// Stop the collectors (dumpcap flushes on SIGTERM). Called right after the workload, before
/// anything else touches the fabric.
pub async fn stop(cfg: &Config, job: &JobCtx, run: &mut Running) -> Result<()> {
    let r = on_every_node(
        cfg,
        "stopping capture collectors",
        &format!(
            "systemctl stop {UNIT}; cat {NODE_DIR}/{}/stopped_ns",
            run.run_id
        ),
        QUICK,
    )
    .await;
    run.stopped_unix_s = Some(crate::state::now_ms() / 1000);
    job.log("capture: collectors stopped");
    r.map(|_| ())
}

/// Cut the journal slices, pack each node's capture, pull it into `<rel>/capture/<node>/`
/// (size and sha256 verified), then delete it from the node.
pub async fn collect(
    cfg: &Config,
    rec: &Recorder,
    job: &JobCtx,
    run: &Running,
    rel: &str,
) -> Result<Value> {
    let since = run.started_unix_s.saturating_sub(JOURNAL_PAD_S);
    let until = run
        .stopped_unix_s
        .unwrap_or_else(|| crate::state::now_ms() / 1000)
        + JOURNAL_PAD_S;
    let id = &run.run_id;
    let dir = format!("{NODE_DIR}/{id}");
    let pack = format!(
        // Only the journal files written since the collector started: a time slice over the
        // whole journal opened and merged every file -- 4 GB on minidronesys-02's SD card,
        // more than 10 minutes and past this timeout; the same slice over the recent files
        // took 0.18 s. Rotated archives are included (their mtime is their last write).
        "set -eu; cd {dir}; \
         F=$(find /var/log/journal /run/log/journal -name '*.journal' -newer started_ns 2>/dev/null | sed 's/^/--file=/' | tr '\\n' ' '); \
         journalctl $F -a -o short-unix --no-pager --since @{since} --until @{until} -u nfd -u 'muas-*' | gzip -1 > journal.txt.gz; \
         journalctl $F -k -a -o short-unix --no-pager --since @{since} --until @{until} | gzip -1 > kernel.txt.gz; \
         {{ uname -a; iw dev; for i in /sys/class/net/*; do d=$(readlink $i/device/driver 2>/dev/null) && echo \"driver ${{i##*/}} ${{d##*/}}\"; done; \
            cat /var/lib/minimuas/fabric/active; echo; readlink /run/current-system; }} > node.txt 2>&1; \
         for f in wire.pcapng link.txt; do [ -f $f ] && gzip -1 $f; done; \
         cd {NODE_DIR}; tar -cf {id}.tar {id}; \
         echo \"$(stat -c %s {id}.tar) $(sha256sum {id}.tar | cut -d' ' -f1)\""
    );
    let cmd = format!("sudo -n sh -c {}", sh_quote(&pack));
    let packed = status::join_all(cfg.nodes.iter().map(|n| remote::ssh(cfg, n, &cmd, PACK))).await;
    let local_dir = rec.dir().join(rel).join("capture");
    std::fs::create_dir_all(&local_dir)
        .with_context(|| format!("creating {}", local_dir.display()))?;

    let mut per_node = serde_json::Map::new();
    for (node, out) in cfg.nodes.iter().zip(packed) {
        let out = out?;
        let text = out
            .stdout_ok()
            .with_context(|| format!("{}: packing capture", node.name))?;
        let (size, sha) = parse_size_sha(text)
            .with_context(|| format!("{}: unexpected pack output `{}`", node.name, text.trim()))?;
        let entry = pull(cfg, job, node, id, size, &sha, &local_dir).await?;
        per_node.insert(node.name.clone(), entry);
    }
    // Only once every node's bundle is verified locally.
    on_every_node(
        cfg,
        "deleting pulled captures",
        &format!("rm -rf {dir} {dir}.tar"),
        QUICK,
    )
    .await?;
    Ok(json!({"dir": format!("{rel}/capture"), "window_unix_s": [since, until], "nodes": per_node}))
}

fn parse_size_sha(text: &str) -> Option<(u64, String)> {
    let mut w = text.lines().last()?.split_whitespace();
    let size = w.next()?.parse().ok()?;
    let sha = w.next()?.to_string();
    (sha.len() == 64).then_some((size, sha))
}

async fn pull(
    cfg: &Config,
    job: &JobCtx,
    node: &Node,
    id: &str,
    size: u64,
    sha: &str,
    local_dir: &Path,
) -> Result<Value> {
    let tar: PathBuf = local_dir.join(format!("{}.tar", node.name));
    let out =
        remote::ssh_to_file(cfg, node, &format!("cat {NODE_DIR}/{id}.tar"), &tar, PULL).await?;
    out.stdout_ok()
        .with_context(|| format!("{}: pulling capture", node.name))?;
    let got = std::fs::metadata(&tar)?.len();
    if got != size {
        bail!("{}: pulled {got} bytes, node packed {size}", node.name);
    }
    let tar_s = tar.to_string_lossy().to_string();
    let local_sha = remote::local("shasum", &["-a", "256", &tar_s], None, &[], QUICK).await?;
    let local_sha = local_sha
        .stdout_ok()?
        .split_whitespace()
        .next()
        .unwrap_or("")
        .to_string();
    if local_sha != sha {
        bail!(
            "{}: sha256 mismatch after pull ({local_sha} != {sha})",
            node.name
        );
    }
    let node_dir = local_dir.join(&node.name);
    std::fs::create_dir_all(&node_dir)?;
    let node_s = node_dir.to_string_lossy().to_string();
    // The bundle's top directory is the run id; strip it so the layout is capture/<node>/<file>.
    remote::local(
        "tar",
        &["-xf", &tar_s, "-C", &node_s, "--strip-components", "1"],
        None,
        &[],
        QUICK,
    )
    .await?
    .stdout_ok()
    .with_context(|| format!("{}: unpacking capture", node.name))?;
    std::fs::remove_file(&tar)?;
    let mut files = serde_json::Map::new();
    for e in std::fs::read_dir(&node_dir)? {
        let e = e?;
        files.insert(
            e.file_name().to_string_lossy().into(),
            json!(e.metadata()?.len()),
        );
    }
    job.log(format!(
        "capture: {} pulled ({} KB, sha256 ok)",
        node.name,
        size >> 10
    ));
    Ok(json!({"bytes": size, "sha256": sha, "files": files}))
}

async fn local_free_bytes(dir: &Path) -> Result<u64> {
    let d = dir.to_string_lossy().to_string();
    let out = remote::local("df", &["-k", &d], None, &[], QUICK).await?;
    let text = out.stdout_ok()?;
    text.lines()
        .nth(1)
        .and_then(|l| l.split_whitespace().nth(3))
        .and_then(|k| k.parse::<u64>().ok())
        .map(|k| k * 1024)
        .with_context(|| format!("parsing `df -k {d}`: {text}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cap() -> Capture {
        Capture {
            iface: "mesh0".into(),
            snaplen: 200,
            link_interval_ms: 1000,
            nfd_log: BTreeMap::from([
                ("*".into(), "INFO".into()),
                ("Forwarder".into(), "DEBUG".into()),
            ]),
            role_env: BTreeMap::from([("NDN_LOG".into(), "a=DEBUG:b=INFO".into())]),
        }
    }

    #[test]
    fn values_that_would_break_a_systemd_environment_line_are_refused() {
        assert!(cap().validate().is_ok());
        for bad in ["50%", "a\"b", "a\\b", "a\nb"] {
            let mut c = cap();
            c.role_env.insert("X".into(), bad.into());
            assert!(c.validate().is_err(), "{bad:?} accepted");
        }
        let mut c = cap();
        c.nfd_log.insert("Forwarder".into(), "VERBOSE".into());
        assert!(c.validate().is_err());
    }

    /// The generated nfd.conf must keep every other section untouched and lose the original
    /// log section, or NFD would run with two (or none of the cell's config).
    #[tokio::test]
    async fn the_log_section_is_dropped_and_nothing_else_is_touched() {
        let conf = "general\n{\n}\n\nlog\n{\n    default_level NONE\n    FaceManager DEBUG\n}\n\n\
                    tables\n{\n  cs_max_packets 65536\n}\n";
        let path = std::env::temp_dir().join(format!("ndn-fleet-awk-{}.conf", std::process::id()));
        std::fs::write(&path, conf).unwrap();
        let out = remote::local(
            "awk",
            &[NFD_LOG_STRIP_AWK, &path.to_string_lossy()],
            None,
            &[],
            QUICK,
        )
        .await
        .unwrap();
        std::fs::remove_file(&path).ok();
        let text = out.stdout_ok().unwrap();
        assert_eq!(
            text,
            "general\n{\n}\n\n\ntables\n{\n  cs_max_packets 65536\n}\n"
        );
    }
}
