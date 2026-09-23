//! Face and content-store counters, parsed from the forwarder's own CLI on every node, and the
//! before/after delta a measurement reports (I7: "counters are deltas" — lifetime totals mixed
//! pre-window traffic into every earlier comparison).

use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};

use crate::config::{Config, Node};
use crate::remote;
use crate::state::now_ms;
use crate::status::join_all;

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Face {
    pub id: u64,
    /// Kind/scope/persistency words (`UDP permanent non-local point-to-point`).
    pub header: String,
    pub remote: Option<String>,
    pub local: Option<String>,
    /// `section.key` → number; bytes normalised to bytes, percentages as the percent number,
    /// durations in the unit the forwarder printed.
    pub values: BTreeMap<String, f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct NodeCounters {
    pub node: String,
    /// `ndn-fwd` | `nfd`
    pub forwarder: String,
    pub taken_unix_ms: u64,
    pub faces: Vec<Face>,
    /// `entries`, `hits`, `misses`, `capacity`, … (NFD's `nHits`-style keys normalised).
    pub cs: BTreeMap<String, f64>,
    pub raw: String,
}

/// Parse `ndn-ctl face list` (ndn-fwd): a `faceid=N <header words>` line, then indented
/// `section: k=v …` lines.
pub fn parse_ndnctl_faces(text: &str) -> Vec<Face> {
    let mut faces = Vec::new();
    let mut cur: Option<Face> = None;
    for line in text.lines() {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("faceid=") {
            faces.extend(cur.take());
            let mut words = rest.split_whitespace();
            let Some(id) = words.next().and_then(|w| w.parse().ok()) else {
                continue;
            };
            cur = Some(Face {
                id,
                header: words.collect::<Vec<_>>().join(" "),
                remote: None,
                local: None,
                values: BTreeMap::new(),
            });
            continue;
        }
        let (Some(face), Some((section, rest))) = (cur.as_mut(), t.split_once(':')) else {
            continue;
        };
        match section {
            "remote" => face.remote = Some(rest.trim().to_string()),
            "local" => face.local = Some(rest.trim().to_string()),
            _ => {
                for (k, v) in kv_numbers(rest) {
                    face.values.insert(format!("{section}.{k}"), v);
                }
            }
        }
    }
    faces.extend(cur);
    faces
}

/// Parse `nfdc face list` (NFD): one line per face,
/// `faceid=N remote=… local=… congestion={…} mtu=… counters={in={Ni Nd Nn NB} out={…}} flags={…}`.
/// Tolerant: unknown keys become header words (non-numeric) or values (numeric).
pub fn parse_nfdc_faces(text: &str) -> Vec<Face> {
    let mut faces = Vec::new();
    for line in text.lines() {
        let t = line.trim();
        if !t.starts_with("faceid=") {
            continue;
        }
        let mut face = Face {
            id: 0,
            header: String::new(),
            remote: None,
            local: None,
            values: BTreeMap::new(),
        };
        let mut header: Vec<String> = Vec::new();
        let mut flags = String::new();
        for tok in brace_tokens(t) {
            let Some((k, v)) = tok.split_once('=') else {
                header.push(tok.to_string());
                continue;
            };
            match (k, v.strip_prefix('{').and_then(|v| v.strip_suffix('}'))) {
                ("faceid", _) => face.id = v.parse().unwrap_or(0),
                ("remote", _) => face.remote = Some(v.to_string()),
                ("local", _) => face.local = Some(v.to_string()),
                ("flags", Some(inner)) => {
                    flags = inner.split_whitespace().collect::<Vec<_>>().join(" ")
                }
                ("counters", Some(inner)) => {
                    for dir in brace_tokens(inner) {
                        if let Some((d, body)) = dir.split_once('=') {
                            nfdc_counter_group(d, body, &mut face.values);
                        }
                    }
                }
                ("in" | "out", Some(_)) => nfdc_counter_group(k, v, &mut face.values),
                (_, Some(inner)) => {
                    for (ik, iv) in kv_numbers(inner) {
                        face.values.insert(format!("{k}.{ik}"), iv);
                    }
                }
                (_, None) => match quantity(v, None) {
                    Some((n, _)) => {
                        face.values.insert(k.to_string(), n);
                    }
                    None => header.push(tok.to_string()),
                },
            }
        }
        if !flags.is_empty() {
            header.push(flags);
        }
        face.header = header.join(" ");
        faces.push(face);
    }
    faces
}

/// `in={25i 3d 0n 2345B}` → `in.interests`, `in.data`, `in.nacks`, `in.bytes`.
fn nfdc_counter_group(dir: &str, body: &str, values: &mut BTreeMap<String, f64>) {
    let body = body.trim_start_matches('{').trim_end_matches('}');
    for item in body.split_whitespace() {
        let split = item
            .find(|c: char| !c.is_ascii_digit())
            .unwrap_or(item.len());
        let (num, suffix) = item.split_at(split);
        let Ok(n) = num.parse::<f64>() else { continue };
        let name = match suffix {
            "i" => "interests",
            "d" => "data",
            "n" => "nacks",
            "B" | "b" => "bytes",
            _ => continue,
        };
        values.insert(format!("{dir}.{name}"), n);
    }
}

/// Split on whitespace outside `{…}`.
fn brace_tokens(s: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let (mut depth, mut start) = (0usize, None);
    for (i, c) in s.char_indices() {
        match c {
            '{' => depth += 1,
            '}' => depth = depth.saturating_sub(1),
            c if c.is_whitespace() && depth == 0 => {
                if let Some(st) = start.take() {
                    out.push(&s[st..i]);
                }
                continue;
            }
            _ => {}
        }
        start.get_or_insert(i);
    }
    out.extend(start.map(|st| &s[st..]));
    out
}

/// Every numeric `k=v` in `text` (e.g. `ndn-ctl cs info`, `nfdc cs info`); non-numeric values
/// (`variant=lru`) are skipped.
pub fn parse_kv(text: &str) -> BTreeMap<String, f64> {
    let mut out = BTreeMap::new();
    for line in text.lines() {
        out.extend(kv_numbers(line));
    }
    out
}

/// Numeric `k=v` pairs of one line, applying a unit that ndn-ctl prints as a separate word
/// (`bytes=489.4 MiB`).
fn kv_numbers(line: &str) -> Vec<(String, f64)> {
    let words: Vec<&str> = line.split_whitespace().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < words.len() {
        if let Some((k, v)) = words[i].split_once('=')
            && let Some((n, used_next)) = quantity(v, words.get(i + 1).copied())
        {
            out.push((k.to_string(), n));
            i += usize::from(used_next);
        }
        i += 1;
    }
    out
}

/// A number with an optional unit: byte units scale to bytes (attached or as the next word),
/// anything else (`µs`, `ms`, `%`) keeps the printed number. `None` if not numeric.
fn quantity(v: &str, next: Option<&str>) -> Option<(f64, bool)> {
    let split = v
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-'))
        .unwrap_or(v.len());
    let (num, suffix) = v.split_at(split);
    let n: f64 = num.parse().ok()?;
    if suffix.is_empty()
        && let Some(scale) = next.and_then(byte_scale)
    {
        return Some(((n * scale).round(), true));
    }
    Some((byte_scale(suffix).map_or(n, |s| (n * s).round()), false))
}

fn byte_scale(unit: &str) -> Option<f64> {
    Some(match unit {
        "B" => 1.0,
        "KiB" => 1024.0,
        "MiB" => 1024.0 * 1024.0,
        "GiB" => 1024.0 * 1024.0 * 1024.0,
        "TiB" => 1024.0 * 1024.0 * 1024.0 * 1024.0,
        "kB" | "KB" => 1e3,
        "MB" => 1e6,
        "GB" => 1e9,
        _ => return None,
    })
}

const SNAPSHOT_SH: &str = r#"a=$(cat /var/lib/minimuas/fabric/active 2>/dev/null)
echo "@active $a"
case "$a" in
  nfd*) echo "@faces"; nfdc face list || exit 1; echo "@cs"; nfdc cs info || exit 1 ;;
  *) echo "@faces"; ndn-ctl face list || exit 1; echo "@cs"; ndn-ctl cs info || exit 1 ;;
esac"#;

/// Counters of one node, read with the CLI of the forwarder its active cell runs.
pub async fn snapshot(cfg: &Config, node: &Node) -> Result<NodeCounters> {
    let t0 = now_ms();
    let out = remote::ssh(cfg, node, SNAPSHOT_SH, Duration::from_secs(30)).await?;
    let t1 = now_ms();
    let text = out
        .stdout_ok()
        .with_context(|| format!("counters on {}", node.name))?;
    let section = |tag: &str| {
        text.split_once(tag)
            .map(|(_, rest)| rest.split("\n@").next().unwrap_or(""))
            .unwrap_or("")
    };
    let active = section("@active ").lines().next().unwrap_or("").trim();
    let nfd = active.starts_with("nfd");
    let faces = if nfd {
        parse_nfdc_faces(section("@faces"))
    } else {
        parse_ndnctl_faces(section("@faces"))
    };
    if faces.is_empty() {
        bail!(
            "{}: no faces parsed from the forwarder's face list",
            node.name
        );
    }
    let cs = parse_kv(section("@cs"))
        .into_iter()
        .map(|(k, v)| {
            let k = match k.as_str() {
                "nEntries" => "entries".to_string(),
                "nHits" => "hits".to_string(),
                "nMisses" => "misses".to_string(),
                _ => k,
            };
            (k, v)
        })
        .collect();
    Ok(NodeCounters {
        node: node.name.clone(),
        forwarder: if nfd { "nfd" } else { "ndn-fwd" }.into(),
        taken_unix_ms: t0 + (t1 - t0) / 2,
        faces,
        cs,
        raw: out.stdout,
    })
}

/// Every node in parallel; fails if any node cannot be read (a delta missing a node would
/// silently under-report the fabric).
pub async fn snapshot_fleet(cfg: &Config) -> Result<Vec<NodeCounters>> {
    let results = join_all(cfg.nodes.iter().map(|n| snapshot(cfg, n))).await;
    let mut out = Vec::with_capacity(results.len());
    let mut errors = Vec::new();
    for r in results {
        match r {
            Ok(c) => out.push(c),
            Err(e) => errors.push(format!("{e:#}")),
        }
    }
    if !errors.is_empty() {
        bail!("counter snapshot failed: {}", errors.join("; "));
    }
    Ok(out)
}

/// Point-in-time values, reported as the after value rather than differenced.
fn is_gauge(key: &str) -> bool {
    matches!(
        key.rsplit('.').next().unwrap_or(key),
        "rto"
            | "base-interval"
            | "base-marking-interval"
            | "threshold"
            | "default-threshold"
            | "complete"
            | "mtu"
            | "capacity"
            | "used"
            | "entries"
    )
}

/// Stable identity of a face across snapshots. Faceids change whenever the forwarder restarts,
/// so network faces are keyed by remote URI. Internal/app faces all share `internal://…`
/// remotes, so they (and any other duplicated remote) keep the faceid as a discriminator.
fn face_keys(faces: &[Face]) -> Vec<String> {
    let mut count: HashMap<&str, usize> = HashMap::new();
    for f in faces {
        *count.entry(f.remote.as_deref().unwrap_or("")).or_default() += 1;
    }
    faces
        .iter()
        .map(|f| match f.remote.as_deref() {
            Some(r) if !r.starts_with("internal://") && count[r] == 1 => r.to_string(),
            r => format!("{}#{}", r.unwrap_or("?"), f.id),
        })
        .collect()
}

/// Differences counters (`after − before`) and keeps gauges (after). `reset` when the counters
/// restarted inside the window (a counter went down): the after values are then the traffic
/// since the restart, a lower bound for the window.
fn diff_values(
    before: Option<&BTreeMap<String, f64>>,
    after: &BTreeMap<String, f64>,
) -> (Map<String, Value>, Map<String, Value>, bool) {
    let mut counters = Map::new();
    let mut gauges = Map::new();
    let reset = before.is_some_and(|b| {
        after
            .iter()
            .any(|(k, v)| !is_gauge(k) && b.get(k).is_some_and(|bv| v < bv))
    });
    for (k, v) in after {
        if is_gauge(k) {
            gauges.insert(k.clone(), json!(v));
        } else {
            let base = if reset {
                0.0
            } else {
                before.and_then(|b| b.get(k)).copied().unwrap_or(0.0)
            };
            counters.insert(k.clone(), json!(v - base));
        }
    }
    (counters, gauges, reset)
}

/// Per node, per face (keyed by remote URI): counter deltas over the window between two fleet
/// snapshots, faces new in the window, faces gone, CS deltas and the window's CS hit rate.
pub fn delta(before: &[NodeCounters], after: &[NodeCounters]) -> Value {
    let mut nodes = Map::new();
    let mut missing = Vec::new();
    for a in after {
        let Some(b) = before.iter().find(|b| b.node == a.node) else {
            missing.push(format!("{}: no before snapshot", a.node));
            continue;
        };
        let (bkeys, akeys) = (face_keys(&b.faces), face_keys(&a.faces));
        let bmap: HashMap<&str, &Face> = bkeys.iter().map(String::as_str).zip(&b.faces).collect();
        let mut faces = Map::new();
        let mut totals: BTreeMap<String, f64> = BTreeMap::new();
        for (key, af) in akeys.iter().zip(&a.faces) {
            let bf = bmap.get(key.as_str()).copied();
            // A new faceid for the same remote is a new face object: its counters began at 0.
            let recreated = bf.is_some_and(|bf| bf.id != af.id);
            let base = bf.filter(|_| !recreated).map(|bf| &bf.values);
            let (counters, gauges, reset) = diff_values(base, &af.values);
            if af.header.split_whitespace().any(|w| w == "non-local") {
                for (k, v) in &counters {
                    *totals.entry(k.clone()).or_default() += v.as_f64().unwrap_or(0.0);
                }
            }
            faces.insert(
                key.clone(),
                json!({
                    "faceid": af.id,
                    "faceid_before": bf.map(|f| f.id),
                    "header": af.header,
                    "new": bf.is_none(),
                    "reset": reset || recreated,
                    "counters": counters,
                    "gauges": gauges,
                }),
            );
        }
        let gone: Vec<&String> = bkeys.iter().filter(|k| !akeys.contains(k)).collect();
        let (cs, cs_gauges, cs_reset) = diff_values(Some(&b.cs), &a.cs);
        let get = |k: &str| cs.get(k).and_then(Value::as_f64);
        let hit_rate = match (get("hits"), get("misses")) {
            (Some(h), Some(m)) if h + m > 0.0 => json!(h / (h + m)),
            _ => Value::Null,
        };
        nodes.insert(
            a.node.clone(),
            json!({
                "forwarder": a.forwarder,
                "forwarder_changed": (a.forwarder != b.forwarder).then(|| format!("{} -> {}", b.forwarder, a.forwarder)),
                "window_ms": a.taken_unix_ms.saturating_sub(b.taken_unix_ms),
                "faces": faces,
                "gone": gone,
                "non_local_totals": totals,
                "cs": cs,
                "cs_gauges": cs_gauges,
                "cs_reset": cs_reset,
                "cs_hit_rate": hit_rate,
            }),
        );
    }
    for b in before {
        if !after.iter().any(|a| a.node == b.node) {
            missing.push(format!("{}: no after snapshot", b.node));
        }
    }
    json!({ "nodes": nodes, "missing": missing })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim `ndn-ctl face list` from minidronesys-03 (ndn-fwd wifi, 2026-09-23).
    const NDNCTL: &str = "\
faceid=6  ?  on-demand  local  point-to-point
  remote: internal://internal
  in:  interests=0  data=0  nacks=0  bytes=0 B
  out: interests=0  data=0  nacks=0  bytes=0 B

faceid=257  ?  on-demand  local  point-to-point
  remote: internal://management
  local:  unix:///run/nfd/nfd.sock
  in:  interests=8  data=76  nacks=0  bytes=109.1 KiB
  out: interests=76  data=8  nacks=0  bytes=47 B

faceid=4  UDP  permanent  non-local  point-to-point
  remote: udp4://192.168.1.14:6363
  local:  udp4://192.168.1.13:6363
  flags:  lp-reliability
  in:  interests=354214  data=293857  nacks=0  bytes=489.4 MiB
  out: interests=388672  data=931  nacks=112  bytes=36.6 MiB
  congestion: base-interval=100000µs  threshold=65536  marks-sent=0  marks-received=0
  reliability: rto=200000µs  resent=5550  fast-retx=2443  dup-rx=25932  acks-tx=890401  acks-rx=393492  gave-up=1519  evicted=0
  reassembly: fragments-in=271772  completed=55374  timed-out=1159  fragments-wasted=4302  rejected=0  evicted-groups=0  complete=97.9%
  features: fragmentation reassembly local-fields incoming-face-id nack trace-context reliability congestion-marking a-lal
";

    #[test]
    fn ndnctl_faces_parse_with_units_normalised() {
        let faces = parse_ndnctl_faces(NDNCTL);
        assert_eq!(faces.len(), 3);
        let internal = &faces[0];
        assert_eq!(internal.id, 6);
        assert_eq!(internal.remote.as_deref(), Some("internal://internal"));
        assert_eq!(internal.local, None);
        assert_eq!(internal.values["in.bytes"], 0.0);

        let mgmt = &faces[1];
        assert_eq!(mgmt.values["in.bytes"], (109.1f64 * 1024.0).round());
        assert_eq!(mgmt.values["out.bytes"], 47.0);

        let udp = &faces[2];
        assert_eq!(udp.id, 4);
        assert_eq!(udp.header, "UDP permanent non-local point-to-point");
        assert_eq!(udp.remote.as_deref(), Some("udp4://192.168.1.14:6363"));
        assert_eq!(udp.local.as_deref(), Some("udp4://192.168.1.13:6363"));
        assert_eq!(udp.values["in.interests"], 354_214.0);
        assert_eq!(udp.values["out.nacks"], 112.0);
        assert_eq!(udp.values["out.bytes"], (36.6f64 * 1048576.0).round());
        assert_eq!(udp.values["reliability.dup-rx"], 25_932.0);
        assert_eq!(udp.values["reliability.rto"], 200_000.0);
        assert_eq!(udp.values["reassembly.complete"], 97.9);
        assert_eq!(udp.values["congestion.threshold"], 65_536.0);
        // `flags:`/`features:` are words, not numbers; nothing leaks in from them.
        assert!(!udp.values.keys().any(|k| k.starts_with("features")));
    }

    #[test]
    fn byte_units_scale_attached_or_separate() {
        let kv = parse_kv("a=1.5 GiB b=2KiB c=3 MiB d=10B e=7");
        assert_eq!(kv["a"], 1.5 * 1073741824.0);
        assert_eq!(kv["b"], 2048.0);
        assert_eq!(kv["c"], 3.0 * 1048576.0);
        assert_eq!(kv["d"], 10.0);
        assert_eq!(kv["e"], 7.0);
    }

    #[test]
    fn ndnctl_cs_info_parses() {
        let cs = parse_kv(
            "✓ 200 capacity=67108864B entries=89013 used=67108477B hits=18209 misses=1113620 variant=lru",
        );
        assert_eq!(cs["capacity"], 67_108_864.0);
        assert_eq!(cs["hits"], 18_209.0);
        assert_eq!(cs["misses"], 1_113_620.0);
        assert!(!cs.contains_key("variant"));
    }

    #[test]
    fn nfdc_faces_parse() {
        let text = "\
faceid=1 remote=internal:// local=internal:// congestion={base-marking-interval=100ms default-threshold=65536B} mtu=8800 counters={in={0i 0d 0n 0B} out={3i 1d 0n 120B}} flags={local permanent point-to-point}
faceid=262 remote=udp4://192.168.1.14:6363 local=udp4://192.168.1.13:6363 congestion={base-marking-interval=100ms default-threshold=65536B} mtu=8800 counters={in={354214i 293857d 0n 513173094B} out={388672i 931d 112n 38377882B}} flags={non-local permanent point-to-point congestion-marking}
";
        let faces = parse_nfdc_faces(text);
        assert_eq!(faces.len(), 2);
        let udp = &faces[1];
        assert_eq!(udp.id, 262);
        assert_eq!(udp.remote.as_deref(), Some("udp4://192.168.1.14:6363"));
        assert_eq!(udp.values["in.interests"], 354_214.0);
        assert_eq!(udp.values["in.data"], 293_857.0);
        assert_eq!(udp.values["out.nacks"], 112.0);
        assert_eq!(udp.values["out.bytes"], 38_377_882.0);
        assert_eq!(udp.values["congestion.default-threshold"], 65_536.0);
        assert_eq!(udp.values["mtu"], 8800.0);
        assert!(udp.header.contains("non-local"));
        assert_eq!(faces[0].values["out.bytes"], 120.0);
    }

    fn snap(node: &str, t: u64, faces: Vec<Face>, cs: &[(&str, f64)]) -> NodeCounters {
        NodeCounters {
            node: node.into(),
            forwarder: "ndn-fwd".into(),
            taken_unix_ms: t,
            faces,
            cs: cs.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            raw: String::new(),
        }
    }

    fn face(id: u64, remote: &str, vals: &[(&str, f64)]) -> Face {
        Face {
            id,
            header: "UDP permanent non-local point-to-point".into(),
            remote: Some(remote.into()),
            local: None,
            values: vals.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
        }
    }

    #[test]
    fn delta_matches_faces_by_remote_across_faceid_changes() {
        let x = "udp4://192.168.1.14:6363";
        let y = "udp4://192.168.1.11:6363";
        let before = vec![snap(
            "n",
            1_000,
            vec![
                face(4, x, &[("in.interests", 100.0), ("reliability.rto", 200.0)]),
                face(2, y, &[("in.interests", 1_000.0)]),
            ],
            &[("hits", 10.0), ("misses", 90.0), ("entries", 5.0)],
        )];
        let after = vec![snap(
            "n",
            61_000,
            vec![
                // Same face, same id: plain difference.
                face(4, x, &[("in.interests", 150.0), ("reliability.rto", 300.0)]),
                // Forwarder restarted this link: new faceid, counters from zero.
                face(9, y, &[("in.interests", 30.0)]),
                face(11, "udp4://192.168.1.12:6363", &[("in.interests", 7.0)]),
            ],
            &[("hits", 40.0), ("misses", 100.0), ("entries", 8.0)],
        )];
        let d = delta(&before, &after);
        let n = &d["nodes"]["n"];
        assert_eq!(n["window_ms"], 60_000);
        assert_eq!(n["faces"][x]["counters"]["in.interests"], 50.0);
        assert_eq!(n["faces"][x]["gauges"]["reliability.rto"], 300.0);
        assert_eq!(n["faces"][x]["reset"], false);
        assert_eq!(n["faces"][y]["faceid_before"], 2);
        assert_eq!(n["faces"][y]["counters"]["in.interests"], 30.0);
        assert_eq!(n["faces"][y]["reset"], true);
        assert_eq!(n["faces"]["udp4://192.168.1.12:6363"]["new"], true);
        assert_eq!(n["non_local_totals"]["in.interests"], 50.0 + 30.0 + 7.0);
        assert_eq!(n["cs"]["hits"], 30.0);
        assert_eq!(n["cs_gauges"]["entries"], 8.0);
        assert_eq!(n["cs_hit_rate"], 30.0 / 40.0);
    }

    #[test]
    fn internal_faces_sharing_a_remote_do_not_collide() {
        let faces = parse_ndnctl_faces(NDNCTL);
        let mut dup = faces.clone();
        dup.push(Face {
            id: 300,
            ..faces[1].clone()
        });
        let keys = face_keys(&dup);
        assert_eq!(keys.len(), 4);
        assert!(keys.contains(&"internal://management#257".to_string()));
        assert!(keys.contains(&"internal://management#300".to_string()));
        assert!(keys.contains(&"udp4://192.168.1.14:6363".to_string()));
    }
}
