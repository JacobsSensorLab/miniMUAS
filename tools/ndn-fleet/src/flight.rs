//! `flight collect`: pull what the nodes recorded during a field day into
//! `<results>/flights/<id>/<node>/` for `analysis/flight_report.py`.
//!
//! Read-only on the nodes: the only remote commands are `find` (listing) and rsync's sender.
//! It therefore takes neither the fleet lock nor the armed check (I1/I2 guard mutations) and
//! does not stamp the settle clock; it may run while vehicles fly, and is ledgered like every
//! tool call (I8).
//!
//! What is pulled (flight-metrics contract A/B):
//! - every node: the recorder (`/var/lib/minimuas/recorder/`: cells.log, link sampler, dumpcap
//!   ring, forwarding snapshots, chrony; on the GCS also gps.log and ping/) and the role
//!   journals (`fleet.journal_glob`);
//! - every node: the recorder's RAM staging (`/run/muas-recorder/`, into `recorder-live/`).
//!   The recorder writes the current link file (up to 10 min) and dumpcap's current ring file
//!   (up to 120 s / 32 MB) there and copies each to the SD only once it is closed (config
//!   911b74f: live SD appends froze the GCS for seconds). Collected before the node is cleanly
//!   stopped, the end of the last sortie is only there. flight_report.py prefers the SD copy of
//!   any file present in both. `recorder/.incoming/` (copies in progress) is skipped;
//! - the GCS: the dashboard recordings (`/var/lib/minimuas/replays/dash-*.jsonl`).
//!
//! The GCS's replays held 16 GB on 2026-10-12, so `since` limits a collection to files written
//! since a time (mtime); long-lived files such as cells.log and ping.jsonl always qualify. A
//! second collection into the same `id` fetches only new or grown files.

use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use serde_json::{Value, json};

use crate::config::{Config, Node};
use crate::jobs::JobCtx;
use crate::remote;
use crate::state::{Recorder, iso8601, now_ms};
use crate::status;

/// Job kind of a collection. Read-only, so a dead one leaves nothing on the fleet to repair
/// (`measure::recover_abandoned` must not stamp the settle clock for it).
pub const JOB_KIND: &str = "collect";

const RECORDER_DIR: &str = "/var/lib/minimuas/recorder";
const RECORDER_STAGE_DIR: &str = "/run/muas-recorder";
const REPLAYS_DIR: &str = "/var/lib/minimuas/replays";
const LIST: Duration = Duration::from_secs(120);
/// One rsync of one root; a vehicle's recorder may hold up to its 10 GB cap.
const PULL: Duration = Duration::from_secs(4 * 3600);
/// Keep this much free under the results directory after the pull.
const MIN_FREE_BYTES: u64 = 1 << 30;

/// A remote directory pulled into `<flight>/<node>/<name>/`.
struct Root {
    name: &'static str,
    dir: String,
    /// `find -name` pattern; `None` = the whole tree.
    pattern: Option<String>,
    /// Subdirectory (relative) not to descend into.
    skip: Option<&'static str>,
}

fn roots(cfg: &Config, node: &Node) -> Result<Vec<Root>> {
    let glob = Path::new(&cfg.fleet.journal_glob);
    let (Some(log_dir), Some(pattern)) = (glob.parent(), glob.file_name()) else {
        bail!(
            "fleet.journal_glob '{}' is not <dir>/<pattern>",
            cfg.fleet.journal_glob
        );
    };
    let mut r = vec![
        Root {
            name: "recorder",
            dir: RECORDER_DIR.into(),
            pattern: None,
            // The recorder's in-progress SD copies, renamed into link/ or wire/ when complete.
            skip: Some(".incoming"),
        },
        Root {
            name: "recorder-live",
            dir: RECORDER_STAGE_DIR.into(),
            pattern: None,
            skip: None,
        },
        Root {
            name: "log",
            dir: log_dir.to_string_lossy().into_owned(),
            pattern: Some(pattern.to_string_lossy().into_owned()),
            skip: None,
        },
    ];
    if node.is_gcs() {
        r.push(Root {
            name: "replays",
            dir: REPLAYS_DIR.into(),
            pattern: None,
            skip: None,
        });
    }
    Ok(r)
}

/// One listed remote file.
#[derive(Debug, PartialEq)]
struct Remote {
    root: String,
    rel: String,
    size: u64,
    mtime_s: f64,
}

/// One `find` per root, tab-separated `<root> <size> <mtime> <relpath>`; a missing root prints
/// `missing <root>` (a node deployed before the recorder existed, a vehicle without replays).
fn list_script(roots: &[Root]) -> String {
    let mut s = String::new();
    for r in roots {
        let depth = if r.pattern.is_some() {
            " -maxdepth 1"
        } else {
            ""
        };
        let name = r
            .pattern
            .as_ref()
            .map(|p| format!(" -name {}", remote::sh_quote(p)))
            .unwrap_or_default();
        let dir = remote::sh_quote(&r.dir);
        let skip = r
            .skip
            .map(|s| {
                format!(
                    " -path {} -prune -o",
                    remote::sh_quote(&format!("{}/{s}", r.dir))
                )
            })
            .unwrap_or_default();
        // A file the recorder rotates or the disk cap deletes mid-walk makes find exit 1; that
        // is a warning in the manifest, not a failed node.
        s += &format!(
            "if [ -d {dir} ]; then sudo -n find {dir}{depth}{skip} -type f{name} \
             -printf '{n}\\t%s\\t%T@\\t%P\\n' || echo \"warn\t{n}\tfind exit $?\"; \
             else printf 'missing\\t{n}\\n'; fi\n",
            n = r.name
        );
    }
    s
}

#[derive(Default)]
struct Listing {
    files: Vec<Remote>,
    missing: Vec<String>,
    warnings: Vec<String>,
}

fn parse_listing(text: &str) -> Listing {
    let mut l = Listing::default();
    for line in text.lines() {
        let f: Vec<&str> = line.splitn(4, '\t').collect();
        match f.as_slice() {
            ["missing", root] => l.missing.push(root.to_string()),
            ["warn", root, what] => l.warnings.push(format!("{root}: {what}")),
            [root, size, mtime, rel] if !rel.is_empty() => {
                if let (Ok(size), Ok(mtime_s)) = (size.parse(), mtime.parse()) {
                    l.files.push(Remote {
                        root: root.to_string(),
                        rel: rel.to_string(),
                        size,
                        mtime_s,
                    });
                }
            }
            _ => {}
        }
    }
    l
}

/// `since` as unix seconds: an integer, or a UTC time `YYYY-MM-DD[(T| )HH:MM[:SS]][Z]`.
pub fn parse_since(s: &str) -> Result<u64> {
    let s = s.trim();
    if !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()) {
        return s.parse().context("since: unix seconds out of range");
    }
    let bad = || format!("since '{s}': expected unix seconds or YYYY-MM-DD[THH:MM[:SS]][Z] (UTC)");
    let t = s.strip_suffix('Z').unwrap_or(s);
    let (date, time) = match t.split_once(['T', ' ']) {
        Some((d, t)) => (d, Some(t)),
        None => (t, None),
    };
    let num = |p: Option<&str>, lo: u32, hi: u32| -> Result<u32> {
        let v: u32 = p.with_context(bad)?.parse().with_context(bad)?;
        if !(lo..=hi).contains(&v) {
            bail!(bad());
        }
        Ok(v)
    };
    let mut d = date.split('-');
    let y = num(d.next(), 1970, 9999)?;
    let mo = num(d.next(), 1, 12)?;
    let day = num(d.next(), 1, 31)?;
    if d.next().is_some() {
        bail!(bad());
    }
    let (h, mi, sec) = match time {
        None => (0, 0, 0),
        Some(t) => {
            let mut p = t.split(':');
            let h = num(p.next(), 0, 23)?;
            let mi = num(p.next(), 0, 59)?;
            let sec = match p.next() {
                Some(x) => num(Some(x), 0, 60)?,
                None => 0,
            };
            if p.next().is_some() {
                bail!(bad());
            }
            (h, mi, sec)
        }
    };
    let days = days_from_civil(i64::from(y), mo, day);
    Ok((days * 86_400) as u64 + u64::from(h * 3600 + mi * 60 + sec))
}

/// Howard Hinnant's days_from_civil (the inverse of `state::utc_parts`).
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = i64::from((m + 9) % 12);
    let doy = (153 * mp + 2) / 5 + i64::from(d) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// Files, bytes and mtime span of everything already under `dir` (what the flight holds,
/// whichever collection fetched it).
fn local_totals(dir: &Path) -> Value {
    fn walk(dir: &Path, acc: &mut (u64, u64, f64, f64)) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let Ok(md) = e.metadata() else { continue };
            if md.is_dir() {
                walk(&e.path(), acc);
            } else if md.is_file() {
                let mtime = md
                    .modified()
                    .ok()
                    .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                    .map_or(0.0, |d| d.as_secs_f64());
                acc.0 += 1;
                acc.1 += md.len();
                acc.2 = acc.2.min(mtime);
                acc.3 = acc.3.max(mtime);
            }
        }
    }
    let mut acc = (0, 0, f64::INFINITY, 0.0);
    walk(dir, &mut acc);
    let span = |t: f64| t.is_finite().then(|| iso8601((t * 1000.0) as u64));
    json!({
        "files": acc.0,
        "bytes": acc.1,
        "mtime_first": if acc.0 > 0 { span(acc.2) } else { None },
        "mtime_last": if acc.0 > 0 { span(acc.3) } else { None },
    })
}

/// `cells.log` lines (`<unix_ns> <cell>`) as `[[unix_s, cell], ...]`: the stack history the
/// report's periods come from, in the manifest for a glance.
fn cells_history(path: &Path) -> Value {
    let Ok(text) = std::fs::read_to_string(path) else {
        return Value::Null;
    };
    Value::Array(
        text.lines()
            .filter_map(|l| {
                let (ns, cell) = l.split_once(' ')?;
                let ns: u64 = ns.parse().ok()?;
                Some(json!([iso8601(ns / 1_000_000), cell.trim()]))
            })
            .collect(),
    )
}

async fn local_free_bytes(dir: &Path) -> Result<u64> {
    let d = dir.to_string_lossy().to_string();
    let out = remote::local("df", &["-Pk", &d], None, &[], LIST).await?;
    let text = out.stdout_ok()?;
    text.lines()
        .nth(1)
        .and_then(|l| l.split_whitespace().nth(3))
        .and_then(|k| k.parse::<u64>().ok())
        .map(|k| k * 1024)
        .with_context(|| format!("parsing `df -Pk {d}`: {text}"))
}

/// Bytes a pull of `files` would add under `node_dir` (absent files, plus growth of present ones).
fn bytes_needed(files: &[&Remote], node_dir: &Path) -> u64 {
    files
        .iter()
        .map(|f| {
            let have =
                std::fs::metadata(node_dir.join(&f.root).join(&f.rel)).map_or(0, |m| m.len());
            f.size.saturating_sub(have)
        })
        .sum()
}

/// Pull one node. Errors are per node: a node that cannot be reached leaves the others' data
/// intact and the flight resumable with `id`.
async fn collect_node(
    cfg: &Config,
    job: &JobCtx,
    node: &Node,
    flight_dir: &Path,
    since: Option<u64>,
) -> Result<Value> {
    let roots = roots(cfg, node)?;
    let listed = remote::ssh(cfg, node, &list_script(&roots), LIST).await?;
    let listing = parse_listing(
        listed
            .stdout_ok()
            .with_context(|| format!("{}: listing", node.name))?,
    );
    let wanted: Vec<&Remote> = listing
        .files
        .iter()
        .filter(|f| since.is_none_or(|s| f.mtime_s >= s as f64))
        .collect();
    let node_dir = flight_dir.join(&node.name);
    let need = bytes_needed(&wanted, &node_dir);
    let total: u64 = wanted.iter().map(|f| f.size).sum();
    job.log(format!(
        "{}: {} files listed, {} selected ({} MB), {} MB new; missing roots {:?}",
        node.name,
        listing.files.len(),
        wanted.len(),
        total >> 20,
        need >> 20,
        listing.missing
    ));
    std::fs::create_dir_all(&node_dir)
        .with_context(|| format!("creating {}", node_dir.display()))?;
    let free = local_free_bytes(&node_dir).await?;
    if free < need + MIN_FREE_BYTES {
        bail!(
            "{}: {} MB to fetch but only {} MB free under {} (keeps {} MB spare)",
            node.name,
            need >> 20,
            free >> 20,
            node_dir.display(),
            MIN_FREE_BYTES >> 20
        );
    }

    let mut per_root = serde_json::Map::new();
    let mut errors = Vec::new();
    for root in &roots {
        let files: Vec<&&Remote> = wanted.iter().filter(|f| f.root == root.name).collect();
        let dest = node_dir.join(root.name);
        let mut entry = json!({
            "remote": root.dir,
            "pattern": root.pattern,
            "missing": listing.missing.iter().any(|m| m == root.name),
            "selected_files": files.len(),
            "selected_bytes": files.iter().map(|f| f.size).sum::<u64>(),
        });
        if !files.is_empty() {
            std::fs::create_dir_all(&dest)?;
            let list_path = node_dir.join(format!(".{}.files", root.name));
            let list: String = files.iter().map(|f| format!("{}\n", f.rel)).collect();
            std::fs::write(&list_path, list)
                .with_context(|| format!("writing {}", list_path.display()))?;
            let out = remote::rsync_pull(cfg, node, &root.dir, &list_path, &dest, PULL).await;
            std::fs::remove_file(&list_path).ok();
            let out = out?;
            let fetched = out.stdout.lines().filter(|l| !l.ends_with('/')).count();
            entry["fetched_files"] = json!(fetched);
            entry["rsync_ms"] = json!(out.elapsed_ms);
            match out.status {
                Some(0) => {}
                // 24: a file vanished between listing and transfer (recorder disk cap,
                // rotation); the rest arrived.
                Some(24) => entry["warning"] = json!(out.stderr.trim()),
                _ => {
                    let e = format!(
                        "{}: rsync failed: {}",
                        root.name,
                        out.stdout_ok().unwrap_err()
                    );
                    errors.push(e.clone());
                    entry["error"] = json!(e);
                }
            }
            job.log(format!(
                "{}: {} fetched {fetched} of {} files in {} ms",
                node.name,
                root.name,
                files.len(),
                out.elapsed_ms
            ));
        }
        entry["local"] = local_totals(&dest);
        per_root.insert(root.name.into(), entry);
    }
    Ok(json!({
        "ok": errors.is_empty(),
        "errors": errors,
        "warnings": listing.warnings,
        "roots": per_root,
        "cells": cells_history(&node_dir.join("recorder/cells.log")),
    }))
}

/// Pull every node into `flights/<id>/` (a new id unless `resume` names an existing flight)
/// and write its `manifest.json`. Unreachable nodes are reported, not fatal: re-run with the
/// flight id once they are back.
pub async fn collect(
    cfg: &Config,
    rec: &Recorder,
    job: &JobCtx,
    resume: Option<String>,
    since: Option<u64>,
) -> Result<Value> {
    let id = resume.unwrap_or_else(|| rec.new_id("flight"));
    let rel = format!("flights/{id}");
    let flight_dir: PathBuf = rec.dir().join(&rel);
    std::fs::create_dir_all(&flight_dir)
        .with_context(|| format!("creating {}", flight_dir.display()))?;
    job.log(format!(
        "flight {id}: collecting from {} nodes{}",
        cfg.nodes.len(),
        since.map_or(String::new(), |s| format!(
            ", files written since {}",
            iso8601(s * 1000)
        ))
    ));
    let started = now_ms();
    let outs = status::join_all(
        cfg.nodes
            .iter()
            .map(|n| collect_node(cfg, job, n, &flight_dir, since)),
    )
    .await;

    let previous: Value = rec
        .read_json(&format!("{rel}/manifest.json"))
        .unwrap_or_else(|_| json!({}));
    let mut nodes = serde_json::Map::new();
    let mut failed = Vec::new();
    for (node, out) in cfg.nodes.iter().zip(outs) {
        let mut v = out
            .unwrap_or_else(|e| json!({ "ok": false, "errors": [format!("{e:#}")], "roots": {} }));
        if v["ok"] != json!(true) {
            failed.push(node.name.clone());
            job.log(format!("{}: INCOMPLETE {}", node.name, v["errors"]));
        }
        v["host"] = json!(node.host);
        v["vehicle"] = json!(node.vehicle);
        v["role"] = json!(node.role);
        nodes.insert(node.name.clone(), v);
    }
    let this = json!({
        "started": iso8601(started),
        "finished": iso8601(now_ms()),
        "since": since.map(|s| iso8601(s * 1000)),
        "job": job.id,
        "incomplete_nodes": failed,
    });
    let mut collections = previous["collections"]
        .as_array()
        .cloned()
        .unwrap_or_default();
    collections.push(this);
    let manifest = json!({
        "id": id,
        "layout": "<node>/{recorder,log,replays}/ mirror the node's /var/lib/minimuas/{recorder,log,replays}/",
        "complete": failed.is_empty(),
        "nodes": nodes,
        "collections": collections,
    });
    rec.write_json(&format!("{rel}/manifest.json"), &manifest)?;
    // Per node: bytes the flight now holds, so the ledger alone shows what each collection got.
    let held: serde_json::Map<String, Value> = nodes
        .iter()
        .map(|(n, v)| {
            let bytes: u64 = v["roots"]
                .as_object()
                .map(|r| {
                    r.values()
                        .filter_map(|e| e["local"]["bytes"].as_u64())
                        .sum()
                })
                .unwrap_or(0);
            (n.clone(), json!(bytes))
        })
        .collect();
    rec.ledger(
        "flight_collected",
        json!({ "id": id, "complete": failed.is_empty(), "incomplete_nodes": failed,
                "bytes_held": held }),
    );
    Ok(json!({
        "flight_id": id,
        "dir": rel,
        "complete": failed.is_empty(),
        "incomplete_nodes": failed,
        "next": format!("python3 analysis/flight_report.py {}", flight_dir.display()),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn since_parses_utc_and_round_trips_through_the_formatter() {
        assert_eq!(parse_since("1970-01-01").unwrap(), 0);
        assert_eq!(parse_since("1789486160").unwrap(), 1_789_486_160);
        // Leap day, year boundary, and the field day.
        for unix_s in [
            951_782_400u64,
            1_709_164_799,
            1_767_225_600,
            1_791_849_600 + 3_723,
        ] {
            let iso = iso8601(unix_s * 1000);
            assert_eq!(parse_since(&iso[..19]).unwrap(), unix_s, "{iso}");
            assert_eq!(
                parse_since(&format!("{}Z", &iso[..16])).unwrap(),
                unix_s - unix_s % 60
            );
        }
        for bad in [
            "2026-13-01",
            "2026-10-13T25:00",
            "yesterday",
            "2026-10",
            "2026-10-13T10",
        ] {
            assert!(parse_since(bad).is_err(), "{bad} accepted");
        }
    }

    #[test]
    fn listing_keeps_files_and_reports_missing_roots() {
        let text = "recorder\t12\t1791849600.5\tcells.log\n\
                    recorder\t9\t1791849601.0\twire/wire.pcapng_00001_20261013T000000Z\n\
                    missing\treplays\n\
                    warn\trecorder\tfind exit 1\n\
                    log\t5\t1791849602.25\tiuas-01-agent.jsonl\n";
        let l = parse_listing(text);
        assert_eq!(l.files.len(), 3);
        assert_eq!(
            l.files[1],
            Remote {
                root: "recorder".into(),
                rel: "wire/wire.pcapng_00001_20261013T000000Z".into(),
                size: 9,
                mtime_s: 1_791_849_601.0
            }
        );
        assert_eq!(l.missing, ["replays"]);
        assert_eq!(l.warnings.len(), 1);
    }

    /// The listing runs the generated `find` (minus sudo) over a recorder-shaped tree: the
    /// recorder's in-progress copies must not be listed (they are renamed into link/ or wire/
    /// when complete, so a collect would fetch half a file under a name the report never
    /// reads), the journal pattern must not descend, and an absent root is reported.
    #[tokio::test]
    async fn the_listing_skips_incoming_copies_and_reports_absent_roots() {
        let base = std::env::temp_dir().join(format!("ndn-fleet-flight-{}", std::process::id()));
        let rec = base.join("recorder");
        for d in ["link", "wire", ".incoming"] {
            std::fs::create_dir_all(rec.join(d)).unwrap();
        }
        std::fs::create_dir_all(base.join("log/old")).unwrap();
        for f in [
            "recorder/cells.log",
            "recorder/link/link-20261013T120000Z.txt.gz",
            "recorder/wire/wire_00001_20261013120000.pcapng",
            "recorder/.incoming/wire_00002_20261013120200.pcapng",
            "log/iuas-01-agent.jsonl",
            "log/notes.txt",
            "log/old/iuas-01-agent.jsonl",
        ] {
            std::fs::write(base.join(f), b"x").unwrap();
        }
        let roots = [
            Root {
                name: "recorder",
                dir: rec.to_string_lossy().into_owned(),
                pattern: None,
                skip: Some(".incoming"),
            },
            Root {
                name: "log",
                dir: base.join("log").to_string_lossy().into_owned(),
                pattern: Some("*.jsonl".into()),
                skip: None,
            },
            Root {
                name: "replays",
                dir: base.join("replays").to_string_lossy().into_owned(),
                pattern: None,
                skip: None,
            },
        ];
        let script = list_script(&roots).replace("sudo -n ", "");
        let out = remote::local("sh", &["-c", &script], None, &[], LIST)
            .await
            .unwrap();
        std::fs::remove_dir_all(&base).ok();
        let l = parse_listing(out.stdout_ok().unwrap());
        let mut got: Vec<String> = l
            .files
            .iter()
            .map(|f| format!("{}/{}", f.root, f.rel))
            .collect();
        got.sort();
        assert_eq!(
            got,
            [
                "log/iuas-01-agent.jsonl",
                "recorder/cells.log",
                "recorder/link/link-20261013T120000Z.txt.gz",
                "recorder/wire/wire_00001_20261013120000.pcapng",
            ]
        );
        assert_eq!(l.missing, ["replays"]);
        assert!(l.warnings.is_empty(), "{:?}", l.warnings);
    }
}
