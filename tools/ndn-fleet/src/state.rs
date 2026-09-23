//! The results directory and the fleet's persistent state: ledger (I8), settle clock (I3) and the
//! fleet lock (I1). Everything that must survive a server restart lives here, on disk.

use std::collections::HashMap;
use std::fs::{File, OpenOptions, TryLockError};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::config::Config;

/// Owner of `<results>`: every file the protocol writes goes through here, so layout, atomicity
/// and the ledger are uniform (PROTOCOL.md "Results layout").
pub struct Recorder {
    dir: PathBuf,
    /// Ids issued in the current UTC second, per prefix, so two ids never collide in-process.
    ids: Mutex<(String, HashMap<String, u32>)>,
    /// Serialises read-modify-write of `state/state.json`.
    state_mu: Mutex<()>,
    tmp_seq: AtomicU64,
}

#[derive(Serialize, Deserialize, Default, Clone, Debug)]
pub struct State {
    pub last_disturbance: Option<Disturbance>,
    pub last_deploy: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Disturbance {
    pub at_unix_ms: u64,
    pub kind: String,
    pub detail: String,
}

/// Contents of `state/lock` while held.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct LockInfo {
    pub holder: String,
    pub pid: u32,
    pub since_unix_ms: u64,
}

/// The fleet lock (I1). Backed by an exclusive `flock` on `state/lock`: the kernel drops it when
/// the holding process dies, so a crashed server can never wedge the fleet, and the holder record
/// left behind tells the next taker who died holding it.
pub struct FleetLock {
    file: File,
    holder: String,
    ledger: PathBuf,
}

impl Drop for FleetLock {
    fn drop(&mut self) {
        // Clear the record before the flock goes: an empty file means "released cleanly".
        let _ = self.file.set_len(0);
        let _ = self.file.unlock();
        append_ledger(
            &self.ledger,
            "lock_released",
            &json!({ "holder": self.holder }),
        );
    }
}

impl Recorder {
    pub fn open(cfg: &Config) -> Result<Arc<Recorder>> {
        Self::open_dir(&cfg.fleet.results_dir)
    }

    /// [`Recorder::open`] on an explicit directory (tests, alternative inventories).
    pub fn open_dir(dir: &Path) -> Result<Arc<Recorder>> {
        for sub in ["state", "deploys", "runs", "jobs"] {
            std::fs::create_dir_all(dir.join(sub))
                .with_context(|| format!("creating {}", dir.join(sub).display()))?;
        }
        Ok(Arc::new(Recorder {
            dir: dir.to_path_buf(),
            ids: Mutex::new((String::new(), HashMap::new())),
            state_mu: Mutex::new(()),
            tmp_seq: AtomicU64::new(0),
        }))
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// `20260923T141503Z-<prefix>`; a second id with the same prefix in the same second gets a
    /// `-2`, `-3`… suffix. Ids sort chronologically, which `results` relies on.
    pub fn new_id(&self, prefix: &str) -> String {
        let stamp = stamp_compact(now_ms());
        let mut ids = self.ids.lock().unwrap_or_else(|e| e.into_inner());
        if ids.0 != stamp {
            ids.0 = stamp.clone();
            ids.1.clear();
        }
        loop {
            let n = ids.1.entry(prefix.to_string()).or_insert(0);
            *n += 1;
            let id = if *n == 1 {
                format!("{stamp}-{prefix}")
            } else {
                format!("{stamp}-{prefix}-{n}")
            };
            // Another process (CLI beside the MCP server) may have issued it this second.
            if !self.id_taken(&id) {
                return id;
            }
        }
    }

    fn id_taken(&self, id: &str) -> bool {
        [
            format!("jobs/{id}.log"),
            format!("jobs/{id}.json"),
            format!("runs/{id}"),
            format!("deploys/{id}.json"),
            format!("deploys/{id}.plan.json"),
            format!("state/snapshots/{id}.json"),
        ]
        .iter()
        .any(|rel| self.dir.join(rel).exists())
    }

    /// Append `{ts, event, detail}` to `ledger.jsonl` (I8). Never fails the caller: a full disk
    /// must not abort a rollout half-way; the failure goes to stderr instead.
    pub fn ledger(&self, event: &str, detail: Value) {
        append_ledger(&self.dir.join("ledger.jsonl"), event, &detail);
    }

    /// Atomic write (tmp + rename) of pretty JSON; creates parent directories.
    pub fn write_json(&self, rel: &str, v: &impl Serialize) -> Result<()> {
        let mut bytes = serde_json::to_vec_pretty(v).context("serialising JSON")?;
        bytes.push(b'\n');
        self.write_bytes(rel, &bytes)
    }

    pub fn write_text(&self, rel: &str, s: &str) -> Result<()> {
        self.write_bytes(rel, s.as_bytes())
    }

    fn write_bytes(&self, rel: &str, bytes: &[u8]) -> Result<()> {
        let path = self.dir.join(rel);
        let parent = path.parent().unwrap_or(&self.dir);
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating {}", parent.display()))?;
        let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("out");
        let tmp = parent.join(format!(
            ".{name}.tmp{}.{}",
            std::process::id(),
            self.tmp_seq.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::write(&tmp, bytes).with_context(|| format!("writing {}", tmp.display()))?;
        std::fs::rename(&tmp, &path).with_context(|| format!("renaming into {}", path.display()))
    }

    pub fn read_json<T: serde::de::DeserializeOwned>(&self, rel: &str) -> Result<T> {
        let path = self.dir.join(rel);
        let bytes = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
        serde_json::from_slice(&bytes).with_context(|| format!("parsing {}", path.display()))
    }

    pub fn state(&self) -> State {
        match self.read_json::<State>("state/state.json") {
            Ok(s) => s,
            Err(_) if !self.dir.join("state/state.json").exists() => State::default(),
            // An unreadable state must not silently reset the settle clock (I3): treat it as a
            // disturbance happening now, so the next measurement waits a full settle.
            Err(e) => State {
                last_disturbance: Some(Disturbance {
                    at_unix_ms: now_ms(),
                    kind: "unknown".into(),
                    detail: format!("state/state.json unreadable: {e:#}"),
                }),
                last_deploy: None,
            },
        }
    }

    fn update_state(&self, f: impl FnOnce(&mut State)) {
        let _g = self.state_mu.lock().unwrap_or_else(|e| e.into_inner());
        let mut st = self.state();
        f(&mut st);
        if let Err(e) = self.write_json("state/state.json", &st) {
            eprintln!("ndn-fleet: cannot write state/state.json: {e:#}");
        }
    }

    /// Stamp the settle clock (I3) and ledger the disturbance.
    pub fn disturb(&self, kind: &str, detail: &str) {
        let at = now_ms();
        self.update_state(|st| {
            st.last_disturbance = Some(Disturbance {
                at_unix_ms: at,
                kind: kind.into(),
                detail: detail.into(),
            });
        });
        self.ledger("disturbance", json!({ "kind": kind, "detail": detail }));
    }

    pub fn set_last_deploy(&self, deploy_id: &str) {
        self.update_state(|st| st.last_deploy = Some(deploy_id.into()));
        self.ledger("last_deploy", json!({ "deploy_id": deploy_id }));
    }

    pub fn seconds_since_disturbance(&self) -> Option<u64> {
        self.state()
            .last_disturbance
            .map(|d| now_ms().saturating_sub(d.at_unix_ms) / 1000)
    }

    fn lock_path(&self) -> PathBuf {
        self.dir.join("state/lock")
    }

    /// Take the fleet lock (I1). Err names the live holder. A record left by a holder that died
    /// without releasing is taken over and ledgered.
    pub fn lock(&self, holder: &str) -> Result<FleetLock> {
        let path = self.lock_path();
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&path)
            .with_context(|| format!("opening {}", path.display()))?;
        let mut attempts = 0;
        loop {
            match file.try_lock() {
                Ok(()) => break,
                Err(TryLockError::WouldBlock) if attempts < 10 => {
                    // A `lock_holder` peek or a holder mid-acquire holds the flock for
                    // microseconds; don't refuse a mutation because of it.
                    attempts += 1;
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(TryLockError::WouldBlock) => match read_lock_record(&mut file) {
                    Some(info) => bail!(
                        "fleet is locked by {} (pid {}, since {}); one mutation at a time (I1)",
                        info.holder,
                        info.pid,
                        iso8601(info.since_unix_ms)
                    ),
                    None => bail!("fleet lock {} is held by an unknown holder", path.display()),
                },
                Err(TryLockError::Error(e)) => {
                    return Err(e).with_context(|| format!("locking {}", path.display()));
                }
            }
        }
        if let Some(prev) = read_lock_record(&mut file) {
            self.ledger(
                "lock_takeover",
                json!({ "holder": holder, "stale": prev,
                        "why": "previous holder exited without releasing" }),
            );
        }
        let info = LockInfo {
            holder: holder.into(),
            pid: std::process::id(),
            since_unix_ms: now_ms(),
        };
        file.set_len(0)?;
        file.seek(SeekFrom::Start(0))?;
        file.write_all(&serde_json::to_vec(&info)?)?;
        file.sync_data().ok();
        self.ledger("lock_taken", json!({ "holder": holder }));
        Ok(FleetLock {
            file,
            holder: holder.into(),
            ledger: self.dir.join("ledger.jsonl"),
        })
    }

    /// The live lock holder, if any (read-only peek; used to refuse a second mutation up front).
    pub fn lock_holder(&self) -> Option<LockInfo> {
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(self.lock_path())
            .ok()?;
        match file.try_lock() {
            Ok(()) => {
                let _ = file.unlock();
                None
            }
            Err(TryLockError::WouldBlock) => read_lock_record(&mut file).or(Some(LockInfo {
                holder: "unknown".into(),
                pid: 0,
                since_unix_ms: 0,
            })),
            Err(TryLockError::Error(_)) => None,
        }
    }
}

fn read_lock_record(file: &mut File) -> Option<LockInfo> {
    let mut s = String::new();
    file.seek(SeekFrom::Start(0)).ok()?;
    file.read_to_string(&mut s).ok()?;
    serde_json::from_str(&s).ok()
}

fn append_ledger(path: &Path, event: &str, detail: &Value) {
    let mut line = json!({ "ts": iso8601(now_ms()), "event": event, "detail": detail }).to_string();
    line.push('\n');
    // One write per line under O_APPEND keeps concurrent writers (CLI + server) from interleaving.
    let res = OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .and_then(|mut f| f.write_all(line.as_bytes()));
    if let Err(e) = res {
        eprintln!("ndn-fleet: ledger append to {} failed: {e}", path.display());
    }
}

pub fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as u64)
}

/// UTC civil time from a unix timestamp in ms: (year, month, day, hour, min, sec, ms).
fn utc_parts(unix_ms: u64) -> (i64, u32, u32, u32, u32, u32, u32) {
    let secs = (unix_ms / 1000) as i64;
    let (days, sod) = (secs.div_euclid(86_400), secs.rem_euclid(86_400) as u32);
    // Howard Hinnant's civil_from_days.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe as i64 + era * 400 + i64::from(m <= 2);
    (
        y,
        m,
        d,
        sod / 3600,
        sod / 60 % 60,
        sod % 60,
        (unix_ms % 1000) as u32,
    )
}

/// `20260923T141503Z`.
pub fn stamp_compact(unix_ms: u64) -> String {
    let (y, mo, d, h, mi, s, _) = utc_parts(unix_ms);
    format!("{y:04}{mo:02}{d:02}T{h:02}{mi:02}{s:02}Z")
}

/// `2026-09-23T14:15:03.123Z`.
pub fn iso8601(unix_ms: u64) -> String {
    let (y, mo, d, h, mi, s, ms) = utc_parts(unix_ms);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}.{ms:03}Z")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp_recorder(tag: &str) -> Arc<Recorder> {
        let dir = std::env::temp_dir().join(format!(
            "ndn-fleet-state-{tag}-{}-{}",
            std::process::id(),
            now_ms()
        ));
        Recorder::open_dir(&dir).unwrap()
    }

    #[test]
    fn utc_formatting_handles_leap_days_and_epoch() {
        assert_eq!(iso8601(0), "1970-01-01T00:00:00.000Z");
        // 2024-02-29T12:34:56.789Z
        assert_eq!(iso8601(1_709_210_096_789), "2024-02-29T12:34:56.789Z");
        assert_eq!(stamp_compact(1_709_210_096_789), "20240229T123456Z");
        // 2100 is not a leap year: 2100-03-01T00:00:00Z
        assert_eq!(iso8601(4_107_542_400_000), "2100-03-01T00:00:00.000Z");
    }

    #[test]
    fn ids_are_unique_within_a_second() {
        let rec = tmp_recorder("ids");
        let a = rec.new_id("measure");
        let b = rec.new_id("measure");
        assert_ne!(a, b);
        assert!(a.ends_with("Z-measure") || b.ends_with("-2"));
    }

    #[test]
    fn a_second_lock_is_refused_naming_the_holder_and_released_on_drop() {
        let rec = tmp_recorder("lock");
        let held = rec.lock("job-A").unwrap();
        let err = rec.lock("job-B").err().expect("second lock refused");
        assert!(err.to_string().contains("job-A"), "{err}");
        assert_eq!(
            rec.lock_holder().map(|i| i.holder).as_deref(),
            Some("job-A")
        );
        drop(held);
        assert!(rec.lock_holder().is_none());
        let _again = rec.lock("job-B").expect("free after release");
    }

    #[test]
    fn a_record_left_by_a_dead_holder_is_taken_over() {
        let rec = tmp_recorder("stale");
        let stale = LockInfo {
            holder: "crashed-job".into(),
            pid: 999_999,
            since_unix_ms: 1,
        };
        std::fs::write(
            rec.dir().join("state/lock"),
            serde_json::to_vec(&stale).unwrap(),
        )
        .unwrap();
        let _l = rec.lock("next").expect("stale record does not block");
        let ledger = std::fs::read_to_string(rec.dir().join("ledger.jsonl")).unwrap();
        assert!(ledger.contains("lock_takeover") && ledger.contains("crashed-job"));
    }
}
