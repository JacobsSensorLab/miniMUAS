//! The WebSocket broadcast hub + session-scoped mission recorder.
//!
//! Every JSON broadcast fans out to all connected clients and — while a
//! recording session is ARMED — lands in that session's JSONL via
//! uas-console's power-loss-safe [`Recorder`] (per-line flush, fsync ≤ 2 s).
//! Binary video frames fan out but are never recorded — exactly the v2
//! rule.
//!
//! Recording semantics (round 3): the recorder is session-scoped, not
//! process-scoped. It arms at mission start (or an explicit Record request)
//! and finalizes at mission end / RTL-all / explicit stop; an idle
//! dashboard produces NO recording. Sessions are named
//! `<run>-<mission>-<t>.jsonl` so the replay picker can group by run.
//!
//! Truth layering: the per-vehicle journal chains remain the durable record
//! of what happened — recordings are derived UI artifacts (what the
//! operator's screen received), useful for replaying the console, never a
//! second source of truth.
//!
//! Documented deviation from v2: recording lines are uas-console
//! `RecordedEvent`s (`{"t_ns": .., "event": ..}`) instead of v2's ad-hoc
//! `{"ts": .., "m": ..}`; the ported frontend loads both.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use serde_json::Value;
use tokio::sync::broadcast;
use uas_console::{RecordedEvent, Recorder};

/// One outbound WS message.
#[derive(Debug, Clone)]
pub enum Outbound {
    /// A JSON text message (already serialized).
    Text(Arc<String>),
    /// A binary message (`[vehicle index][jpeg]` video frames).
    Binary(Arc<Vec<u8>>),
}

struct RecorderState {
    /// `None` = recording disabled (no dir, or an earlier write failed).
    dir: Option<PathBuf>,
    /// The run label recordings group under (`<run>-<mission>-<t>.jsonl`).
    run: String,
    recorder: Option<Recorder>,
}

/// Broadcast hub shared by pollers, the command layer, and WS sessions.
pub struct Hub {
    tx: broadcast::Sender<Outbound>,
    rec: Mutex<RecorderState>,
}

impl Hub {
    /// A hub able to record into `record_dir` (`None` disables recording).
    /// `run_name` labels this dashboard session's recordings; empty picks
    /// `run-<start time>` so every process gets a distinct, groupable run.
    pub fn new(record_dir: Option<PathBuf>, run_name: &str) -> Self {
        let (tx, _) = broadcast::channel(512);
        let run = if run_name.is_empty() {
            format!("run-{}", timestamp())
        } else {
            sanitize(run_name)
        };
        Self {
            tx,
            rec: Mutex::new(RecorderState { dir: record_dir, run, recorder: None }),
        }
    }

    /// Subscribe a WS session to the broadcast stream.
    pub fn subscribe(&self) -> broadcast::Receiver<Outbound> {
        self.tx.subscribe()
    }

    /// Broadcast one JSON message to every client and the recorder.
    pub fn broadcast(&self, message: &Value) {
        self.record(message);
        let _ = self.tx.send(Outbound::Text(Arc::new(message.to_string())));
    }

    /// Broadcast one binary frame (never recorded — the v2 rule).
    pub fn broadcast_binary(&self, bytes: Vec<u8>) {
        let _ = self.tx.send(Outbound::Binary(Arc::new(bytes)));
    }

    /// Path of the live recording, if one is open.
    pub fn recording_path(&self) -> Option<PathBuf> {
        let state = self.lock();
        state.recorder.as_ref().map(|r| r.path().to_path_buf())
    }

    /// True while a recording session is armed.
    pub fn is_recording(&self) -> bool {
        self.lock().recorder.is_some()
    }

    /// Arm a recording session named `<run>-<mission>-<t>.jsonl`
    /// (`mission` is the mission id, or `"manual"` for the Record button).
    /// Idempotent: an already-armed session keeps recording (a mission
    /// started under a manual recording lands in that recording). Returns
    /// the session file name, `None` when recording is disabled or the
    /// directory is unwritable.
    pub fn arm(&self, mission: &str) -> Option<String> {
        let mut state = self.lock();
        if let Some(rec) = state.recorder.as_ref() {
            return rec.path().file_name().map(|n| n.to_string_lossy().into_owned());
        }
        let dir = state.dir.clone()?;
        let stem = format!("{}-{}-{}", state.run, sanitize(mission), timestamp());
        // Timestamps are second-granular: suffix on collision so two
        // sessions in one second never share a file.
        let mut name = format!("{stem}.jsonl");
        let mut i = 1;
        while dir.join(&name).exists() {
            i += 1;
            name = format!("{stem}-{i}.jsonl");
        }
        match std::fs::create_dir_all(&dir).and_then(|()| Recorder::create(dir.join(&name))) {
            Ok(rec) => {
                tracing::info!(path = %rec.path().display(), "dash.record.armed");
                state.recorder = Some(rec);
                Some(name)
            }
            Err(err) => {
                tracing::warn!(%err, "dash.record.disabled");
                state.dir = None;
                None
            }
        }
    }

    /// Finalize the armed recording session: fsync and close. Broadcasts
    /// after this land nowhere until the next [`arm`](Self::arm) — idle
    /// periods produce nothing. Returns the finalized file name.
    pub fn finalize(&self) -> Option<String> {
        let mut state = self.lock();
        let mut rec = state.recorder.take()?;
        let _ = rec.sync();
        let name = rec.path().file_name().map(|n| n.to_string_lossy().into_owned());
        tracing::info!(path = %rec.path().display(), "dash.record.finalized");
        name
    }

    /// Force an fsync of the live recording (pre-shutdown, and before an
    /// authorized companion shutdown — "the recording should hold this
    /// moment").
    pub fn sync(&self) {
        let mut state = self.lock();
        if let Some(rec) = state.recorder.as_mut() {
            let _ = rec.sync();
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, RecorderState> {
        self.rec.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn record(&self, message: &Value) {
        let mut state = self.lock();
        // Session-scoped: broadcasts land only while armed (no lazy open —
        // idle produces nothing). A write failure disables recording
        // without killing the process.
        if let Some(rec) = state.recorder.as_mut() {
            let event = RecordedEvent { t_ns: now_ns(), event: message.clone() };
            if let Err(err) = rec.record(&event) {
                tracing::warn!(%err, "dash.record.disabled");
                state.recorder = None;
                state.dir = None;
            }
        }
    }
}

/// Wall clock, nanoseconds since the Unix epoch.
pub fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// `YYYYMMDD-HHMMSS` (UTC) — the `<t>` part of a session name.
fn timestamp() -> String {
    let secs = (now_ns() / 1_000_000_000) as i64;
    let (y, mo, d, h, mi, s) = civil_utc(secs);
    format!("{y:04}{mo:02}{d:02}-{h:02}{mi:02}{s:02}")
}

/// Replay-route-safe label: `[A-Za-z0-9._-]`, everything else becomes `_`
/// (the `/replays/{name}` route validates the same alphabet).
fn sanitize(label: &str) -> String {
    let cleaned: String = label
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-' { c } else { '_' })
        .collect();
    if cleaned.is_empty() { "x".into() } else { cleaned }
}

/// Unix seconds → UTC civil date-time (Howard Hinnant's days algorithm).
fn civil_utc(secs: i64) -> (i64, u32, u32, u32, u32, u32) {
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    let (h, mi, s) = ((rem / 3600) as u32, ((rem % 3600) / 60) as u32, (rem % 60) as u32);
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d, h, mi, s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_conversion_matches_known_instants() {
        assert_eq!(civil_utc(0), (1970, 1, 1, 0, 0, 0));
        // `date -u -r 1783773296` → Sat Jul 11 12:34:56 UTC 2026
        assert_eq!(civil_utc(1_783_773_296), (2026, 7, 11, 12, 34, 56));
    }

    #[test]
    fn session_names_are_replay_route_safe_and_grouped_by_run() {
        let dir = std::env::temp_dir().join(format!("muas-hub-name-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let hub = Hub::new(Some(dir.clone()), "run 42/α");
        let name = hub.arm("mission-1783708295").expect("arms");
        // `<run>-<mission>-<t>.jsonl`, sanitized to the /replays alphabet.
        assert!(name.starts_with("run_42__-mission-1783708295-"), "name: {name}");
        assert!(name.ends_with(".jsonl"));
        assert!(name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-'));
        // Re-arming while armed keeps the session (idempotent).
        assert_eq!(hub.arm("manual").as_deref(), Some(name.as_str()));
        assert_eq!(hub.finalize().as_deref(), Some(name.as_str()));
        assert!(!hub.is_recording());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn idle_broadcasts_are_not_recorded() {
        let dir = std::env::temp_dir().join(format!("muas-hub-idle-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let hub = Hub::new(Some(dir.clone()), "");
        hub.broadcast(&serde_json::json!({ "type": "telemetry", "idle": true }));
        assert!(hub.recording_path().is_none(), "no lazy open");
        assert!(!dir.exists() || std::fs::read_dir(&dir).unwrap().next().is_none());
        // Arm → broadcasts land; finalize → they stop.
        hub.arm("manual").expect("arms");
        hub.broadcast(&serde_json::json!({ "type": "event", "kind": "x", "t": 1.0 }));
        let path = hub.recording_path().expect("recording open");
        hub.finalize();
        hub.broadcast(&serde_json::json!({ "type": "event", "kind": "after", "t": 2.0 }));
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("\"kind\":\"x\""));
        assert!(!text.contains("after"), "post-finalize broadcasts land nowhere");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
