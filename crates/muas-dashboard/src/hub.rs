//! The WebSocket broadcast hub + mission recorder.
//!
//! Every JSON broadcast fans out to all connected clients AND lands in the
//! session's JSONL recording via uas-console's power-loss-safe
//! [`Recorder`] (per-line flush, fsync ≤ 2 s). Binary video frames fan out
//! but are never recorded — exactly the v2 rule.
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
    recorder: Option<Recorder>,
}

/// Broadcast hub shared by pollers, the command layer, and WS sessions.
pub struct Hub {
    tx: broadcast::Sender<Outbound>,
    rec: Mutex<RecorderState>,
}

impl Hub {
    /// A hub recording into `record_dir` (`None` disables recording).
    pub fn new(record_dir: Option<PathBuf>) -> Self {
        let (tx, _) = broadcast::channel(512);
        Self {
            tx,
            rec: Mutex::new(RecorderState { dir: record_dir, recorder: None }),
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
        let Some(dir) = state.dir.clone() else { return };
        if state.recorder.is_none() {
            // Lazy open on first broadcast, like v2: one timestamped file
            // per dashboard session. An unwritable directory disables
            // recording without killing the process.
            match std::fs::create_dir_all(&dir)
                .and_then(|()| Recorder::create(dir.join(recording_name())))
            {
                Ok(rec) => {
                    tracing::info!(path = %rec.path().display(), "dash.record.started");
                    state.recorder = Some(rec);
                }
                Err(err) => {
                    tracing::warn!(%err, "dash.record.disabled");
                    state.dir = None;
                    return;
                }
            }
        }
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

/// `dash-YYYYMMDD-HHMMSS.jsonl` (UTC), matching the v2 naming.
fn recording_name() -> String {
    let secs = (now_ns() / 1_000_000_000) as i64;
    let (y, mo, d, h, mi, s) = civil_utc(secs);
    format!("dash-{y:04}{mo:02}{d:02}-{h:02}{mi:02}{s:02}.jsonl")
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
    fn recording_name_is_replay_route_safe() {
        let name = recording_name();
        assert!(name.starts_with("dash-") && name.ends_with(".jsonl"));
        assert!(name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-'));
    }
}
