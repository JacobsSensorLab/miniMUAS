//! Power-loss-safe event journal (v2 parity) + optional Block-chain mirror.
//!
//! v2 journaled every `print_json` event to per-line-flushed JSONL with an
//! fsync per line; journal failures never killed the process. This port keeps
//! that contract: [`JournalHandle::event`] is fire-and-forget, the writer task
//! does `write → flush → sync_data` per line and only ever `warn!`s on error.
//!
//! When an ndf-apps [`AppRuntime`] is attached (`--journal-chain`), the same
//! event lines are ALSO mirrored into a signed Block chain via
//! `AppRuntime::publish`. Batching choice (documented per the M3 brief): one
//! Block per **2-second window** (plus a final flush on shutdown) — journal
//! lines are small and bursty, and a time window bounds both Block count and
//! loss horizon; per-event Blocks would sign 4 Hz telemetry-adjacent chatter,
//! and a count-only batch could hold a tail event back indefinitely.

use std::io::Write;
use std::path::PathBuf;

use ndf_apps::{AppRuntime, ChainAddress};
use tokio::sync::{mpsc, oneshot};
use tracing::{info, warn};

/// One journal message.
enum Msg {
    Event(serde_json::Value),
    /// Barrier: flush + fsync the JSONL file and flush the chain batch, then
    /// ack. Used by the shutdown path ("flush journal + sync" before exit).
    Sync(oneshot::Sender<()>),
}

/// Cloneable fire-and-forget sender into the journal task.
#[derive(Clone)]
pub struct JournalHandle {
    tx: mpsc::UnboundedSender<Msg>,
}

impl JournalHandle {
    /// Journal an event line. `kind` is the v2 event kind string
    /// (`"service.flight_takeoff"`, `"coord.coop"`, ...); `fields` merge into
    /// the line alongside `ts_ns`/`kind`/`vehicle_id`.
    pub fn event(&self, kind: &str, mut fields: serde_json::Value) {
        if !fields.is_object() {
            fields = serde_json::json!({ "value": fields });
        }
        let map = fields.as_object_mut().expect("object ensured above");
        map.insert("kind".into(), serde_json::json!(kind));
        map.insert("ts_ns".into(), serde_json::json!(crate::telemetry::gps_time_ns()));
        // Fire-and-forget: a closed journal must never take the agent down.
        let _ = self.tx.send(Msg::Event(fields));
    }

    /// Flush + fsync barrier; resolves once everything sent before it is
    /// durable (or the journal task is gone).
    pub async fn sync(&self) {
        let (ack_tx, ack_rx) = oneshot::channel();
        if self.tx.send(Msg::Sync(ack_tx)).is_ok() {
            let _ = ack_rx.await;
        }
    }
}

/// The chain-mirror half: an attached AppRuntime and the journal chain.
pub struct ChainMirror {
    pub runtime: AppRuntime,
    pub address: ChainAddress,
}

/// Chain-mirror batch window (see module docs).
const CHAIN_BATCH_WINDOW: std::time::Duration = std::time::Duration::from_secs(2);

struct Writer {
    file: Option<std::fs::File>,
    vehicle_id: String,
    chain: Option<ChainMirror>,
    batch: Vec<u8>,
}

impl Writer {
    fn write_line(&mut self, mut value: serde_json::Value) {
        if let Some(map) = value.as_object_mut() {
            map.entry("vehicle_id")
                .or_insert_with(|| serde_json::json!(self.vehicle_id));
        }
        let line = match serde_json::to_vec(&value) {
            Ok(line) => line,
            Err(err) => {
                warn!(%err, "journal: event line failed to encode");
                return;
            }
        };
        if let Some(file) = self.file.as_mut() {
            // Per-line flush + fsync; failures are logged, never fatal.
            let write = file
                .write_all(&line)
                .and_then(|()| file.write_all(b"\n"))
                .and_then(|()| file.flush())
                .and_then(|()| file.sync_data());
            if let Err(err) = write {
                warn!(%err, "journal: write failed (continuing)");
            }
        }
        if self.chain.is_some() {
            self.batch.extend_from_slice(&line);
            self.batch.push(b'\n');
        }
    }

    async fn flush_chain(&mut self) {
        if self.batch.is_empty() {
            return;
        }
        let Some(mirror) = self.chain.as_mut() else {
            self.batch.clear();
            return;
        };
        let payload = std::mem::take(&mut self.batch);
        match mirror
            .runtime
            .publish(&mirror.address, "application/x-ndjson", &payload)
            .await
        {
            Ok(receipt) => {
                tracing::debug!(chain_seq = receipt.chain_seq, "journal: chain block published")
            }
            Err(err) => warn!(?err, "journal: chain mirror publish failed (continuing)"),
        }
    }

    fn sync_file(&mut self) {
        if let Some(file) = self.file.as_mut() {
            let _ = file.flush().and_then(|()| file.sync_all());
        }
    }
}

/// Spawn the journal task. `log_dir = None` disables the JSONL file (events
/// still reach tracing and the optional chain mirror).
pub fn spawn(
    vehicle_id: &str,
    log_dir: Option<PathBuf>,
    chain: Option<ChainMirror>,
) -> (JournalHandle, tokio::task::JoinHandle<()>) {
    let file = log_dir.and_then(|dir| {
        if let Err(err) = std::fs::create_dir_all(&dir) {
            warn!(%err, dir = %dir.display(), "journal: cannot create log dir; journaling to tracing only");
            return None;
        }
        let path = dir.join(format!(
            "agent-{}-{}.jsonl",
            vehicle_id,
            crate::telemetry::gps_time_ns() / 1_000_000_000
        ));
        match std::fs::OpenOptions::new().create(true).append(true).open(&path) {
            Ok(file) => {
                info!(path = %path.display(), "journal: writing power-loss-safe JSONL");
                Some(file)
            }
            Err(err) => {
                warn!(%err, path = %path.display(), "journal: cannot open journal file");
                None
            }
        }
    });

    let (tx, mut rx) = mpsc::unbounded_channel();
    let mut writer = Writer {
        file,
        vehicle_id: vehicle_id.to_string(),
        chain,
        batch: Vec::new(),
    };
    let task = tokio::spawn(async move {
        let mut window = tokio::time::interval(CHAIN_BATCH_WINDOW);
        window.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                msg = rx.recv() => match msg {
                    Some(Msg::Event(value)) => writer.write_line(value),
                    Some(Msg::Sync(ack)) => {
                        writer.sync_file();
                        writer.flush_chain().await;
                        let _ = ack.send(());
                    }
                    None => break,
                },
                _ = window.tick() => writer.flush_chain().await,
            }
        }
        writer.sync_file();
        writer.flush_chain().await;
    });
    (JournalHandle { tx }, task)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn journal_lines_are_durable_and_tagged() {
        let dir = std::env::temp_dir().join(format!("muas-journal-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let (journal, task) = spawn("iuas-01", Some(dir.clone()), None);
        journal.event("service.flight_takeoff", serde_json::json!({"agl_m": 5.0}));
        journal.event("coord.coop", serde_json::json!({"peer": "wuas-01"}));
        journal.sync().await;

        let file = std::fs::read_dir(&dir)
            .unwrap()
            .next()
            .expect("journal file exists")
            .unwrap();
        let text = std::fs::read_to_string(file.path()).unwrap();
        let lines: Vec<serde_json::Value> = text
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .collect();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0]["kind"], "service.flight_takeoff");
        assert_eq!(lines[0]["vehicle_id"], "iuas-01");
        assert_eq!(lines[0]["agl_m"], 5.0);
        assert!(lines[0]["ts_ns"].as_u64().unwrap() > 0);
        assert_eq!(lines[1]["kind"], "coord.coop");

        drop(journal);
        task.await.unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test]
    async fn journal_without_file_never_errors() {
        let (journal, task) = spawn("iuas-01", None, None);
        journal.event("noop", serde_json::json!({}));
        journal.sync().await;
        drop(journal);
        task.await.unwrap();
    }
}
