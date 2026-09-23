//! Background jobs: every long operation (deploy, cell switch, measurement, restore) runs as a
//! job with an id, a log file and a result, so an MCP client can poll it and the record survives
//! the process (I8).
//!
//! A running job holds an exclusive `flock` on its `jobs/<id>.log`, so another process (the CLI
//! beside the MCP server) can tell "still running" from "its process died" without trusting pids.

use std::collections::{HashMap, VecDeque};
use std::fs::{File, OpenOptions, TryLockError};
use std::io::Write;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::watch;

use crate::state::{Recorder, iso8601, now_ms};

/// Lines of log kept in memory and returned by [`Jobs::view`].
const TAIL: usize = 40;

pub struct Jobs {
    rec: Arc<Recorder>,
    jobs: Mutex<HashMap<String, Arc<Entry>>>,
}

struct Entry {
    kind: String,
    started_ms: u64,
    inner: Mutex<Inner>,
    done: watch::Sender<bool>,
}

struct Inner {
    /// Held open (and flocked) until the job finishes.
    log: Option<File>,
    tail: VecDeque<String>,
    finished_ms: Option<u64>,
    result: Option<Value>,
    error: Option<String>,
}

#[derive(Clone)]
pub struct JobCtx {
    pub id: String,
    pub rec: Arc<Recorder>,
    entry: Arc<Entry>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct JobView {
    pub id: String,
    pub kind: String,
    /// `running` | `succeeded` | `failed`
    pub status: String,
    pub started_ms: u64,
    pub finished_ms: Option<u64>,
    pub result: Option<Value>,
    pub error: Option<String>,
    #[serde(default)]
    pub log_tail: Vec<String>,
}

impl JobView {
    pub fn finished(&self) -> bool {
        self.status != "running"
    }
}

impl JobCtx {
    /// Timestamped line to `jobs/<id>.log`, the in-memory tail, and stderr.
    pub fn log(&self, line: impl AsRef<str>) {
        let line = format!("{} {}", &iso8601(now_ms())[11..23], line.as_ref());
        eprintln!("[{}] {line}", self.id);
        let mut inner = self.entry.inner.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(f) = inner.log.as_mut() {
            let _ = f.write_all(format!("{line}\n").as_bytes());
        }
        if inner.tail.len() == TAIL {
            inner.tail.pop_front();
        }
        inner.tail.push_back(line);
    }
}

impl Jobs {
    pub fn new(rec: Arc<Recorder>) -> Arc<Jobs> {
        Arc::new(Jobs {
            rec,
            jobs: Mutex::new(HashMap::new()),
        })
    }

    /// Start `f` as job `<stamp>-<kind>`; returns its id at once.
    pub fn spawn<F, Fut>(self: &Arc<Self>, kind: &str, f: F) -> String
    where
        F: FnOnce(JobCtx) -> Fut + Send + 'static,
        Fut: Future<Output = anyhow::Result<Value>> + Send + 'static,
    {
        let id = self.rec.new_id(kind);
        let log = open_locked_log(&self.log_path(&id));
        let (done, _) = watch::channel(false);
        let entry = Arc::new(Entry {
            kind: kind.into(),
            started_ms: now_ms(),
            inner: Mutex::new(Inner {
                log,
                tail: VecDeque::new(),
                finished_ms: None,
                result: None,
                error: None,
            }),
            done,
        });
        self.jobs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(id.clone(), entry.clone());
        let ctx = JobCtx {
            id: id.clone(),
            rec: self.rec.clone(),
            entry,
        };
        self.persist(&ctx);
        self.rec
            .ledger("job_started", json!({ "id": id, "kind": kind }));
        ctx.log(format!("job {id} started ({kind})"));

        let work = tokio::spawn(f(ctx.clone()));
        let jobs = self.clone();
        tokio::spawn(async move {
            let outcome = match work.await {
                Ok(Ok(v)) => Ok(v),
                Ok(Err(e)) => Err(format!("{e:#}")),
                Err(e) if e.is_panic() => Err(format!("job panicked: {e}")),
                Err(e) => Err(format!("job aborted: {e}")),
            };
            match &outcome {
                Ok(_) => ctx.log("job succeeded"),
                Err(e) => ctx.log(format!("job failed: {e}")),
            }
            {
                let mut inner = ctx.entry.inner.lock().unwrap_or_else(|e| e.into_inner());
                inner.finished_ms = Some(now_ms());
                match outcome {
                    Ok(v) => inner.result = Some(v),
                    Err(e) => inner.error = Some(e),
                }
            }
            jobs.persist(&ctx);
            let v = jobs.view_entry(&ctx.id, &ctx.entry);
            jobs.rec.ledger(
                "job_finished",
                json!({ "id": v.id, "kind": v.kind, "status": v.status, "error": v.error }),
            );
            // Release the log flock only after the final record is on disk.
            ctx.entry
                .inner
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .log
                .take();
            ctx.entry.done.send_replace(true);
        });
        id
    }

    /// The job's current view; long-polls until it finishes or `wait` elapses. Jobs started by
    /// another process are read from `jobs/<id>.{json,log}`.
    pub async fn view(&self, id: &str, wait: Duration) -> Option<JobView> {
        let entry = self
            .jobs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(id)
            .cloned();
        if let Some(entry) = entry {
            let mut rx = entry.done.subscribe();
            let _ = tokio::time::timeout(wait, rx.wait_for(|d| *d)).await;
            return Some(self.view_entry(id, &entry));
        }
        let deadline = tokio::time::Instant::now() + wait;
        loop {
            let v = self.view_file(id)?;
            if v.finished() || tokio::time::Instant::now() >= deadline {
                return Some(v);
            }
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }

    /// Ids of jobs this process is still running.
    pub fn running(&self) -> Vec<String> {
        let jobs = self.jobs.lock().unwrap_or_else(|e| e.into_inner());
        jobs.iter()
            .filter(|(_, e)| !*e.done.borrow())
            .map(|(id, _)| id.clone())
            .collect()
    }

    /// Wait until every job this process started has finished: a mutation abandoned half-way
    /// (a rollout between canary and GCS) is worse than a server that exits late.
    pub async fn wait_idle(&self) {
        let entries: Vec<Arc<Entry>> = self
            .jobs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .values()
            .cloned()
            .collect();
        for e in entries {
            let _ = e.done.subscribe().wait_for(|d| *d).await;
        }
    }

    /// Jobs a now-dead process left `running` on disk (their log is no longer locked). Read-only:
    /// the caller repairs what each left behind (streams on, a half-written run, a partial
    /// rollout) and only then records it with [`Jobs::mark_abandoned`], so a recovery that is
    /// itself killed is redone by the next mutation instead of lost. Found in the field: a CLI
    /// measurement killed by its caller's 300 s timeout mid-sample, then four recoveries killed
    /// after marking but before repairing.
    pub fn abandoned(&self) -> Vec<JobView> {
        let Ok(dir) = std::fs::read_dir(self.rec.dir().join("jobs")) else {
            return Vec::new();
        };
        let mut found: Vec<JobView> = dir
            .flatten()
            .filter_map(|entry| {
                let name = entry.file_name();
                let id = name.to_str()?.strip_suffix(".json")?;
                let v: JobView = self.rec.read_json(&format!("jobs/{id}.json")).ok()?;
                (!v.finished() && !self.is_live(id)).then_some(v)
            })
            .collect();
        found.sort_by(|a, b| a.id.cmp(&b.id));
        found
    }

    /// Record an abandoned job as failed, once what it left behind is repaired.
    pub fn mark_abandoned(&self, job: &JobView) -> anyhow::Result<()> {
        let mut v = job.clone();
        v.status = "failed".into();
        v.finished_ms = Some(crate::state::now_ms());
        v.error = Some("abandoned: the process running this job exited before it finished".into());
        v.log_tail.clear();
        self.rec.write_json(&format!("jobs/{}.json", v.id), &v)
    }

    /// Whether a process is still running job `id` (it holds the job log's lock).
    pub fn is_live(&self, id: &str) -> bool {
        log_is_locked(&self.log_path(id))
    }

    fn view_entry(&self, id: &str, entry: &Entry) -> JobView {
        let inner = entry.inner.lock().unwrap_or_else(|e| e.into_inner());
        JobView {
            id: id.into(),
            kind: entry.kind.clone(),
            status: match (&inner.finished_ms, &inner.error) {
                (None, _) => "running",
                (Some(_), None) => "succeeded",
                (Some(_), Some(_)) => "failed",
            }
            .into(),
            started_ms: entry.started_ms,
            finished_ms: inner.finished_ms,
            result: inner.result.clone(),
            error: inner.error.clone(),
            log_tail: inner.tail.iter().cloned().collect(),
        }
    }

    fn view_file(&self, id: &str) -> Option<JobView> {
        let mut v: JobView = self.rec.read_json(&format!("jobs/{id}.json")).ok()?;
        let log_path = self.log_path(id);
        if !v.finished() && !log_is_locked(&log_path) {
            v.status = "failed".into();
            v.error = Some(format!(
                "the process running this job exited before it finished (see {})",
                log_path.display()
            ));
        }
        let log = std::fs::read_to_string(&log_path).unwrap_or_default();
        let lines: Vec<&str> = log.lines().collect();
        v.log_tail = lines[lines.len().saturating_sub(TAIL)..]
            .iter()
            .map(|s| s.to_string())
            .collect();
        Some(v)
    }

    fn persist(&self, ctx: &JobCtx) {
        let mut v = self.view_entry(&ctx.id, &ctx.entry);
        v.log_tail.clear();
        if let Err(e) = self.rec.write_json(&format!("jobs/{}.json", ctx.id), &v) {
            eprintln!("ndn-fleet: cannot record job {}: {e:#}", ctx.id);
        }
    }

    fn log_path(&self, id: &str) -> PathBuf {
        self.rec.dir().join(format!("jobs/{id}.log"))
    }
}

fn open_locked_log(path: &PathBuf) -> Option<File> {
    let f = OpenOptions::new().create(true).append(true).open(path);
    match f {
        Ok(f) => {
            let _ = f.try_lock();
            Some(f)
        }
        Err(e) => {
            eprintln!("ndn-fleet: cannot open {}: {e}", path.display());
            None
        }
    }
}

fn log_is_locked(path: &PathBuf) -> bool {
    let Ok(f) = File::open(path) else {
        return false;
    };
    match f.try_lock_shared() {
        Ok(()) => {
            let _ = f.unlock();
            false
        }
        Err(TryLockError::WouldBlock) => true,
        Err(TryLockError::Error(_)) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn a_failed_job_reports_its_error_and_another_process_sees_it_finished() {
        let dir = std::env::temp_dir().join(format!("ndn-fleet-jobs-{}", std::process::id()));
        let rec = Recorder::open_dir(&dir).unwrap();
        let jobs = Jobs::new(rec.clone());
        let id = jobs.spawn("t", |ctx| async move {
            ctx.log("working");
            anyhow::bail!("boom")
        });
        let v = jobs.view(&id, Duration::from_secs(5)).await.unwrap();
        assert_eq!(v.status, "failed");
        assert!(v.error.as_deref().unwrap().contains("boom"));
        assert!(v.log_tail.iter().any(|l| l.ends_with("working")));

        // A fresh `Jobs` (another process) reads the persisted record, not "running".
        let other = Jobs::new(rec);
        let v = other.view(&id, Duration::ZERO).await.unwrap();
        assert_eq!(v.status, "failed");
    }

    #[tokio::test]
    async fn a_running_job_is_seen_running_from_another_process() {
        let dir = std::env::temp_dir().join(format!("ndn-fleet-jobs2-{}", std::process::id()));
        let rec = Recorder::open_dir(&dir).unwrap();
        let jobs = Jobs::new(rec.clone());
        let (tx, rx) = tokio::sync::oneshot::channel::<()>();
        let id = jobs.spawn("t", |_ctx| async move {
            let _ = rx.await;
            Ok(json!({ "ok": true }))
        });
        let other = Jobs::new(rec);
        assert_eq!(
            other.view(&id, Duration::ZERO).await.unwrap().status,
            "running"
        );
        tx.send(()).unwrap();
        let v = jobs.view(&id, Duration::from_secs(5)).await.unwrap();
        assert_eq!(v.status, "succeeded");
        assert_eq!(v.result, Some(json!({ "ok": true })));
    }

    /// A job a dead process left "running" is found and recorded failed once; a job this process
    /// is still running (its log is locked) is never taken for abandoned.
    #[tokio::test]
    async fn only_jobs_of_dead_processes_are_abandoned() {
        let dir = std::env::temp_dir().join(format!("ndn-fleet-jobs3-{}", std::process::id()));
        let rec = Recorder::open_dir(&dir).unwrap();
        let jobs = Jobs::new(rec.clone());

        // What a killed process leaves: a "running" record and an unlocked log.
        let dead = JobView {
            id: "20260923T165529Z-measure".into(),
            kind: "measure".into(),
            status: "running".into(),
            started_ms: 1,
            finished_ms: None,
            result: None,
            error: None,
            log_tail: Vec::new(),
        };
        rec.write_json(&format!("jobs/{}.json", dead.id), &dead)
            .unwrap();
        std::fs::write(dir.join(format!("jobs/{}.log", dead.id)), "started\n").unwrap();

        let (tx, rx) = tokio::sync::oneshot::channel::<()>();
        let live = jobs.spawn("measure", |_ctx| async move {
            let _ = rx.await;
            Ok(json!({}))
        });
        // Let the live job persist its own "running" record.
        tokio::time::sleep(Duration::from_millis(100)).await;

        let found = jobs.abandoned();
        assert_eq!(
            found.iter().map(|j| j.id.as_str()).collect::<Vec<_>>(),
            [dead.id.as_str()]
        );
        // Detection alone must not consume it: a recovery killed before repairing is redone.
        assert_eq!(jobs.abandoned().len(), 1, "still found until marked");
        jobs.mark_abandoned(&found[0]).unwrap();
        let v: JobView = rec.read_json(&format!("jobs/{}.json", dead.id)).unwrap();
        assert_eq!(v.status, "failed");
        assert!(jobs.abandoned().is_empty(), "marked once, not re-found");

        tx.send(()).unwrap();
        assert_eq!(
            jobs.view(&live, Duration::from_secs(5))
                .await
                .unwrap()
                .status,
            "succeeded"
        );
    }
}
