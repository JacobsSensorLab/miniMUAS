//! The per-vehicle task-queue wire types (`tasks/queue` latest-wins stream).
//!
//! Additive contract for the v3 queue engine: every vehicle publishes its
//! ordered task queue as one JSON [`TaskQueueStatus`] document under
//! [`crate::names::TASK_QUEUE_STREAM`] (latest-wins, refreshed at 1 Hz while
//! anything changes). The dashboard's queue panel renders pending tasks with
//! details, time estimates, and percent-complete straight from this shape;
//! reordering goes back through the `queue_reorder` service op
//! ([`crate::services::VehicleService::queue_reorder`]).
//!
//! States and origins are plain strings on the wire (JSON tolerates unknown
//! values — additive evolution), with the known values pinned in
//! [`task_state`] / [`task_origin`].

use serde::{Deserialize, Serialize};

/// Known task states.
pub mod task_state {
    /// The one task currently flying (flight is exclusive; position 0).
    pub const ACTIVE: &str = "active";
    /// Waiting in the queue.
    pub const PENDING: &str = "pending";
    /// Ran to its natural end (also the parent side of a split).
    pub const DONE: &str = "done";
    /// Cancelled (operator abort, ladder command, queue flush).
    pub const ABORTED: &str = "aborted";
    /// Ended without completing (timeout, could not get airborne).
    pub const FAILED: &str = "failed";
}

/// Known task origins.
pub mod task_origin {
    /// Operator-issued from the dashboard/console.
    pub const OPERATOR: &str = "operator";
    /// Auto-dispatched by the mission machine (investigate jobs).
    pub const DISPATCH: &str = "dispatch";
    /// Continuation of a preempted task (see `parent`).
    pub const SPLIT: &str = "split";
}

/// Task kinds the queue engine schedules (watchpoints deliberately stay
/// outside the queue — they ride along with whatever the vehicle is doing).
pub mod task_kind {
    pub const RASTER_SEARCH: &str = "raster-search";
    pub const INVESTIGATE: &str = "investigate";
    pub const SENSOR_OVERRIDE: &str = "sensor-override";
}

/// Live progress of the ACTIVE task, published by its runner.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TaskProgress {
    /// Percent complete, 0..100.
    pub pct: f64,
    /// Human-readable progress note ("leg 3/7, 12 frames").
    pub detail: String,
    /// Estimated seconds to completion (remaining path ÷ commanded speed).
    pub eta_s: f64,
}

/// One queue entry as published on the stream.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct QueuedTaskInfo {
    /// Queue id, `tsk-<n>` (unique per agent boot).
    pub task_id: String,
    /// See [`task_kind`].
    pub kind: String,
    /// Short human digest of the request parameters plus a stable 8-hex
    /// content hash ("7 legs @ 8m agl [3fa2c81b]").
    pub params_digest: String,
    /// See [`task_state`].
    pub state: String,
    /// See [`task_origin`].
    pub origin: String,
    /// Parent task id when `origin == "split"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
    /// GPS time the entry was queued, nanoseconds.
    pub enqueued_ns: u64,
    /// GPS time the task started flying (absent while pending).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub started_ns: Option<u64>,
    /// Live progress (active tasks with a measurable notion of progress).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub progress: Option<TaskProgress>,
    /// Estimated seconds until this PENDING task starts (active remainder
    /// plus estimated durations of everything ahead of it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub eta_to_start_s: Option<f64>,
    /// Rough estimated task duration, seconds (queue ETA arithmetic).
    #[serde(default)]
    pub est_duration_s: f64,
}

/// The whole queue snapshot: active task first, then pending in run order,
/// then a short tail of recently finished entries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TaskQueueStatus {
    pub vehicle_id: String,
    pub gps_time_ns: u64,
    /// Pending-depth limit currently enforced (`queue-full` beyond it).
    pub depth_limit: u32,
    pub tasks: Vec<QueuedTaskInfo>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queue_status_round_trips_and_tolerates_missing_optionals() {
        let status = TaskQueueStatus {
            vehicle_id: "iuas-01".into(),
            gps_time_ns: 42,
            depth_limit: 4,
            tasks: vec![QueuedTaskInfo {
                task_id: "tsk-1".into(),
                kind: task_kind::RASTER_SEARCH.into(),
                params_digest: "5 legs @ 8m [00c0ffee]".into(),
                state: task_state::ACTIVE.into(),
                origin: task_origin::OPERATOR.into(),
                enqueued_ns: 41,
                started_ns: Some(42),
                progress: Some(TaskProgress {
                    pct: 40.0,
                    detail: "leg 2/5".into(),
                    eta_s: 33.0,
                }),
                ..QueuedTaskInfo::default()
            }],
        };
        let bytes = serde_json::to_vec(&status).unwrap();
        let decoded: TaskQueueStatus = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, status);

        // Minimal dicts decode (additive evolution: absent optionals fill).
        let min: QueuedTaskInfo = serde_json::from_str(
            r#"{"task_id":"tsk-2","kind":"investigate","params_digest":"d",
                "state":"pending","origin":"split","enqueued_ns":7}"#,
        )
        .unwrap();
        assert_eq!(min.parent, None);
        assert_eq!(min.progress, None);
        assert_eq!(min.est_duration_s, 0.0);
    }
}
