//! The deployment's anomaly world model — simulation truth lives HERE, not
//! in the dashboard and not in the agents.
//!
//! [`AnomalySource`] is the read seam consumers (the NDN ground-truth
//! server, the net exporter) see; [`AnomalyField`] is the hand-placed
//! backend this increment ships (operator clicks + `--verify` scripting via
//! the control endpoint). Future backends — scripted timelines, digital
//! twins — implement the same trait and register a new
//! [`AnomalySourceConfig`] tag; nothing downstream changes.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use muas_contracts::anomaly::Anomaly;
use serde::{Deserialize, Serialize};

/// Read seam over the anomaly world model.
pub trait AnomalySource: Send + Sync {
    /// The current ground truth (order unspecified).
    fn snapshot(&self) -> Vec<Anomaly>;
}

/// Hand-placed anomaly field: the operator (dashboard tool) and scripts
/// (control endpoint) mutate it; everything else reads [`AnomalySource`].
#[derive(Default)]
pub struct AnomalyField {
    anomalies: Mutex<Vec<Anomaly>>,
    next_id: AtomicU64,
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

impl AnomalyField {
    pub fn new() -> Self {
        Self::default()
    }

    /// Place one anomaly: assigns an id when the request carries none,
    /// stamps `created_ns`, returns the stored value.
    pub fn place(&self, mut anomaly: Anomaly) -> Anomaly {
        if anomaly.id().is_empty() {
            let n = self.next_id.fetch_add(1, Ordering::Relaxed) + 1;
            anomaly.set_id(format!("anom-{n}"));
        }
        anomaly.set_created_ns(now_ns());
        let mut anomalies = lock(&self.anomalies);
        anomalies.retain(|a| a.id() != anomaly.id()); // idempotent upsert
        anomalies.push(anomaly.clone());
        anomaly
    }

    /// Remove by id; true when something was removed.
    pub fn remove(&self, id: &str) -> bool {
        let mut anomalies = lock(&self.anomalies);
        let before = anomalies.len();
        anomalies.retain(|a| a.id() != id);
        anomalies.len() != before
    }

    /// Remove everything; returns how many were removed.
    pub fn clear(&self) -> usize {
        std::mem::take(&mut *lock(&self.anomalies)).len()
    }
}

impl AnomalySource for AnomalyField {
    fn snapshot(&self) -> Vec<Anomaly> {
        lock(&self.anomalies).clone()
    }
}

/// Serde-tagged backend selection — the pluggable seam for future anomaly
/// sources. `field` (hand-placed, optionally pre-seeded) is the only
/// backend today; scripted timelines and digital twins are additive tags
/// whose builders return the same [`AnomalySource`] view (only `field`
/// exposes the placement API the control endpoint mutates).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(tag = "backend", rename_all = "snake_case")]
pub enum AnomalySourceConfig {
    #[default]
    Field,
    /// Hand-placed field pre-seeded with `initial` anomalies.
    SeededField {
        #[serde(default)]
        initial: Vec<Anomaly>,
    },
}

impl AnomalySourceConfig {
    /// Build the configured backend.
    pub fn build(&self) -> Arc<AnomalyField> {
        let field = Arc::new(AnomalyField::new());
        if let Self::SeededField { initial } = self {
            for anomaly in initial {
                field.place(anomaly.clone());
            }
        }
        field
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn visual(id: &str) -> Anomaly {
        Anomaly::Visual {
            id: id.into(),
            lat_deg: 35.0,
            lon_deg: 149.0,
            size_m: 4.0,
            signature: "red".into(),
            created_ns: 0,
        }
    }

    #[test]
    fn place_assigns_ids_and_upserts() {
        let field = AnomalyField::new();
        let a = field.place(visual(""));
        assert_eq!(a.id(), "anom-1");
        let b = field.place(visual(""));
        assert_eq!(b.id(), "anom-2");
        assert_eq!(field.snapshot().len(), 2);

        // Re-placing an existing id replaces, never duplicates.
        let again = field.place(visual("anom-1"));
        assert_eq!(again.id(), "anom-1");
        assert_eq!(field.snapshot().len(), 2);

        assert!(field.remove("anom-2"));
        assert!(!field.remove("anom-2"));
        assert_eq!(field.clear(), 1);
        assert!(field.snapshot().is_empty());
    }

    #[test]
    fn config_builds_seeded_backends() {
        let config: AnomalySourceConfig = serde_json::from_str(
            r#"{"backend":"seeded_field","initial":[
                {"kind":"audio","id":"s1","lat_deg":35.0,"lon_deg":149.0,"loudness_db":80.0}
            ]}"#,
        )
        .unwrap();
        let field = config.build();
        assert_eq!(field.snapshot().len(), 1);
        assert_eq!(field.snapshot()[0].kind(), "audio");
        assert_eq!(AnomalySourceConfig::default().build().snapshot().len(), 0);
    }
}
