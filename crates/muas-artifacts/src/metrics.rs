//! Outcome metrics computed **on demand** from the one [`MissionDataset`].
//!
//! Deliberately a set of pure functions: no artifact keeps a transformed
//! copy of the data — every renderer calls [`metrics`] against the same
//! dataset at render time, so two artifacts can never disagree about what
//! happened (they are reading the same Blocks).

use std::collections::BTreeMap;

use crate::dataset::{DatumKind, MissionDataset};

/// Percentile summary of a sample set (milliseconds).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Pctl {
    /// Median.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// Max.
    pub max: f64,
    /// Sample count.
    pub n: usize,
}

fn pctl(samples: &mut [f64]) -> Option<Pctl> {
    if samples.is_empty() {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).expect("finite samples"));
    let q = |p: f64| samples[((samples.len() - 1) as f64 * p).round() as usize];
    Some(Pctl { p50: q(0.50), p95: q(0.95), max: *samples.last().expect("non-empty"), n: samples.len() })
}

/// Per-vehicle flight summary derived from telemetry samples.
#[derive(Clone, Debug, Default)]
pub struct VehicleSummary {
    /// Telemetry samples seen.
    pub samples: usize,
    /// Max AGL (m).
    pub max_agl_m: f64,
    /// Max ground speed (m/s).
    pub max_speed_m_s: f64,
    /// Battery percent, first → last sample (when reported).
    pub battery_pct: Option<(f64, f64)>,
    /// Flight-controller modes seen, in first-seen order.
    pub modes: Vec<String>,
    /// Telemetry inter-arrival stats (ms).
    pub interarrival_ms: Option<Pctl>,
}

/// The run's outcome metrics.
#[derive(Clone, Debug, Default)]
pub struct RunMetrics {
    /// Mission wall span, seconds.
    pub duration_s: f64,
    /// Per-vehicle summaries.
    pub vehicles: BTreeMap<String, VehicleSummary>,
    /// Takeoff attempts (`flight.takeoff.result` data).
    pub takeoff_attempts: u32,
    /// Takeoffs that reported airborne.
    pub takeoff_ok: u32,
    /// Cooperative avoidance confirmations (`coord.coop`).
    pub coop: u32,
    /// Uncooperative fallbacks (`coord.unco`).
    pub unco: u32,
    /// Max vertical avoidance bias seen (m).
    pub max_bias_m: Option<f64>,
    /// Smart-RTL outcome (`rtl.done`), last one wins.
    pub rtl_outcome: Option<String>,
    /// Service invocations journaled (`service.*`).
    pub service_ops: u32,
    /// Service invocations rejected (`accepted == false`).
    pub service_rejected: u32,
    /// Service round-trip stats (ms) when events carry `rtt_ms`.
    pub service_rtt_ms: Option<Pctl>,
    /// `telemetry_stale` marks broadcast by the dashboard.
    pub stale_marks: u32,
    /// Spark telemetry-lane frame loss percent, when journaled
    /// (`spark.stats` events with `frame_loss_pct`).
    pub spark_loss_pct: Option<f64>,
    /// Mean video bitrate (kbps) across `video_stats`, when present.
    pub video_kbps_mean: Option<f64>,
}

impl RunMetrics {
    /// Cooperative-avoidance success rate (percent), when any episode ran.
    pub fn coop_rate_pct(&self) -> Option<f64> {
        let total = self.coop + self.unco;
        (total > 0).then(|| self.coop as f64 / total as f64 * 100.0)
    }

    /// Fleet-wide telemetry inter-arrival (ms) pooled across vehicles.
    pub fn telemetry_interarrival_ms(&self) -> Option<Pctl> {
        let mut all: Vec<f64> = Vec::new();
        for v in self.vehicles.values() {
            if let Some(p) = &v.interarrival_ms {
                // Re-derive from per-vehicle stats is lossy; pool medians is
                // wrong. We store per-vehicle only; the fleet view samples
                // the per-vehicle p50/p95 as representative points.
                all.push(p.p50);
                all.push(p.p95);
            }
        }
        pctl(&mut all)
    }
}

/// Compute the outcome metrics for one dataset.
pub fn metrics(ds: &MissionDataset) -> RunMetrics {
    let mut m = RunMetrics {
        duration_s: (ds.t1().saturating_sub(ds.t0())) as f64 / 1e9,
        ..RunMetrics::default()
    };

    // Telemetry per vehicle.
    let mut arrivals: BTreeMap<String, Vec<u64>> = BTreeMap::new();
    for d in ds.of_kind(DatumKind::Telemetry) {
        let Some(vid) = d.vehicle.clone() else { continue };
        let s = &d.body["sample"];
        let v = m.vehicles.entry(vid.clone()).or_default();
        v.samples += 1;
        if let Some(agl) = s["agl_m"].as_f64() {
            v.max_agl_m = v.max_agl_m.max(agl);
        }
        if let Some(spd) = s["groundspeed_m_s"].as_f64() {
            v.max_speed_m_s = v.max_speed_m_s.max(spd);
        }
        if let Some(b) = s["battery_pct"].as_f64().filter(|b| *b >= 0.0) {
            v.battery_pct = Some(match v.battery_pct {
                Some((first, _)) => (first, b),
                None => (b, b),
            });
        }
        if let Some(mode) = s["mode"].as_str() {
            if !v.modes.iter().any(|m| m == mode) {
                v.modes.push(mode.to_string());
            }
        }
        arrivals.entry(vid).or_default().push(d.t_ns);
    }
    for (vid, times) in arrivals {
        let mut deltas: Vec<f64> =
            times.windows(2).map(|w| (w[1].saturating_sub(w[0])) as f64 / 1e6).collect();
        if let Some(v) = m.vehicles.get_mut(&vid) {
            v.interarrival_ms = pctl(&mut deltas);
        }
    }

    // Coordination — counted per EPISODE, not per journal line: both sides
    // of a pair journal the same maneuver (that mutual publication IS the
    // agreement), so mirrored events within the merge window collapse.
    const EPISODE_MERGE_NS: u64 = 3_000_000_000;
    let mut last_episode: BTreeMap<(String, String), u64> = BTreeMap::new();
    for d in ds.of_kind(DatumKind::Coord) {
        if let Some(bias) = d.body["bias_m"].as_f64() {
            m.max_bias_m = Some(m.max_bias_m.unwrap_or(0.0).max(bias.abs()));
        }
        if !matches!(d.label.as_str(), "coord.coop" | "coord.unco") {
            continue;
        }
        let a = d.vehicle.clone().unwrap_or_default();
        let b = d.body["peer"].as_str().unwrap_or_default().to_string();
        let pair = if a <= b { (a, b) } else { (b, a) };
        let key = (d.label.clone(), format!("{}|{}", pair.0, pair.1));
        if last_episode
            .get(&key)
            .is_some_and(|last| d.t_ns.saturating_sub(*last) < EPISODE_MERGE_NS)
        {
            last_episode.insert(key, d.t_ns);
            continue; // the peer's mirror of an already-counted episode
        }
        last_episode.insert(key, d.t_ns);
        match d.label.as_str() {
            "coord.coop" => m.coop += 1,
            _ => m.unco += 1,
        }
    }

    // Services + flight results.
    let mut rtts: Vec<f64> = Vec::new();
    for d in ds.of_kind(DatumKind::Service) {
        if d.label == "flight.takeoff.result" {
            m.takeoff_attempts += 1;
            if d.body["airborne"].as_bool().unwrap_or(false) {
                m.takeoff_ok += 1;
            }
            continue;
        }
        if d.label.starts_with("service.") {
            m.service_ops += 1;
            if d.body["accepted"].as_bool() == Some(false) {
                m.service_rejected += 1;
            }
            if let Some(rtt) = d.body["rtt_ms"].as_f64() {
                rtts.push(rtt);
            }
        }
    }
    m.service_rtt_ms = pctl(&mut rtts);

    // RTL.
    for d in ds.of_kind(DatumKind::Rtl) {
        if d.label == "rtl.done" {
            if let Some(outcome) = d.body["outcome"].as_str() {
                m.rtl_outcome = Some(outcome.to_string());
            }
        }
    }

    // Link lane.
    let mut kbps: Vec<f64> = Vec::new();
    for d in ds.of_kind(DatumKind::Link) {
        match d.label.as_str() {
            "telemetry_stale" => m.stale_marks += 1,
            "video_stats" => {
                if let Some(k) = d.body["kbps"].as_f64() {
                    kbps.push(k);
                }
            }
            _ => {
                if let Some(loss) = d.body["frame_loss_pct"].as_f64() {
                    m.spark_loss_pct = Some(loss);
                }
            }
        }
    }
    if !kbps.is_empty() {
        m.video_kbps_mean = Some(kbps.iter().sum::<f64>() / kbps.len() as f64);
    }

    m
}

/// One comparison-table cell: display string + numeric sort key when the
/// value is a number.
#[derive(Clone, Debug)]
pub struct MetricCell {
    /// Rendered value.
    pub display: String,
    /// Numeric sort key.
    pub num: Option<f64>,
}

fn cell_f(v: f64, decimals: usize) -> MetricCell {
    MetricCell { display: format!("{v:.decimals$}"), num: Some(v) }
}

/// The outcome columns of the comparison table, in a fixed order. Columns a
/// run has no data for are simply absent (the table shows a dash — honest
/// absence, never a guessed zero).
pub fn outcome_columns(m: &RunMetrics) -> BTreeMap<String, MetricCell> {
    let mut out = BTreeMap::new();
    out.insert("duration s".into(), cell_f(m.duration_s, 0));
    if let Some(rate) = m.coop_rate_pct() {
        out.insert("coop rate %".into(), cell_f(rate, 0));
        out.insert(
            "coop/unco".into(),
            MetricCell {
                display: format!("{}/{}", m.coop, m.unco),
                num: Some(m.coop as f64),
            },
        );
    }
    if let Some(b) = m.max_bias_m {
        out.insert("max bias m".into(), cell_f(b, 1));
    }
    if let Some(p) = m.telemetry_interarrival_ms() {
        out.insert("telem p50 ms".into(), cell_f(p.p50, 1));
        out.insert("telem p95 ms".into(), cell_f(p.p95, 1));
    }
    if let Some(p) = &m.service_rtt_ms {
        out.insert("svc rtt p50 ms".into(), cell_f(p.p50, 1));
        out.insert("svc rtt p95 ms".into(), cell_f(p.p95, 1));
    }
    if m.service_ops > 0 {
        out.insert(
            "svc ok/rejected".into(),
            MetricCell {
                display: format!("{}/{}", m.service_ops - m.service_rejected, m.service_rejected),
                num: Some(m.service_rejected as f64),
            },
        );
    }
    if let Some(loss) = m.spark_loss_pct {
        out.insert("spark loss %".into(), cell_f(loss, 1));
    }
    if m.stale_marks > 0 {
        out.insert("stale marks".into(), cell_f(m.stale_marks as f64, 0));
    }
    if m.takeoff_attempts > 0 {
        out.insert(
            "takeoffs".into(),
            MetricCell {
                display: format!("{}/{}", m.takeoff_ok, m.takeoff_attempts),
                num: Some(m.takeoff_ok as f64),
            },
        );
    }
    if let Some(outcome) = &m.rtl_outcome {
        out.insert("rtl".into(), MetricCell { display: outcome.clone(), num: None });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dataset::BlockRef;
    use serde_json::json;

    #[test]
    fn metrics_fold_the_fixture_shapes() {
        let mut ds = MissionDataset::new();
        let journal = [
            json!({"kind":"coord.coop","ts_ns":10_000_000_000u64,"vehicle_id":"a","peer":"b","bias_m":2.0}),
            json!({"kind":"coord.unco","ts_ns":20_000_000_000u64,"vehicle_id":"a","peer":"b","bias_m":4.0}),
            json!({"kind":"service.flight_takeoff","ts_ns":1_000_000_000u64,"vehicle_id":"a","accepted":true,"rtt_ms":12.0}),
            json!({"kind":"flight.takeoff.result","ts_ns":5_000_000_000u64,"vehicle_id":"a","airborne":true}),
            json!({"kind":"rtl.done","ts_ns":30_000_000_000u64,"vehicle_id":"a","outcome":"landed"}),
            json!({"kind":"spark.stats","ts_ns":29_000_000_000u64,"vehicle_id":"a","frame_loss_pct":1.3}),
        ];
        let payload = journal.iter().map(|l| l.to_string()).collect::<Vec<_>>().join("\n");
        ds.add_journal_block(
            BlockRef { hash: [1; 32], chain: "/j".into(), seq: 0 },
            payload.as_bytes(),
        );
        let telem: Vec<String> = (0..5)
            .map(|i| {
                json!({"t_ns": 1_000_000_000u64 + i * 200_000_000, "event": {"type":"telemetry","vehicle":"a",
                       "sample":{"agl_m": 5.0 + i as f64, "groundspeed_m_s": 2.0, "battery_pct": 90.0 - i as f64, "mode":"GUIDED"}}})
                .to_string()
            })
            .collect();
        ds.add_recording_block(
            BlockRef { hash: [2; 32], chain: "/r".into(), seq: 0 },
            telem.join("\n").as_bytes(),
        );
        ds.finish();

        let m = metrics(&ds);
        assert_eq!(m.coop, 1);
        assert_eq!(m.unco, 1);
        assert_eq!(m.coop_rate_pct(), Some(50.0));
        assert_eq!(m.max_bias_m, Some(4.0));
        assert_eq!(m.takeoff_attempts, 1);
        assert_eq!(m.takeoff_ok, 1);
        assert_eq!(m.service_ops, 1);
        assert_eq!(m.rtl_outcome.as_deref(), Some("landed"));
        assert_eq!(m.spark_loss_pct, Some(1.3));
        let v = &m.vehicles["a"];
        assert_eq!(v.samples, 5);
        assert_eq!(v.max_agl_m, 9.0);
        assert_eq!(v.battery_pct, Some((90.0, 86.0)));
        let ia = v.interarrival_ms.as_ref().expect("inter-arrival");
        assert_eq!(ia.n, 4);
        assert!((ia.p50 - 200.0).abs() < 1e-9);
        let cols = outcome_columns(&m);
        assert!(cols.contains_key("coop rate %"));
        assert!(cols.contains_key("spark loss %"));
    }

    #[test]
    fn mirrored_coord_events_count_as_one_episode() {
        // Both sides of the pair journal the same maneuver ~0.3 s apart;
        // the mutual publication is ONE episode. A later episode of the
        // same pair (outside the merge window) counts again.
        let mut ds = MissionDataset::new();
        let t = 10_000_000_000u64;
        let lines = [
            json!({"kind":"coord.coop","ts_ns":t,"vehicle_id":"iuas-01","peer":"wuas-01","bias_m":2.0}),
            json!({"kind":"coord.coop","ts_ns":t + 300_000_000,"vehicle_id":"wuas-01","peer":"iuas-01","bias_m":-2.0}),
            json!({"kind":"coord.coop","ts_ns":t + 9_000_000_000,"vehicle_id":"iuas-01","peer":"wuas-01","bias_m":2.0}),
            json!({"kind":"coord.coop","ts_ns":t + 9_300_000_000,"vehicle_id":"wuas-01","peer":"iuas-01","bias_m":-2.0}),
            json!({"kind":"coord.unco","ts_ns":t + 20_000_000_000u64,"vehicle_id":"iuas-01","peer":"wuas-01","bias_m":4.5}),
        ];
        let payload = lines.iter().map(|l| l.to_string()).collect::<Vec<_>>().join("\n");
        ds.add_journal_block(
            BlockRef { hash: [3; 32], chain: "/j".into(), seq: 0 },
            payload.as_bytes(),
        );
        ds.finish();
        let m = metrics(&ds);
        assert_eq!(m.coop, 2, "two coop episodes, each journaled by both sides");
        assert_eq!(m.unco, 1);
        assert_eq!(m.max_bias_m, Some(4.5));
    }
}
