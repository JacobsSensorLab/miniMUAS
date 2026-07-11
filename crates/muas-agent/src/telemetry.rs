//! Telemetry sample assembly + the optional ndf-spark UDP lane.

use std::net::SocketAddr;

use ndf_spark::{mint_instance, SparkEmitter};
use uas_fleet_data::kinds::TelemetrySample;
use uas_mavlink::BackendTelemetry;

/// Nominal full-battery flight endurance, seconds.
///
/// TODO(strategy): a single conservative constant for the whole airframe
/// class — NOT a real endurance model. Replace with a per-airframe endurance
/// curve or the autopilot's own remaining-time estimate (ArduPilot
/// `BATTERY_STATUS.time_remaining`) once that telemetry is plumbed. Only the
/// PROVIDER STRATEGY's flight-time floor consults the derived estimate; with
/// no strategy record published (today's default) it is never read, so the
/// coarse constant cannot change current behavior.
pub const NOMINAL_ENDURANCE_S: f64 = 900.0;

/// A deliberately simple remaining-flight-time estimate from battery percent:
/// `battery_fraction * NOMINAL_ENDURANCE_S` (linear discharge assumption).
///
/// This is the `flight_time_est_s` the agent feeds the provider strategy's
/// snapshot ([`muas_contracts::strategy::QueueSnapshot`]). See
/// [`NOMINAL_ENDURANCE_S`] for the modeling caveat.
pub fn flight_time_est_s(battery_pct: f64) -> f64 {
    (battery_pct.clamp(0.0, 100.0) / 100.0) * NOMINAL_ENDURANCE_S
}

/// Publisher clock, nanoseconds since the Unix epoch (the v2 `gps_time_ns()`
/// placeholder wall clock until GPS/PPS time is wired into the stack).
pub fn gps_time_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Build the wire [`TelemetrySample`] from a backend snapshot (the v2
/// telemetry dict assembly in `run_drone_agent.py`).
pub fn build_sample(
    vehicle_id: &str,
    busy: &str,
    source: &str,
    t: &BackendTelemetry,
) -> TelemetrySample {
    TelemetrySample {
        agl_alarm: t.agl_alarm,
        agl_m: t.agl_m,
        alt_m: t.alt_m,
        armed: t.armed,
        avoid_bias_m: t.avoid_bias_m,
        battery_pct: t.battery_pct,
        // BackendTelemetry carries no pack voltage; v2's MAVLink dict did.
        // 0.0 is the v2 "unknown voltage" placeholder.
        battery_v: 0.0,
        busy: busy.to_string(),
        gps_time_ns: gps_time_ns(),
        groundspeed_m_s: t.vn_m_s.hypot(t.ve_m_s),
        heading_deg: t.heading_deg,
        lat_deg: t.lat_deg,
        lon_deg: t.lon_deg,
        mode: t.mode.clone(),
        rangefinder_m: t.rangefinder_m,
        source: source.to_string(),
        ve_m_s: t.ve_m_s,
        vehicle_id: vehicle_id.to_string(),
        vn_m_s: t.vn_m_s,
    }
}

/// Checkpoint cadence for the telemetry Spark stream (samples per window).
const SPARK_ANCHOR_EVERY: usize = 32;

/// The UDP `SparkCarrier` binding from the cheat-sheet's real-socket twin:
/// each telemetry sample's JSON bytes ride one Spark datagram to a fixed
/// destination.
///
/// Deviation (documented): window checkpoints are cut by the emitter but not
/// yet signed/anchored — the anchor lane (a Block chain per FS-7) is an M4
/// increment; consumers get seq-ordered latest-wins samples either way.
pub struct SparkLane {
    emitter: SparkEmitter,
    socket: tokio::net::UdpSocket,
    dest: SocketAddr,
}

impl SparkLane {
    /// Bind an ephemeral local socket toward `dest`.
    pub async fn bind(dest: SocketAddr) -> std::io::Result<Self> {
        let local: SocketAddr = if dest.is_ipv4() {
            "0.0.0.0:0".parse().expect("static addr")
        } else {
            "[::]:0".parse().expect("static addr")
        };
        Ok(Self {
            emitter: SparkEmitter::new(mint_instance(), Some(SPARK_ANCHOR_EVERY)),
            socket: tokio::net::UdpSocket::bind(local).await?,
            dest,
        })
    }

    /// Emit one sample; transport errors are logged and swallowed (the spark
    /// lane is lossy by contract).
    pub async fn emit(&mut self, sample_json: &[u8]) {
        let now_us = (gps_time_ns() / 1_000) as i64;
        let out = self.emitter.emit(now_us, sample_json, None);
        if let Err(err) = self.socket.send_to(&out.bytes, self.dest).await {
            tracing::debug!(%err, "spark: send failed (lossy lane, continuing)");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_maps_backend_fields_and_wire_keys() {
        let t = BackendTelemetry {
            lat_deg: 35.0,
            lon_deg: -90.0,
            alt_m: 10.0,
            agl_m: 10.0,
            heading_deg: 90.0,
            armed: true,
            mode: "GUIDED".into(),
            battery_pct: 88.0,
            rangefinder_m: -1.0,
            agl_alarm: false,
            vn_m_s: 3.0,
            ve_m_s: 4.0,
            avoid_bias_m: 2.5,
        };
        let sample = build_sample("iuas-01", "investigate", "sim", &t);
        assert_eq!(sample.vehicle_id, "iuas-01");
        assert_eq!(sample.busy, "investigate");
        assert_eq!(sample.source, "sim");
        assert_eq!(sample.groundspeed_m_s, 5.0);
        assert_eq!(sample.avoid_bias_m, 2.5);
        assert!(sample.gps_time_ns > 0);

        // Wire keys are the v2 Python dict keys (uas-fleet-data pins these;
        // spot-check a few here so the mapping never drifts).
        let value = serde_json::to_value(&sample).unwrap();
        for key in ["lat_deg", "vn_m_s", "avoid_bias_m", "busy", "gps_time_ns"] {
            assert!(value.get(key).is_some(), "missing wire key {key}");
        }
    }
}
