//! RC-over-NDN wire types (`rc/status` latest-wins stream, RC-CONTROL R1).
//!
//! Additive contract: while an agent's RC receiver task runs it publishes
//! one JSON [`RcStatus`] document under [`crate::names::RC_STATUS_STREAM`]
//! (latest-wins, ~4 Hz plus immediately on every engage / release /
//! failsafe transition). The dashboard R2 pilot surface renders its RC
//! status strip (rate, gap %, age, failsafe state, e-stop) from this shape.
//!
//! Like the task-queue stream, enum-ish values ride as plain strings (JSON
//! tolerates unknown values — additive evolution) with the known
//! vocabulary pinned in [`failsafe_state`].

use serde::{Deserialize, Serialize};

/// Known `failsafe_state` values (the uas-rc silence ladder's states).
pub mod failsafe_state {
    /// RC task running, no stream ever admitted (nothing subsumed).
    pub const IDLE: &str = "idle";
    /// Frames flowing; sticks applied.
    pub const MANUAL: &str = "manual";
    /// Holding position after the hold-silence threshold.
    pub const HOLD: &str = "hold";
    /// Returned to launch after the RTL-silence threshold (the session
    /// released the override on the way).
    pub const RTL: &str = "rtl";
    /// Operator emergency stop; live flag-cleared frames release it.
    pub const EMERGENCY_STOP: &str = "emergency-stop";
}

/// One rc/status sample.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct RcStatus {
    /// Which vehicle this sample describes.
    pub vehicle_id: String,
    /// Publisher clock, ns since the Unix epoch.
    pub gps_time_ns: u64,
    /// An RC session currently owns (or is safing) the vehicle.
    pub engaged: bool,
    /// Receiver binding, `listen:<addr>` (plain UDP) or `spark:<addr>`.
    pub source: String,
    /// Loss-honest gap percentage: seq values that never arrived over the
    /// expected total, 0..100.
    pub seq_gap_pct: f64,
    /// Milliseconds since the last admitted frame; `None` until the first
    /// frame arrives.
    #[serde(default)]
    pub age_ms: Option<u64>,
    /// Silence-ladder state (see [`failsafe_state`]).
    pub failsafe_state: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rc_status_round_trips() {
        let status = RcStatus {
            vehicle_id: "iuas-02".into(),
            gps_time_ns: 42,
            engaged: true,
            source: "listen:0.0.0.0:14650".into(),
            seq_gap_pct: 3.5,
            age_ms: Some(18),
            failsafe_state: failsafe_state::MANUAL.into(),
        };
        let bytes = serde_json::to_vec(&status).unwrap();
        assert_eq!(serde_json::from_slice::<RcStatus>(&bytes).unwrap(), status);
    }

    /// Additive-decode: a minimal (or older) publisher's dict still decodes
    /// — missing `age_ms` is `None`, unknown extra keys are ignored.
    #[test]
    fn rc_status_tolerates_minimal_and_extended_dicts() {
        let minimal: RcStatus = serde_json::from_str(
            r#"{"vehicle_id":"iuas-02","gps_time_ns":1,"engaged":false,
                "source":"","seq_gap_pct":0.0,"failsafe_state":"idle"}"#,
        )
        .unwrap();
        assert_eq!(minimal.age_ms, None);
        assert_eq!(minimal.failsafe_state, failsafe_state::IDLE);

        let extended: RcStatus = serde_json::from_str(
            r#"{"vehicle_id":"iuas-02","gps_time_ns":1,"engaged":true,
                "source":"spark:0.0.0.0:14650","seq_gap_pct":1.0,
                "age_ms":5,"failsafe_state":"manual",
                "some_future_key":{"nested":true}}"#,
        )
        .unwrap();
        assert!(extended.engaged);
        assert_eq!(extended.age_ms, Some(5));
    }
}
