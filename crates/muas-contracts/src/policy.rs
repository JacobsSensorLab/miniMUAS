//! The v2 ack-gating rules, as pure typed functions.
//!
//! v2 enforced its field-safety rails inline in `run_drone_agent.py` ack
//! callbacks (`--max-range-m 300`, `--max-agl-m 20`, min AGL 3.5 on MAVLink,
//! busy guard, shutdown confirm-phrase, `--audio-range-m`). Here each rule is
//! a pure function over plain values returning `Result<(), PolicyRejection>`,
//! so providers gate acks with the same table the tests pin down, and every
//! rejection is a typed value (code + display) rather than a log string.
//!
//! Geometry uses the fleet-wide flat-earth frame (111 111 m per degree of
//! latitude — byte-for-byte with the v2 Python and uas-fleet-node).

use serde::Serialize;

/// Field-safety default: maximum commanded range from home, metres.
pub const DEFAULT_MAX_RANGE_M: f64 = 300.0;
/// Field-safety default: minimum commanded AGL, metres (the ArduCopter
/// goto-gate floor; bench sims may lower it to 0.5).
pub const DEFAULT_MIN_AGL_M: f64 = 3.5;
/// Field-safety default: maximum commanded AGL, metres.
pub const DEFAULT_MAX_AGL_M: f64 = 20.0;

/// Metres per degree of latitude on the flat-earth frame used fleet-wide.
const EARTH_M_PER_DEG_LAT: f64 = 111_111.0;

/// Flat-earth distance in metres between two `(lat_deg, lon_deg)` points.
pub fn dist_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    let m_per_deg_lon =
        EARTH_M_PER_DEG_LAT * ((a.0 + b.0) / 2.0).to_radians().cos().max(1e-6);
    let dn = (a.0 - b.0) * EARTH_M_PER_DEG_LAT;
    let de = (a.1 - b.1) * m_per_deg_lon;
    dn.hypot(de)
}

/// Commandable AGL window (the v2 `min_agl`/`--max-agl-m` pair).
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct AglBounds {
    pub min_agl_m: f64,
    pub max_agl_m: f64,
}

impl Default for AglBounds {
    fn default() -> Self {
        Self {
            min_agl_m: DEFAULT_MIN_AGL_M,
            max_agl_m: DEFAULT_MAX_AGL_M,
        }
    }
}

/// A typed ack-gate rejection. `code()` is the stable machine-readable
/// identifier carried in [`crate::services::Ack::code`]; `Display` is the
/// operator-facing reason.
#[derive(Debug, Clone, PartialEq, Serialize, thiserror::Error)]
pub enum PolicyRejection {
    /// Range guard: a commanded point lies beyond `max_range_m` from home.
    #[error("target {distance_m:.1} m from home exceeds max range {max_range_m:.1} m")]
    OutOfRange { distance_m: f64, max_range_m: f64 },
    /// AGL guard: commanded altitude outside the commandable window.
    #[error("agl {agl_m:.1} m outside allowed {min_agl_m:.1}..={max_agl_m:.1} m")]
    AglOutOfBounds {
        agl_m: f64,
        min_agl_m: f64,
        max_agl_m: f64,
    },
    /// Busy guard: the vehicle is already occupied by a long-running task.
    #[error("vehicle busy with task '{task}'")]
    Busy { task: String },
    /// Shutdown authorization: confirm phrase did not equal the vehicle id.
    #[error("shutdown confirm phrase does not match the vehicle id")]
    ShutdownConfirmMismatch,
    /// Shutdown refused while the vehicle is armed.
    #[error("shutdown refused: vehicle is armed")]
    ShutdownWhileArmed,
    /// Shutdown refused while a task is running.
    #[error("shutdown refused: vehicle busy with task '{task}'")]
    ShutdownWhileBusy { task: String },
    /// Audio range guard: capture point beyond the microphone's useful range.
    #[error("audio target {distance_m:.1} m away exceeds audio range {audio_range_m:.1} m")]
    AudioOutOfRange {
        distance_m: f64,
        audio_range_m: f64,
    },
}

impl PolicyRejection {
    /// Stable machine-readable code for [`crate::services::Ack::code`].
    pub fn code(&self) -> &'static str {
        match self {
            Self::OutOfRange { .. } => "out-of-range",
            Self::AglOutOfBounds { .. } => "agl-out-of-bounds",
            Self::Busy { .. } => "busy",
            Self::ShutdownConfirmMismatch => "shutdown-confirm-mismatch",
            Self::ShutdownWhileArmed => "shutdown-while-armed",
            Self::ShutdownWhileBusy { .. } => "shutdown-while-busy",
            Self::AudioOutOfRange { .. } => "audio-out-of-range",
        }
    }
}

/// Range guard (`--max-range-m`, v2 default 300): every commanded point must
/// lie within `max_range_m` of `home`. With `home` unknown the guard cannot
/// evaluate and passes — exactly the v2 behavior, where the rail armed itself
/// once the backend captured home at the first ground arm.
pub fn range_guard(
    home: Option<(f64, f64)>,
    targets: &[(f64, f64)],
    max_range_m: f64,
) -> Result<(), PolicyRejection> {
    let Some(home) = home else { return Ok(()) };
    for target in targets {
        let distance_m = dist_m(home, *target);
        if distance_m > max_range_m {
            return Err(PolicyRejection::OutOfRange {
                distance_m,
                max_range_m,
            });
        }
    }
    Ok(())
}

/// AGL guard (v2: reject out-of-range AGL at ack; MAVLink floor 3.5 m,
/// `--max-agl-m` 20).
pub fn agl_guard(agl_m: f64, bounds: AglBounds) -> Result<(), PolicyRejection> {
    if !(bounds.min_agl_m..=bounds.max_agl_m).contains(&agl_m) {
        return Err(PolicyRejection::AglOutOfBounds {
            agl_m,
            min_agl_m: bounds.min_agl_m,
            max_agl_m: bounds.max_agl_m,
        });
    }
    Ok(())
}

/// Busy guard: a non-empty busy label means a long-running task owns the
/// vehicle. (RTL/Land/Hold deliberately do NOT call this — they abort.)
pub fn busy_guard(busy: &str) -> Result<(), PolicyRejection> {
    if busy.is_empty() {
        Ok(())
    } else {
        Err(PolicyRejection::Busy {
            task: busy.to_string(),
        })
    }
}

/// Shutdown double-authorization (v2 `system/shutdown`): the operator-typed
/// `confirm` must equal `vehicle_id`, and the vehicle must be neither armed
/// nor busy. v2 ran this table twice — at ack and again in the handler —
/// so providers should call it at both points.
pub fn shutdown_guard(
    confirm: &str,
    vehicle_id: &str,
    armed: bool,
    busy: &str,
) -> Result<(), PolicyRejection> {
    if confirm != vehicle_id {
        return Err(PolicyRejection::ShutdownConfirmMismatch);
    }
    if armed {
        return Err(PolicyRejection::ShutdownWhileArmed);
    }
    if !busy.is_empty() {
        return Err(PolicyRejection::ShutdownWhileBusy {
            task: busy.to_string(),
        });
    }
    Ok(())
}

/// Audio range guard (`--audio-range-m`): an audio capture point farther than
/// `audio_range_m` from the vehicle would record nothing useful — reject at
/// ack instead of flying a pointless capture.
pub fn audio_range_guard(
    vehicle: (f64, f64),
    target: (f64, f64),
    audio_range_m: f64,
) -> Result<(), PolicyRejection> {
    let distance_m = dist_m(vehicle, target);
    if distance_m > audio_range_m {
        return Err(PolicyRejection::AudioOutOfRange {
            distance_m,
            audio_range_m,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOME: (f64, f64) = (35.0, -90.0);

    /// A point `metres` north of `origin` on the flat-earth frame.
    fn north_of(origin: (f64, f64), metres: f64) -> (f64, f64) {
        (origin.0 + metres / EARTH_M_PER_DEG_LAT, origin.1)
    }

    #[test]
    fn range_guard_accepts_inside_and_rejects_beyond_300m() {
        assert_eq!(range_guard(Some(HOME), &[north_of(HOME, 299.0)], 300.0), Ok(()));
        let err = range_guard(
            Some(HOME),
            &[north_of(HOME, 10.0), north_of(HOME, 301.0)],
            300.0,
        )
        .unwrap_err();
        match err {
            PolicyRejection::OutOfRange {
                distance_m,
                max_range_m,
            } => {
                assert!((distance_m - 301.0).abs() < 0.5, "distance {distance_m}");
                assert_eq!(max_range_m, 300.0);
            }
            other => panic!("wrong rejection: {other:?}"),
        }
        assert_eq!(err.code(), "out-of-range");
    }

    #[test]
    fn range_guard_passes_when_home_unknown() {
        // The rail arms itself once home is captured (v2 behavior).
        assert_eq!(range_guard(None, &[north_of(HOME, 5000.0)], 300.0), Ok(()));
    }

    #[test]
    fn agl_guard_enforces_the_3_5_to_20_window() {
        let bounds = AglBounds::default();
        assert_eq!(agl_guard(3.5, bounds), Ok(()));
        assert_eq!(agl_guard(20.0, bounds), Ok(()));
        assert!(matches!(
            agl_guard(3.4, bounds),
            Err(PolicyRejection::AglOutOfBounds { .. })
        ));
        assert!(matches!(
            agl_guard(20.1, bounds),
            Err(PolicyRejection::AglOutOfBounds { .. })
        ));
        assert_eq!(agl_guard(3.4, bounds).unwrap_err().code(), "agl-out-of-bounds");
    }

    #[test]
    fn agl_guard_respects_bench_bounds() {
        let bench = AglBounds {
            min_agl_m: 0.5,
            max_agl_m: 20.0,
        };
        assert_eq!(agl_guard(1.0, bench), Ok(()));
    }

    #[test]
    fn busy_guard_rejects_only_nonempty_tasks() {
        assert_eq!(busy_guard(""), Ok(()));
        let err = busy_guard("investigate").unwrap_err();
        assert_eq!(
            err,
            PolicyRejection::Busy {
                task: "investigate".into()
            }
        );
        assert_eq!(err.code(), "busy");
    }

    #[test]
    fn shutdown_guard_checks_confirm_then_armed_then_busy() {
        // Wrong phrase loses first, regardless of the rest.
        assert_eq!(
            shutdown_guard("iuas-02", "iuas-01", true, "raster-search"),
            Err(PolicyRejection::ShutdownConfirmMismatch)
        );
        // Right phrase, armed.
        assert_eq!(
            shutdown_guard("iuas-01", "iuas-01", true, ""),
            Err(PolicyRejection::ShutdownWhileArmed)
        );
        // Right phrase, disarmed, busy.
        assert_eq!(
            shutdown_guard("iuas-01", "iuas-01", false, "investigate"),
            Err(PolicyRejection::ShutdownWhileBusy {
                task: "investigate".into()
            })
        );
        // Fully authorized.
        assert_eq!(shutdown_guard("iuas-01", "iuas-01", false, ""), Ok(()));
    }

    #[test]
    fn audio_range_guard_uses_flat_earth_distance() {
        let vehicle = HOME;
        assert_eq!(
            audio_range_guard(vehicle, north_of(vehicle, 25.0), 30.0),
            Ok(())
        );
        let err = audio_range_guard(vehicle, north_of(vehicle, 45.0), 30.0).unwrap_err();
        match err {
            PolicyRejection::AudioOutOfRange { distance_m, .. } => {
                assert!((distance_m - 45.0).abs() < 0.5)
            }
            other => panic!("wrong rejection: {other:?}"),
        }
        assert_eq!(err.code(), "audio-out-of-range");
    }

    #[test]
    fn dist_m_matches_the_fleet_flat_earth_frame() {
        // 1 degree of latitude = 111_111 m exactly in this frame.
        let d = dist_m((35.0, -90.0), (36.0, -90.0));
        assert!((d - 111_111.0).abs() < 1e-6);
    }
}
