//! MAVLink vehicle adapter for muas-flight. Lands at milestone M2.
//!
//! Non-negotiable behaviors to reimplement (evidence in
//! `docs/v3/surveys/uas-ipbrc.md` and `surveys/minimuas-v2.md`):
//! - HEARTBEAT filtered strictly by the autopilot component id; pinned
//!   target sys/comp (field bug: sysid flapping via mavp2p/GCS heartbeats).
//! - 1 Hz GCS heartbeat task so ArduCopter's FS_GCS failsafe holds quiet.
//! - Takeoff latch: no position targets until off the ground, and the
//!   3.5 m AGL command floor (ArduCopter silently drops lower gotos).
//! - All-AGL altitude frame pinned to `home_alt_m = 0` (2026-06-15 fix).
//! - `ensure_airborne`: force GUIDED pre-arm; ground check; climb check.
//! - Arm/mode-set retry state machines confirmed by heartbeat.
//! - Goto lead capped (~15 m) while an avoidance altitude bias is active.

/// The AGL command floor on real vehicles: ArduCopter silently drops goto
/// targets below ~3 m right after takeoff, so nothing below this is ever
/// commanded.
pub const MIN_AGL_M: f64 = 3.5;
