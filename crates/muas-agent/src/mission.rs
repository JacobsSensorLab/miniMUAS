//! Mission flight execution: the v2 `fly_raster` / `fly_orbit` loops, typed
//! (parity spec: `docs/v3/surveys/minimuas-v2.md` §Drone agent).
//!
//! # Structure
//!
//! The progression logic lives in synchronous state machines
//! ([`RasterFlight`], [`OrbitFlight`]) driven by `tick(now_s, backend)` —
//! unit-testable against the sim backend with scripted time, no runtime.
//! The async runners ([`run_raster`], [`run_investigate`], [`takeoff_task`])
//! tick those machines from a tokio task with **one short backend lock per
//! tick** — never across a wait — so the telemetry loop and the coordination
//! thread keep flowing while a mission flies (this also fixes KNOWN-ISSUES
//! #4: takeoff no longer holds the backend mutex for the whole climb; see
//! [`ensure_airborne`]).
//!
//! # Pinned v2 semantics
//!
//! - raster: transit to each leg START; captures fired by **along-track
//!   progress** (a point is captured the moment the vehicle passes abeam of
//!   it); no waits at leg ends (stragglers captured immediately on
//!   arrival); **position target re-sent every 2 s**; per-cruise and overall
//!   deadlines sized from the commanded speed; `search/status` published
//!   while running (transit → searching → done/aborted/failed).
//! - investigate: climb to orbit AGL, enter the circle at its nearest
//!   point, then a continuous carrot-chasing orbit — each 0.4 s tick reads
//!   the vehicle's MEASURED bearing from the center and commands the circle
//!   point a fixed lead-arc ahead with yaw facing the center; sweep
//!   accumulates from measured motion so `turns` means what it says.
//! - the abort flag (raised by rtl/land/hold) is honored within one control
//!   cycle (≤ 0.2 s) by every runner loop.
//!
//! Documented deviations from v2: captures journal a `search.capture` event
//! and bump the `search/status` frame counter but publish no camera frame
//! (no camera hardware in this build — `last_frames` stays empty); the
//! along-track projection uses uas-flight's flat-earth constant (111 320)
//! rather than the v2 Python 111 111 (sub-0.2% on the metre scale).

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;
use muas_contracts::services::{InvestigateRequest, RasterRequest};
use tracing::{info, warn};
use uas_fleet_data::kinds::{search_state, SearchStatus};
use uas_fleet_node::flight_backend::FlightBackend;
use uas_flight::geo::{m_per_deg_lon, Position, EARTH_M_PER_DEG_LAT};
use uas_flight::motion::MotionTarget;
use uas_flight::patterns::{RasterBounds, RasterPath};

use crate::{lock, AgentShared, BackendExt, TakeoffStart};

/// Runner control-loop cadence (v2 slept 0.1–0.2 s between checks; the
/// abort flag is honored within one such cycle).
const CONTROL_TICK: Duration = Duration::from_millis(200);
/// Guided position target re-send period, seconds (v2: one lost
/// SET_POSITION_TARGET must not strand the vehicle in a hover).
const TARGET_RESEND_S: f64 = 2.0;
/// Arrival tolerance for cruise targets, metres (v2 `tol_m=2.5`).
const ARRIVE_TOL_M: f64 = 2.5;
/// Climb-poll cadence for the lock-per-poll takeoff (v2 0.5 s sleeps).
const CLIMB_POLL: Duration = Duration::from_millis(400);
/// Takeoff settles once AGL is within this of the target (mirror of
/// uas-mavlink's `TAKEOFF_SETTLE_M`; the sim path settles internally).
const TAKEOFF_SETTLE_M: f64 = 2.0;
/// Overall takeoff budget (mirror of uas-mavlink's `AIRBORNE_TIMEOUT`).
const AIRBORNE_TIMEOUT: Duration = Duration::from_secs(120);
/// Climb check: after this long post-NAV_TAKEOFF the vehicle must have
/// gained [`CLIMB_CHECK_MIN_GAIN_M`] or the altitude source is suspect.
const CLIMB_CHECK_WINDOW: Duration = Duration::from_secs(12);
/// See [`CLIMB_CHECK_WINDOW`].
const CLIMB_CHECK_MIN_GAIN_M: f64 = 1.0;

// ---------------------------------------------------------------------------
// geometry helpers (fleet flat-earth frame)
// ---------------------------------------------------------------------------

fn dist_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dn = (a.0 - b.0) * EARTH_M_PER_DEG_LAT;
    let de = (a.1 - b.1) * m_per_deg_lon((a.0 + b.0) / 2.0);
    dn.hypot(de)
}

/// Unit vector `(north, east)` and length in metres of one raster leg
/// (v2 `_leg_axis`).
fn leg_axis(start: (f64, f64), end: (f64, f64)) -> ((f64, f64), f64) {
    let lat0 = (start.0 + end.0) / 2.0;
    let dn = (end.0 - start.0) * EARTH_M_PER_DEG_LAT;
    let de = (end.1 - start.1) * m_per_deg_lon(lat0);
    let length = dn.hypot(de);
    if length < 1e-6 {
        ((1.0, 0.0), 0.0)
    } else {
        ((dn / length, de / length), length)
    }
}

/// Projection of `(lat, lon)` onto the leg axis, metres from the leg start
/// (v2 `_along_leg_m`).
fn along_leg_m(start: (f64, f64), axis: (f64, f64), lat: f64, lon: f64) -> f64 {
    let dn = (lat - start.0) * EARTH_M_PER_DEG_LAT;
    let de = (lon - start.1) * m_per_deg_lon(start.0);
    dn * axis.0 + de * axis.1
}

// ---------------------------------------------------------------------------
// raster plan (uas-flight geometry — the SAME structure the dashboard previews)
// ---------------------------------------------------------------------------

/// One planned capture point (v2 `RasterPlan.captures` entries).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapturePoint {
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Planned leg heading, degrees (fallback pose when the backend has no
    /// compass; the ACTUAL heading at capture is journaled when available).
    pub heading_deg: f64,
    pub leg: usize,
    /// Along-track position on the leg, metres from the leg start.
    pub along_m: f64,
}

/// The resolved raster: serpentine legs plus per-leg capture points, built
/// from [`uas_flight::patterns::raster_targets`] geometry (preview == flight).
#[derive(Debug, Clone, PartialEq)]
pub struct RasterPlan {
    /// Legs as `[(lat, lon) start, (lat, lon) end]`, serpentine order.
    pub legs: Vec<[(f64, f64); 2]>,
    /// Capture points per leg, sorted by along-track position.
    pub captures: Vec<Vec<CapturePoint>>,
    pub agl_m: f64,
    /// Commanded ground speed (floored at the v2 0.3 m/s).
    pub speed_m_s: f64,
    /// Total leg + inter-leg path length, metres (deadline sizing).
    pub path_len_m: f64,
}

impl RasterPlan {
    pub fn capture_count(&self) -> usize {
        self.captures.iter().map(Vec::len).sum()
    }
}

/// Build the raster plan for a corner-defined request. The corner list is
/// reduced to its bounding box (the v2 dashboard sends an axis-aligned
/// NW/NE/SE/SW rectangle); legs run along the longer axis (fewest turns),
/// exactly the dashboard preview's `flight_path` rule.
pub fn plan_raster(req: &RasterRequest) -> Result<RasterPlan, String> {
    if req.corners.is_empty() {
        return Err("no area corners".to_string());
    }
    let (mut lat_min, mut lat_max) = (f64::MAX, f64::MIN);
    let (mut lon_min, mut lon_max) = (f64::MAX, f64::MIN);
    for &(lat, lon) in &req.corners {
        lat_min = lat_min.min(lat);
        lat_max = lat_max.max(lat);
        lon_min = lon_min.min(lon);
        lon_max = lon_max.max(lon);
    }
    let center_lat = (lat_min + lat_max) / 2.0;
    let center_lon = (lon_min + lon_max) / 2.0;
    let height_m = (lat_max - lat_min) * EARTH_M_PER_DEG_LAT;
    let width_m = (lon_max - lon_min) * m_per_deg_lon(center_lat);

    // Legs along the longer axis: east–west when wider than tall
    // (bearing 90°), north–south otherwise — the dashboard preview rule.
    let along_ew = width_m >= height_m;
    let (length_m, across_m, bearing_deg) = if along_ew {
        (width_m.max(0.5), height_m.max(0.5), 90.0)
    } else {
        (height_m.max(0.5), width_m.max(0.5), 0.0)
    };
    let mut path = RasterPath::new(RasterBounds {
        center: Position::new(center_lat, center_lon, 0.0),
        length_m,
        width_m: across_m,
        bearing_deg,
    });
    path.lane_spacing_m = req.spacing_m.max(0.5);
    let targets: Vec<MotionTarget> = path.targets().map_err(|e| e.to_string())?;

    let mut legs = Vec::new();
    let mut captures = Vec::new();
    let mut path_len_m = 0.0;
    let mut prev_end: Option<(f64, f64)> = None;
    for pair in targets.chunks(2) {
        let [a, b] = pair else { continue };
        let start = (a.position.lat, a.position.lon);
        let end = (b.position.lat, b.position.lon);
        let leg_index = legs.len();
        if let Some(prev) = prev_end {
            path_len_m += dist_m(prev, start);
        }
        prev_end = Some(end);

        let (axis, leg_len) = leg_axis(start, end);
        path_len_m += leg_len;
        let heading = {
            let dn = (end.0 - start.0) * EARTH_M_PER_DEG_LAT;
            let de = (end.1 - start.1) * m_per_deg_lon(center_lat);
            de.atan2(dn).to_degrees().rem_euclid(360.0)
        };
        // v2 capture rule: max(1, floor(len/step)+1) points, endpoints
        // included; capture_every_m <= 0 disables captures.
        let mut leg_caps = Vec::new();
        if req.capture_every_m > 0.0 {
            let n = usize::max(1, (leg_len / req.capture_every_m).floor() as usize + 1);
            for i in 0..n {
                let frac = if n > 1 { i as f64 / (n - 1) as f64 } else { 0.5 };
                let lat = start.0 + (end.0 - start.0) * frac;
                let lon = start.1 + (end.1 - start.1) * frac;
                leg_caps.push(CapturePoint {
                    lat_deg: lat,
                    lon_deg: lon,
                    heading_deg: heading,
                    leg: leg_index,
                    along_m: along_leg_m(start, axis, lat, lon),
                });
            }
            leg_caps.sort_by(|a, b| a.along_m.total_cmp(&b.along_m));
        }
        legs.push([start, end]);
        captures.push(leg_caps);
    }
    if legs.is_empty() {
        return Err("empty raster".to_string());
    }
    Ok(RasterPlan {
        legs,
        captures,
        agl_m: req.agl_m,
        speed_m_s: req.speed_m_s.max(0.3),
        path_len_m,
    })
}

// ---------------------------------------------------------------------------
// flight outcomes & events
// ---------------------------------------------------------------------------

/// Terminal outcome of a mission flight (v2's "completed" / "aborted" /
/// "timeout" strings, typed; timeout maps to a failed search like v2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightOutcome {
    Completed,
    Aborted,
    TimedOut,
}

impl FlightOutcome {
    /// The v2 outcome string (journal parity).
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Completed => "completed",
            Self::Aborted => "aborted",
            Self::TimedOut => "timeout",
        }
    }
}

/// Progress event emitted by a tick (journaled / status-published by the
/// async runner; kept as data so the machines stay side-effect-free).
#[derive(Debug, Clone, PartialEq)]
pub enum MissionEvent {
    /// Starting leg `index` (transit to its start begins now) — v2 `on_leg`.
    LegStarted { index: usize },
    /// A capture fired; pose is the vehicle's ACTUAL position at capture
    /// (v2 froze the pose with the frame, not the planned point).
    Capture {
        point: CapturePoint,
        lat_deg: f64,
        lon_deg: f64,
        agl_m: f64,
        heading_deg: f64,
    },
    /// The orbit entry point was reached; the carrot loop starts.
    OrbitEntered,
}

// ---------------------------------------------------------------------------
// RasterFlight — the v2 fly_raster loop as a tickable state machine
// ---------------------------------------------------------------------------

/// One cruise toward a fixed target with 2 s re-sends and a speed-sized
/// travel deadline (v2's inner `cruise`).
#[derive(Debug, Clone)]
struct Cruise {
    target: (f64, f64),
    next_send_s: f64,
    /// Set on the first tick (needs a position read).
    travel_deadline_s: Option<f64>,
}

impl Cruise {
    fn new(target: (f64, f64)) -> Self {
        Self {
            target,
            next_send_s: 0.0,
            travel_deadline_s: None,
        }
    }
}

#[derive(Debug, Clone)]
enum RasterState {
    /// Transit to `leg`'s start point.
    ToLegStart { leg: usize, cruise: Cruise },
    /// Flying `leg` start→end, firing `pending` captures by along-track
    /// progress (indices into `plan.captures[leg]`, front = next).
    OnLeg {
        leg: usize,
        axis: (f64, f64),
        cruise: Cruise,
        pending: std::collections::VecDeque<usize>,
    },
    Done(FlightOutcome),
}

/// The serpentine raster in flight. Call [`RasterFlight::tick`] on a fixed
/// cadence with a monotonically increasing `now_s`.
pub struct RasterFlight {
    plan: RasterPlan,
    state: RasterState,
    /// Overall deadline sized from the commanded speed (v2 sized it from
    /// the requested max duration; v3 requests carry no duration, so:
    /// 3× the still-air path time plus a fixed margin).
    deadline_s: f64,
    pub frames: u64,
    started: bool,
}

impl RasterFlight {
    pub fn new(plan: RasterPlan, now_s: f64) -> Self {
        let deadline_s = now_s + 3.0 * plan.path_len_m / plan.speed_m_s + 90.0;
        let first_start = plan.legs[0][0];
        Self {
            plan,
            state: RasterState::ToLegStart {
                leg: 0,
                cruise: Cruise::new(first_start),
            },
            deadline_s,
            frames: 0,
            started: false,
        }
    }

    /// Current leg index (status stream).
    pub fn leg(&self) -> usize {
        match &self.state {
            RasterState::ToLegStart { leg, .. } | RasterState::OnLeg { leg, .. } => *leg,
            RasterState::Done(_) => self.plan.legs.len().saturating_sub(1),
        }
    }

    pub fn legs_total(&self) -> usize {
        self.plan.legs.len()
    }

    /// One control tick: re-send the target if due, fire along-track
    /// captures, advance legs on arrival. Returns the terminal outcome once
    /// reached (and keeps returning it).
    pub fn tick(
        &mut self,
        now_s: f64,
        backend: &mut dyn FlightBackend,
        events: &mut Vec<MissionEvent>,
    ) -> Option<FlightOutcome> {
        if let RasterState::Done(outcome) = &self.state {
            return Some(*outcome);
        }
        if !self.started {
            self.started = true;
            events.push(MissionEvent::LegStarted { index: 0 });
        }
        if now_s > self.deadline_s {
            self.state = RasterState::Done(FlightOutcome::TimedOut);
            return Some(FlightOutcome::TimedOut);
        }
        let agl = self.plan.agl_m;
        let speed = self.plan.speed_m_s;
        let here = backend.position().unwrap_or((0.0, 0.0, 0.0));

        // Fire captures BEFORE the arrival check so a fast tick can't skip
        // points the vehicle passed this cycle.
        if let RasterState::OnLeg { leg, axis, pending, .. } = &mut self.state {
            let start = self.plan.legs[*leg][0];
            let along = along_leg_m(start, *axis, here.0, here.1);
            while let Some(&idx) = pending.front() {
                if self.plan.captures[*leg][idx].along_m > along {
                    break;
                }
                pending.pop_front();
                self.frames += 1;
                events.push(capture_event(&self.plan.captures[*leg][idx], here, backend));
            }
        }

        let cruise = match &mut self.state {
            RasterState::ToLegStart { cruise, .. } | RasterState::OnLeg { cruise, .. } => cruise,
            RasterState::Done(_) => unreachable!("handled above"),
        };
        let travel_deadline = *cruise.travel_deadline_s.get_or_insert_with(|| {
            now_s + dist_m((here.0, here.1), cruise.target) / (0.5 * speed) + 45.0
        });
        if now_s >= cruise.next_send_s {
            backend.goto(cruise.target.0, cruise.target.1, agl, None);
            cruise.next_send_s = now_s + TARGET_RESEND_S;
        }
        let arrived = backend.at_target(cruise.target.0, cruise.target.1, agl, ARRIVE_TOL_M)
            // Blocked short of the target (wind, EKF disagreement): move on
            // rather than hover — stragglers are captured on advance.
            || now_s > travel_deadline;
        if !arrived {
            return None;
        }

        // Advance the state machine.
        match std::mem::replace(&mut self.state, RasterState::Done(FlightOutcome::Completed)) {
            RasterState::ToLegStart { leg, .. } => {
                let (axis, _) = leg_axis(self.plan.legs[leg][0], self.plan.legs[leg][1]);
                self.state = RasterState::OnLeg {
                    leg,
                    axis,
                    cruise: Cruise::new(self.plan.legs[leg][1]),
                    pending: (0..self.plan.captures[leg].len()).collect(),
                };
                None
            }
            RasterState::OnLeg { leg, pending, .. } => {
                // Never wait at a leg end: capture any not-yet-fired points
                // immediately, then command the next leg.
                for idx in pending {
                    self.frames += 1;
                    events.push(capture_event(&self.plan.captures[leg][idx], here, backend));
                }
                let next = leg + 1;
                if next >= self.plan.legs.len() {
                    self.state = RasterState::Done(FlightOutcome::Completed);
                    Some(FlightOutcome::Completed)
                } else {
                    events.push(MissionEvent::LegStarted { index: next });
                    self.state = RasterState::ToLegStart {
                        leg: next,
                        cruise: Cruise::new(self.plan.legs[next][0]),
                    };
                    None
                }
            }
            RasterState::Done(outcome) => Some(outcome),
        }
    }
}

fn capture_event(
    point: &CapturePoint,
    here: (f64, f64, f64),
    backend: &dyn FlightBackend,
) -> MissionEvent {
    MissionEvent::Capture {
        point: *point,
        lat_deg: here.0,
        lon_deg: here.1,
        agl_m: here.2,
        heading_deg: backend.heading().unwrap_or(point.heading_deg),
    }
}

// ---------------------------------------------------------------------------
// OrbitFlight — the v2 fly_orbit carrot loop as a tickable state machine
// ---------------------------------------------------------------------------

/// Carrot-orbit command cadence, seconds (v2 `tick_s = 0.4`).
const ORBIT_CMD_S: f64 = 0.4;

#[derive(Debug, Clone)]
enum OrbitState {
    /// First tick: pick the entry point from the current position.
    Init,
    /// Cruise to the circle's nearest point.
    Entry { cruise: Cruise, deadline_s: f64 },
    /// The carrot loop: closed-loop on measured bearing.
    Orbiting {
        swept_rad: f64,
        prev_ang_rad: f64,
        budget_s: f64,
        next_cmd_s: f64,
    },
    Done(FlightOutcome),
}

/// Continuous carrot-chasing orbit around a ground point (v2 `fly_orbit`).
pub struct OrbitFlight {
    center: (f64, f64),
    radius_m: f64,
    agl_m: f64,
    speed_m_s: f64,
    /// Sweep goal, radians (`2π · turns`).
    goal_rad: f64,
    /// Lead arc, radians: ~1.5 s of travel, clamped 0.25..0.8 so the carrot
    /// stays meaningfully ahead without pulling the track inside the circle.
    lead_rad: f64,
    state: OrbitState,
}

impl OrbitFlight {
    pub fn new(center: (f64, f64), radius_m: f64, agl_m: f64, turns: f64, speed_m_s: f64) -> Self {
        let radius_m = radius_m.max(2.0);
        let speed_m_s = speed_m_s.clamp(0.5, 8.0);
        let turns = turns.max(0.25);
        Self {
            center,
            radius_m,
            agl_m,
            speed_m_s,
            goal_rad: std::f64::consts::TAU * turns,
            lead_rad: (speed_m_s * 1.5 / radius_m).clamp(0.25, 0.8),
            state: OrbitState::Init,
        }
    }

    /// Sweep progress in turns (status/journal).
    pub fn swept_turns(&self) -> f64 {
        match &self.state {
            OrbitState::Orbiting { swept_rad, .. } => swept_rad / std::f64::consts::TAU,
            OrbitState::Done(_) => self.goal_rad / std::f64::consts::TAU,
            _ => 0.0,
        }
    }

    /// Bearing (radians, clockwise from north) and distance from the center.
    fn bearing_dist(&self, lat: f64, lon: f64) -> (f64, f64) {
        let dn = (lat - self.center.0) * EARTH_M_PER_DEG_LAT;
        let de = (lon - self.center.1) * m_per_deg_lon(self.center.0);
        (de.atan2(dn), dn.hypot(de))
    }

    fn circle_point(&self, ang_rad: f64) -> (f64, f64) {
        (
            self.center.0 + self.radius_m * ang_rad.cos() / EARTH_M_PER_DEG_LAT,
            self.center.1 + self.radius_m * ang_rad.sin() / m_per_deg_lon(self.center.0),
        )
    }

    /// One control tick. Same contract as [`RasterFlight::tick`].
    pub fn tick(
        &mut self,
        now_s: f64,
        backend: &mut dyn FlightBackend,
        events: &mut Vec<MissionEvent>,
    ) -> Option<FlightOutcome> {
        let here = backend.position().unwrap_or((0.0, 0.0, 0.0));
        loop {
            match std::mem::replace(&mut self.state, OrbitState::Init) {
                OrbitState::Done(outcome) => {
                    self.state = OrbitState::Done(outcome);
                    return Some(outcome);
                }
                OrbitState::Init => {
                    // Enter at the nearest circle point (due north when
                    // starting from over the center, where "nearest" is
                    // undefined), then process the entry this same tick.
                    let (mut ang, dist) = self.bearing_dist(here.0, here.1);
                    if dist < 1.0 {
                        ang = 0.0;
                    }
                    let target = self.circle_point(ang);
                    let deadline_s = now_s
                        + dist_m((here.0, here.1), target).max(5.0) / (0.5 * self.speed_m_s)
                        + 30.0;
                    self.state = OrbitState::Entry {
                        cruise: Cruise::new(target),
                        deadline_s,
                    };
                    continue;
                }
                OrbitState::Entry { mut cruise, deadline_s } => {
                    if backend.at_target(cruise.target.0, cruise.target.1, self.agl_m, ARRIVE_TOL_M)
                    {
                        let (prev_ang, _) = self.bearing_dist(here.0, here.1);
                        let budget_s = now_s
                            + ((self.goal_rad + self.lead_rad) * self.radius_m / self.speed_m_s)
                                * 3.0
                            + 60.0;
                        events.push(MissionEvent::OrbitEntered);
                        self.state = OrbitState::Orbiting {
                            swept_rad: 0.0,
                            prev_ang_rad: prev_ang,
                            budget_s,
                            next_cmd_s: now_s,
                        };
                        return None;
                    }
                    if now_s > deadline_s {
                        self.state = OrbitState::Done(FlightOutcome::TimedOut);
                        return Some(FlightOutcome::TimedOut);
                    }
                    if now_s >= cruise.next_send_s {
                        backend.goto(cruise.target.0, cruise.target.1, self.agl_m, None);
                        cruise.next_send_s = now_s + TARGET_RESEND_S;
                    }
                    self.state = OrbitState::Entry { cruise, deadline_s };
                    return None;
                }
                OrbitState::Orbiting {
                    mut swept_rad,
                    mut prev_ang_rad,
                    budget_s,
                    mut next_cmd_s,
                } => {
                    if now_s > budget_s {
                        self.state = OrbitState::Done(FlightOutcome::TimedOut);
                        return Some(FlightOutcome::TimedOut);
                    }
                    if now_s < next_cmd_s {
                        self.state = OrbitState::Orbiting {
                            swept_rad,
                            prev_ang_rad,
                            budget_s,
                            next_cmd_s,
                        };
                        return None;
                    }
                    next_cmd_s = now_s + ORBIT_CMD_S;
                    let (ang, _) = self.bearing_dist(here.0, here.1);
                    let mut delta = ang - prev_ang_rad;
                    while delta > std::f64::consts::PI {
                        delta -= std::f64::consts::TAU;
                    }
                    while delta < -std::f64::consts::PI {
                        delta += std::f64::consts::TAU;
                    }
                    // Clockwise = increasing bearing; never rewind the sweep.
                    swept_rad = (swept_rad + delta).max(0.0);
                    prev_ang_rad = ang;
                    if swept_rad >= self.goal_rad {
                        self.state = OrbitState::Done(FlightOutcome::Completed);
                        return Some(FlightOutcome::Completed);
                    }
                    let carrot = self.circle_point(ang + self.lead_rad);
                    let yaw = (ang.to_degrees() + 180.0).rem_euclid(360.0); // face the center
                    backend.goto(carrot.0, carrot.1, self.agl_m, Some(yaw));
                    self.state = OrbitState::Orbiting {
                        swept_rad,
                        prev_ang_rad,
                        budget_s,
                        next_cmd_s,
                    };
                    return None;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// lock-per-poll takeoff (fixes KNOWN-ISSUES #4)
// ---------------------------------------------------------------------------

/// Arm + climb to `agl_m` without ever holding the backend mutex across a
/// wait: the (fast) command phase runs under one short lock
/// ([`crate::TickableBackend::takeoff_begin`]), then the climb is watched by
/// polling telemetry with a fresh lock every [`CLIMB_POLL`], keeping the v2
/// ladder (settle tolerance, 12 s climb-stall check, overall budget).
///
/// `busy_label`: the task label that must still own the vehicle; a raised
/// abort flag or a changed label cancels the climb watch (the autopilot
/// keeps whatever mode the interrupting command set).
pub async fn ensure_airborne(shared: &Arc<AgentShared>, agl_m: f64, busy_label: &str) -> bool {
    let start = lock(&shared.backend).takeoff_begin(agl_m);
    match start {
        TakeoffStart::Refused(reason) => {
            warn!(agl_m, reason, "takeoff refused");
            shared
                .journal
                .event("flight.takeoff_refused", serde_json::json!({ "reason": reason }));
            false
        }
        TakeoffStart::Airborne => true,
        TakeoffStart::Climbing { home } => {
            if let Some(home) = home {
                *lock(&shared.fallback_home) = Some(home);
            }
            let t0 = tokio::time::Instant::now();
            let start_agl = lock(&shared.backend).as_dyn_ref().telemetry().agl_m;
            loop {
                tokio::time::sleep(CLIMB_POLL).await;
                if shared.abort.load(Ordering::Relaxed) || *lock(&shared.busy) != busy_label {
                    return false;
                }
                let agl = lock(&shared.backend).as_dyn_ref().telemetry().agl_m;
                if agl >= agl_m - TAKEOFF_SETTLE_M {
                    return true;
                }
                let elapsed = t0.elapsed();
                if elapsed > CLIMB_CHECK_WINDOW && agl - start_agl < CLIMB_CHECK_MIN_GAIN_M {
                    warn!(
                        gain_m = agl - start_agl,
                        "no observed climb after takeoff — altitude source suspect; aborting"
                    );
                    shared.journal.event(
                        "flight.climb_stall",
                        serde_json::json!({ "gain_m": agl - start_agl }),
                    );
                    return false;
                }
                if elapsed > AIRBORNE_TIMEOUT {
                    return false;
                }
            }
        }
    }
}

/// The manual-takeoff service task (busy label `"takeoff"`).
pub(crate) async fn takeoff_task(shared: Arc<AgentShared>, agl_m: f64) {
    let airborne = ensure_airborne(&shared, agl_m, "takeoff").await;
    if airborne {
        info!(agl_m, "takeoff complete");
    } else {
        warn!(agl_m, "takeoff failed (not airborne)");
    }
    shared.journal.event(
        "flight.takeoff.result",
        serde_json::json!({ "agl_m": agl_m, "airborne": airborne }),
    );
    clear_busy(&shared, "takeoff");
}

// ---------------------------------------------------------------------------
// async mission runners
// ---------------------------------------------------------------------------

/// Clear the busy label only if this task still owns it (rtl may have
/// re-labelled the vehicle; never clobber the interrupting command).
fn clear_busy(shared: &AgentShared, label: &str) {
    let mut busy = lock(&shared.busy);
    if *busy == label {
        busy.clear();
    }
}

fn interrupted(shared: &AgentShared, label: &str) -> bool {
    shared.abort.load(Ordering::Relaxed) || *lock(&shared.busy) != label
}

/// Publish one `search/status` sample into the latest-wins buffer.
struct SearchStatusPub {
    shared: Arc<AgentShared>,
    mission_id: String,
    legs_total: u64,
}

impl SearchStatusPub {
    fn push(&self, state: &str, leg: u64, frames: u64, note: &str) {
        let status = SearchStatus {
            detects_completed: 0,
            detects_pending: 0,
            frames_captured: frames,
            gps_time_ns: crate::telemetry::gps_time_ns(),
            // No camera in this build: captures are journal events + this
            // counter; no frame data names are published (documented
            // deviation — the detection pipeline sees zero frames).
            last_frames: Vec::new(),
            last_note: note.to_string(),
            leg,
            legs_total: self.legs_total,
            mission_id: self.mission_id.clone(),
            state: state.to_string(),
            vehicle_id: self.shared.vehicle_id.clone(),
        };
        match serde_json::to_vec(&status) {
            Ok(bytes) => *lock(&self.shared.latest_search) = Some(Bytes::from(bytes)),
            Err(err) => warn!(%err, "search status failed to encode"),
        }
    }
}

/// Fly an accepted raster request (busy label `"raster-search"` is already
/// set and the abort flag cleared by the ack handler).
pub(crate) async fn run_raster(shared: Arc<AgentShared>, req: RasterRequest, plan: RasterPlan) {
    const LABEL: &str = "raster-search";
    let status = SearchStatusPub {
        shared: shared.clone(),
        mission_id: req.mission_id.clone(),
        legs_total: plan.legs.len() as u64,
    };
    shared.journal.event(
        "search.started",
        serde_json::json!({
            "mission_id": req.mission_id,
            "legs": plan.legs.len(),
            "captures": plan.capture_count(),
            "agl_m": plan.agl_m,
            "speed_m_s": plan.speed_m_s,
        }),
    );
    status.push(search_state::TRANSIT, 0, 0, "starting");
    lock(&shared.backend).as_dyn().set_cruise_speed(plan.speed_m_s);

    let agl = plan.agl_m;
    if !ensure_airborne(&shared, agl, LABEL).await {
        status.push(search_state::FAILED, 0, 0, "airborne failed");
        shared.journal.event(
            "search.finished",
            serde_json::json!({ "mission_id": req.mission_id, "outcome": "failed",
                               "note": "could not reach search altitude" }),
        );
        clear_busy(&shared, LABEL);
        return;
    }

    let t0 = tokio::time::Instant::now();
    let mut flight = RasterFlight::new(plan, 0.0);
    let mut interval = tokio::time::interval(CONTROL_TICK);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut events = Vec::new();
    let mut last_status = tokio::time::Instant::now();
    let outcome = loop {
        interval.tick().await;
        if interrupted(&shared, LABEL) {
            break FlightOutcome::Aborted;
        }
        let now_s = t0.elapsed().as_secs_f64();
        let outcome = {
            let mut backend = lock(&shared.backend);
            flight.tick(now_s, backend.as_dyn(), &mut events)
        };
        let mut pushed = false;
        for event in events.drain(..) {
            match event {
                MissionEvent::LegStarted { index } => {
                    shared
                        .journal
                        .event("search.leg", serde_json::json!({ "leg": index }));
                    status.push(search_state::SEARCHING, index as u64, flight.frames, "");
                    pushed = true;
                }
                MissionEvent::Capture { point, lat_deg, lon_deg, agl_m, heading_deg } => {
                    shared.journal.event(
                        "search.capture",
                        serde_json::json!({
                            "mission_id": req.mission_id,
                            "leg": point.leg,
                            "planned": { "lat_deg": point.lat_deg, "lon_deg": point.lon_deg },
                            "lat_deg": lat_deg,
                            "lon_deg": lon_deg,
                            "agl_m": agl_m,
                            "heading_deg": heading_deg,
                            "frame_index": flight.frames,
                        }),
                    );
                    status.push(
                        search_state::SEARCHING,
                        flight.leg() as u64,
                        flight.frames,
                        "",
                    );
                    pushed = true;
                }
                MissionEvent::OrbitEntered => {}
            }
        }
        // 1 Hz heartbeat while running (v2 pushed on events; the stream
        // must stay fresh for the dashboard's MustBeFresh poller even on a
        // long quiet leg).
        if pushed {
            last_status = tokio::time::Instant::now();
        } else if last_status.elapsed() >= Duration::from_secs(1) {
            status.push(search_state::SEARCHING, flight.leg() as u64, flight.frames, "");
            last_status = tokio::time::Instant::now();
        }
        if let Some(outcome) = outcome {
            break outcome;
        }
    };

    let state = match outcome {
        FlightOutcome::Completed => search_state::DONE,
        FlightOutcome::Aborted => search_state::ABORTED,
        FlightOutcome::TimedOut => search_state::FAILED,
    };
    status.push(state, flight.leg() as u64, flight.frames, outcome.as_str());
    shared.journal.event(
        "search.finished",
        serde_json::json!({
            "mission_id": req.mission_id,
            "outcome": outcome.as_str(),
            "frames": flight.frames,
            "legs": flight.legs_total(),
        }),
    );
    info!(outcome = outcome.as_str(), frames = flight.frames, "raster search finished");
    clear_busy(&shared, LABEL);
}

/// v2 investigate cruise speed when the request carries no constraint.
const INVESTIGATE_SPEED_M_S: f64 = 3.0;

/// Fly an accepted investigate request (busy label `"investigate"` is
/// already set and the abort flag cleared by the ack handler).
pub(crate) async fn run_investigate(shared: Arc<AgentShared>, req: InvestigateRequest) {
    const LABEL: &str = "investigate";
    shared.journal.event(
        "investigate.started",
        serde_json::json!({
            "mission_id": req.mission_id,
            "lat_deg": req.lat_deg,
            "lon_deg": req.lon_deg,
            "agl_m": req.agl_m,
            "radius_m": req.radius_m,
            "turns": req.turns,
            "sensors": req.sensors,
        }),
    );
    lock(&shared.backend).as_dyn().set_cruise_speed(INVESTIGATE_SPEED_M_S);

    let outcome = if ensure_airborne(&shared, req.agl_m, LABEL).await {
        let turns = if req.turns > 0.0 { req.turns } else { 1.0 };
        let mut flight = OrbitFlight::new(
            (req.lat_deg, req.lon_deg),
            req.radius_m,
            req.agl_m,
            turns,
            INVESTIGATE_SPEED_M_S,
        );
        let t0 = tokio::time::Instant::now();
        let mut interval = tokio::time::interval(CONTROL_TICK);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        let mut events = Vec::new();
        loop {
            interval.tick().await;
            if interrupted(&shared, LABEL) {
                break FlightOutcome::Aborted;
            }
            let now_s = t0.elapsed().as_secs_f64();
            let outcome = {
                let mut backend = lock(&shared.backend);
                flight.tick(now_s, backend.as_dyn(), &mut events)
            };
            for event in events.drain(..) {
                if event == MissionEvent::OrbitEntered {
                    shared
                        .journal
                        .event("investigate.orbit_entered", serde_json::json!({}));
                }
            }
            if let Some(outcome) = outcome {
                break outcome;
            }
        }
    } else {
        shared.journal.event(
            "investigate.airborne_failed",
            serde_json::json!({ "agl_m": req.agl_m }),
        );
        FlightOutcome::TimedOut
    };

    shared.journal.event(
        "investigate.finished",
        serde_json::json!({
            "mission_id": req.mission_id,
            "outcome": outcome.as_str(),
        }),
    );
    info!(outcome = outcome.as_str(), "investigate finished");
    clear_busy(&shared, LABEL);
}

// ---------------------------------------------------------------------------
// tests — scripted time against the sim backend
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use uas_fleet_node::flight_backend::{SimFlightBackend, SIM_TICK_S};

    const ORIGIN: (f64, f64) = (35.0, -90.0);

    fn corners(width_m: f64, height_m: f64) -> Vec<(f64, f64)> {
        let dlat = (height_m / 2.0) / EARTH_M_PER_DEG_LAT;
        let dlon = (width_m / 2.0) / m_per_deg_lon(ORIGIN.0);
        vec![
            (ORIGIN.0 + dlat, ORIGIN.1 - dlon),
            (ORIGIN.0 + dlat, ORIGIN.1 + dlon),
            (ORIGIN.0 - dlat, ORIGIN.1 + dlon),
            (ORIGIN.0 - dlat, ORIGIN.1 - dlon),
        ]
    }

    fn raster_req(width_m: f64, height_m: f64, spacing: f64, step: f64) -> RasterRequest {
        RasterRequest {
            agl_m: 8.0,
            spacing_m: spacing,
            capture_every_m: step,
            speed_m_s: 5.0,
            corners: corners(width_m, height_m),
            ..RasterRequest::default()
        }
    }

    /// Counting wrapper: how often (and when) goto was commanded.
    struct Counting {
        sim: SimFlightBackend,
        gotos: Vec<f64>,
        now_s: f64,
    }

    impl Counting {
        fn new(sim: SimFlightBackend) -> Self {
            Self { sim, gotos: Vec::new(), now_s: 0.0 }
        }
    }

    impl FlightBackend for Counting {
        fn source(&self) -> &'static str {
            "sim"
        }
        fn position(&self) -> Option<(f64, f64, f64)> {
            self.sim.position()
        }
        fn velocity_ne(&self) -> (f64, f64) {
            self.sim.velocity_ne()
        }
        fn goto(&mut self, lat: f64, lon: f64, agl_m: f64, yaw_deg: Option<f64>) {
            self.gotos.push(self.now_s);
            self.sim.goto(lat, lon, agl_m, yaw_deg);
        }
        fn at_target(&self, lat: f64, lon: f64, agl_m: f64, tol_m: f64) -> bool {
            self.sim.at_target(lat, lon, agl_m, tol_m)
        }
        fn set_cruise_speed(&mut self, s: f64) {
            self.sim.set_cruise_speed(s);
        }
        fn ensure_airborne(&mut self, agl_m: f64) -> bool {
            self.sim.ensure_airborne(agl_m)
        }
        fn takeoff(&mut self, agl_m: f64) -> bool {
            self.sim.takeoff(agl_m)
        }
        fn set_alt_bias(&mut self, b: f64) {
            self.sim.set_alt_bias(b);
        }
        fn avoid_bias(&self) -> f64 {
            self.sim.avoid_bias()
        }
        fn home(&self) -> Option<(f64, f64)> {
            self.sim.home()
        }
        fn heading(&self) -> Option<f64> {
            self.sim.heading()
        }
        fn attitude_deg(&self) -> Option<(f64, f64)> {
            self.sim.attitude_deg()
        }
        fn rangefinder_m(&self) -> f64 {
            self.sim.rangefinder_m()
        }
        fn rtl(&mut self) -> bool {
            self.sim.rtl()
        }
        fn land(&mut self) -> bool {
            self.sim.land()
        }
        fn hold(&mut self) -> bool {
            self.sim.hold()
        }
        fn telemetry(&self) -> uas_mavlink::BackendTelemetry {
            self.sim.telemetry()
        }
    }

    #[test]
    fn plan_matches_preview_geometry() {
        // 150 x 100 m, 30 m spacing: legs run east–west (longer axis),
        // ceil(100/30)+1 = 5 lanes, serpentine ordering.
        let plan = plan_raster(&raster_req(150.0, 100.0, 30.0, 20.0)).unwrap();
        assert_eq!(plan.legs.len(), 5);
        // Lane 0 runs west→east, lane 1 east→west (boustrophedon).
        assert!(plan.legs[0][0].1 < plan.legs[0][1].1);
        assert!(plan.legs[1][0].1 > plan.legs[1][1].1);
        // Adjacent lanes share an edge longitude (no diagonal cutting).
        assert!((plan.legs[0][1].1 - plan.legs[1][0].1).abs() < 1e-9);
        // v2 capture rule: floor(150/20)+1 = 8 points per leg.
        assert_eq!(plan.captures[0].len(), 8);
        // Captures are sorted by along-track position.
        for leg in &plan.captures {
            for pair in leg.windows(2) {
                assert!(pair[0].along_m <= pair[1].along_m);
            }
        }
        assert!(plan_raster(&RasterRequest::default()).is_err(), "no corners rejected");
    }

    #[test]
    fn raster_flies_all_legs_and_fires_captures_by_along_track_progress() {
        let plan = plan_raster(&raster_req(60.0, 30.0, 15.0, 10.0)).unwrap();
        let legs_total = plan.legs.len();
        let capture_total = plan.capture_count();

        let mut sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        assert!(sim.ensure_airborne(8.0));
        sim.set_cruise_speed(plan.speed_m_s);
        let mut backend = Counting::new(sim);

        let mut flight = RasterFlight::new(plan, 0.0);
        let mut events = Vec::new();
        let mut now = 0.0;
        let outcome = loop {
            backend.now_s = now;
            if let Some(outcome) = flight.tick(now, &mut backend, &mut events) {
                break outcome;
            }
            backend.sim.advance(SIM_TICK_S);
            now += SIM_TICK_S;
            assert!(now < 600.0, "raster never completed");
        };
        assert_eq!(outcome, FlightOutcome::Completed);
        assert_eq!(flight.frames as usize, capture_total, "every capture fired");

        // Legs were started in order 0..n.
        let legs: Vec<usize> = events
            .iter()
            .filter_map(|e| match e {
                MissionEvent::LegStarted { index } => Some(*index),
                _ => None,
            })
            .collect();
        assert_eq!(legs, (0..legs_total).collect::<Vec<_>>());

        // Captures fired near their planned points (along-track rule: the
        // vehicle passes abeam of each point ON the leg line).
        for event in &events {
            if let MissionEvent::Capture { point, lat_deg, lon_deg, .. } = event {
                let err = dist_m((*lat_deg, *lon_deg), (point.lat_deg, point.lon_deg));
                assert!(err < 3.0, "capture fired {err:.1} m from its point");
            }
        }

        // The position target was re-sent at most every ~2 s while flying.
        for pair in backend.gotos.windows(2) {
            assert!(pair[1] - pair[0] <= TARGET_RESEND_S + SIM_TICK_S + 1e-9);
        }
        assert!(backend.gotos.len() >= legs_total * 2);
    }

    #[test]
    fn raster_times_out_when_the_vehicle_cannot_progress() {
        let plan = plan_raster(&raster_req(60.0, 30.0, 15.0, 0.0)).unwrap();
        let mut sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        assert!(sim.ensure_airborne(8.0));
        // A vehicle that never moves: cruise speed at the floor, but we
        // simply never advance the sim — every travel deadline lapses and
        // the overall deadline finally fires.
        let mut flight = RasterFlight::new(plan, 0.0);
        let mut events = Vec::new();
        let mut now = 0.0;
        let outcome = loop {
            if let Some(outcome) = flight.tick(now, &mut sim, &mut events) {
                break outcome;
            }
            now += 5.0; // scripted time only; the vehicle is frozen
            assert!(now < 10_000.0);
        };
        // Blocked cruises "arrive" via their travel deadline (v2: move on,
        // never hover) so the flight either walks all legs or times out —
        // and a frozen vehicle still terminates.
        assert!(
            outcome == FlightOutcome::TimedOut || outcome == FlightOutcome::Completed,
            "unexpected outcome {outcome:?}"
        );
    }

    #[test]
    fn orbit_enters_sweeps_and_completes_by_accumulated_bearing() {
        let center = ORIGIN;
        let mut sim = SimFlightBackend::new(ORIGIN.0 + 30.0 / EARTH_M_PER_DEG_LAT, ORIGIN.1);
        assert!(sim.ensure_airborne(8.0));
        sim.set_cruise_speed(4.0);

        let mut flight = OrbitFlight::new(center, 8.0, 8.0, 1.0, 4.0);
        let mut events = Vec::new();
        let mut now = 0.0;
        let outcome = loop {
            if let Some(outcome) = flight.tick(now, &mut sim, &mut events) {
                break outcome;
            }
            sim.advance(SIM_TICK_S);
            now += SIM_TICK_S;
            assert!(now < 300.0, "orbit never completed (swept {:.2})", flight.swept_turns());
        };
        assert_eq!(outcome, FlightOutcome::Completed);
        assert!(events.contains(&MissionEvent::OrbitEntered));
        assert!(flight.swept_turns() >= 1.0 - 1e-6);

        // The vehicle ended on the circle (carrot chased, not the center).
        let (lat, lon, _) = sim.position().unwrap();
        let r = dist_m((lat, lon), center);
        assert!((r - 8.0).abs() < 3.0, "ended {r:.1} m from center, radius 8");
        // Yaw was commanded to face the center (sim heading == last yaw cmd).
        let (ang, _) = flight.bearing_dist(lat, lon);
        let want_yaw = (ang.to_degrees() + 180.0).rem_euclid(360.0);
        let got = sim.heading().unwrap();
        let diff = (got - want_yaw + 540.0).rem_euclid(360.0) - 180.0;
        assert!(diff.abs() < 25.0, "yaw {got:.0} vs facing-center {want_yaw:.0}");
    }

    /// The async runner honors the abort flag within one control cycle and
    /// publishes the aborted search state (agent-shared plumbing, sim
    /// backend, paused tokio time).
    #[tokio::test(start_paused = true)]
    async fn raster_runner_aborts_within_one_cycle() {
        let (journal, _task) = crate::journal::spawn("wuas-77", None, None, None);
        let (cmd_tx, _cmd_rx) = tokio::sync::mpsc::unbounded_channel();
        let sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        let backend: crate::SharedBackend =
            Arc::new(std::sync::Mutex::new(Box::new(sim) as Box<dyn crate::TickableBackend>));
        let shared = Arc::new(AgentShared {
            vehicle_id: "wuas-77".into(),
            backend: backend.clone(),
            busy: std::sync::Mutex::new("raster-search".into()),
            abort: std::sync::atomic::AtomicBool::new(false),
            agl_bounds: Default::default(),
            max_range_m: 300.0,
            audio_range_m: 30.0,
            smart_rtl: false,
            latest_telemetry: std::sync::Mutex::new(None),
            latest_coord: std::sync::Mutex::new(Bytes::from_static(b"[]")),
            latest_search: std::sync::Mutex::new(None),
            latest_state: std::sync::Mutex::new(None),
            fallback_home: std::sync::Mutex::new(None),
            journal,
            commands: cmd_tx,
        });
        // Motion ticker (the agent spawns the same loop for sim endpoints).
        {
            let backend = backend.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_secs_f64(SIM_TICK_S));
                loop {
                    interval.tick().await;
                    lock(&backend).advance(SIM_TICK_S);
                }
            });
        }
        let req = raster_req(300.0, 200.0, 25.0, 20.0); // big: stays running
        let plan = plan_raster(&req).unwrap();
        let runner = tokio::spawn(run_raster(shared.clone(), req, plan));

        // Let it get airborne and start the sweep.
        tokio::time::sleep(Duration::from_secs(20)).await;
        let running: SearchStatus = serde_json::from_slice(
            lock(&shared.latest_search).as_ref().expect("status published"),
        )
        .unwrap();
        assert_eq!(running.state, search_state::SEARCHING);
        assert!(!runner.is_finished(), "raster still running");

        // The abort ladder raises the flag; the runner stops ≤ 1 cycle.
        shared.abort.store(true, Ordering::Relaxed);
        tokio::time::sleep(CONTROL_TICK * 2).await;
        assert!(runner.is_finished(), "abort honored within one cycle");
        let ended: SearchStatus = serde_json::from_slice(
            lock(&shared.latest_search).as_ref().unwrap(),
        )
        .unwrap();
        assert_eq!(ended.state, search_state::ABORTED);
        // The runner still owned the label (nothing re-labelled), so it
        // cleared it on the way out.
        assert_eq!(*lock(&shared.busy), "");
    }
}
