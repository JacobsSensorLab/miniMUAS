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
use muas_contracts::services::{investigate_pattern, InvestigateRequest, RasterRequest};
use tracing::{info, warn};
use uas_fleet_data::kinds::{search_state, SearchStatus};
use uas_fleet_node::flight_backend::FlightBackend;
use uas_flight::geo::{m_per_deg_lon, Position, EARTH_M_PER_DEG_LAT};
use uas_flight::motion::MotionTarget;
use uas_flight::patterns::{RasterBounds, RasterPath};

use crate::{lock, AgentShared, BackendExt, IdlePolicy, TakeoffStart};

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

    /// The remainder of this plan after a split: legs from `start_leg` on,
    /// with the first `skip_captures` capture points of `start_leg` (the
    /// ones the parent already fired) removed. Capture `leg` indices are
    /// remapped to the trimmed plan; `along_m` stays in the (unchanged) leg
    /// frame, so the along-track firing rule is untouched. `None` when
    /// nothing is left.
    ///
    /// Earlier legs need no capture bookkeeping: the raster never leaves a
    /// leg with captures unfired ("never wait at a leg end" fires
    /// stragglers on advance), so a split's remaining work is always
    /// (whole legs after the current one) + (uncaptured tail of it).
    pub fn remainder(&self, start_leg: usize, skip_captures: usize) -> Option<RasterPlan> {
        if start_leg >= self.legs.len() {
            return None;
        }
        let legs: Vec<[(f64, f64); 2]> = self.legs[start_leg..].to_vec();
        let mut captures: Vec<Vec<CapturePoint>> = self.captures[start_leg..]
            .iter()
            .map(|leg| {
                leg.iter()
                    .map(|c| CapturePoint { leg: c.leg - start_leg, ..*c })
                    .collect()
            })
            .collect();
        let skip = skip_captures.min(captures[0].len());
        captures[0].drain(..skip);
        let mut path_len_m = 0.0;
        let mut prev_end: Option<(f64, f64)> = None;
        for leg in &legs {
            if let Some(prev) = prev_end {
                path_len_m += dist_m(prev, leg[0]);
            }
            path_len_m += leg_axis(leg[0], leg[1]).1;
            prev_end = Some(leg[1]);
        }
        Some(RasterPlan {
            legs,
            captures,
            agl_m: self.agl_m,
            speed_m_s: self.speed_m_s,
            path_len_m,
        })
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

    /// Split/resume snapshot: `(first leg still to fly, captures already
    /// fired on it)` — indices into THIS flight's plan. Exact at any point
    /// between ticks (captures only fire inside [`RasterFlight::tick`]).
    pub fn resume_state(&self) -> (usize, usize) {
        match &self.state {
            RasterState::ToLegStart { leg, .. } => (*leg, 0),
            RasterState::OnLeg { leg, pending, .. } => {
                (*leg, self.plan.captures[*leg].len() - pending.len())
            }
            RasterState::Done(_) => (self.plan.legs.len(), 0),
        }
    }

    /// Remaining path length from `here`, metres (progress/ETA).
    pub fn remaining_path_m(&self, here: (f64, f64)) -> f64 {
        let (leg, to_leg_start) = match &self.state {
            RasterState::ToLegStart { leg, .. } => (*leg, true),
            RasterState::OnLeg { leg, .. } => (*leg, false),
            RasterState::Done(_) => return 0.0,
        };
        let mut remaining = if to_leg_start {
            dist_m(here, self.plan.legs[leg][0])
                + leg_axis(self.plan.legs[leg][0], self.plan.legs[leg][1]).1
        } else {
            dist_m(here, self.plan.legs[leg][1])
        };
        let mut prev_end = self.plan.legs[leg][1];
        for next in &self.plan.legs[leg + 1..] {
            remaining += dist_m(prev_end, next[0]) + leg_axis(next[0], next[1]).1;
            prev_end = next[1];
        }
        remaining
    }

    /// `(percent complete, eta seconds)` from remaining path ÷ commanded
    /// speed (the queue-stream progress numbers).
    pub fn progress(&self, here: (f64, f64)) -> (f64, f64) {
        let remaining = self.remaining_path_m(here);
        let pct = if self.plan.path_len_m > 1e-6 {
            (100.0 * (1.0 - remaining / self.plan.path_len_m)).clamp(0.0, 100.0)
        } else {
            100.0
        };
        (pct, remaining / self.plan.speed_m_s)
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
// WaypointFlight — a cruise sequence (the flyover profile in flight)
// ---------------------------------------------------------------------------

/// A fixed waypoint sequence flown with the v2 cruise discipline: 2 s
/// target re-sends, arrival tolerance, per-leg travel deadlines that "move
/// on" when blocked (never hover a dead leg), and an overall speed-sized
/// budget. Used for the acoustic flyover profile — the geometry comes from
/// [`uas_flight::patterns::flyover_targets`].
pub struct WaypointFlight {
    /// `(lat_deg, lon_deg, agl_m)` in flight order.
    points: Vec<(f64, f64, f64)>,
    idx: usize,
    next_send_s: f64,
    leg_deadline_s: Option<f64>,
    speed_m_s: f64,
    deadline_s: f64,
    done: Option<FlightOutcome>,
}

impl WaypointFlight {
    pub fn new(points: Vec<(f64, f64, f64)>, speed_m_s: f64, now_s: f64) -> Self {
        let speed_m_s = speed_m_s.max(0.3);
        let mut path_len = 0.0;
        for pair in points.windows(2) {
            path_len += dist_m((pair[0].0, pair[0].1), (pair[1].0, pair[1].1));
        }
        Self {
            points,
            idx: 0,
            next_send_s: 0.0,
            leg_deadline_s: None,
            speed_m_s,
            deadline_s: now_s + 3.0 * path_len / speed_m_s + 90.0,
            done: None,
        }
    }

    /// `(next_waypoint_index, total)` — progress for status/journal.
    pub fn progress(&self) -> (usize, usize) {
        (self.idx, self.points.len())
    }

    /// One control tick; same contract as [`RasterFlight::tick`].
    pub fn tick(&mut self, now_s: f64, backend: &mut dyn FlightBackend) -> Option<FlightOutcome> {
        if let Some(outcome) = self.done {
            return Some(outcome);
        }
        if self.idx >= self.points.len() {
            self.done = Some(FlightOutcome::Completed);
            return self.done;
        }
        if now_s > self.deadline_s {
            self.done = Some(FlightOutcome::TimedOut);
            return self.done;
        }
        let (lat, lon, agl) = self.points[self.idx];
        let here = backend.position().unwrap_or((0.0, 0.0, 0.0));
        let leg_deadline = *self.leg_deadline_s.get_or_insert_with(|| {
            now_s + dist_m((here.0, here.1), (lat, lon)) / (0.5 * self.speed_m_s) + 45.0
        });
        if now_s >= self.next_send_s {
            backend.goto(lat, lon, agl, None);
            self.next_send_s = now_s + TARGET_RESEND_S;
        }
        if backend.at_target(lat, lon, agl, ARRIVE_TOL_M) || now_s > leg_deadline {
            self.idx += 1;
            self.leg_deadline_s = None;
            self.next_send_s = 0.0;
            if self.idx >= self.points.len() {
                self.done = Some(FlightOutcome::Completed);
                return self.done;
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// investigate pattern selection (ROUND-3: dispatch-by-sensor)
// ---------------------------------------------------------------------------

/// Which flight geometry an investigate request flies. Explicit `orbit` /
/// `flyover` are honored; `auto` (or absent — older callers) selects the
/// acoustic flyover for audio-only jobs on an audio-capable vehicle, and
/// the carrot orbit for everything else (camera keeps the orbit).
pub fn select_investigate_pattern(req: &InvestigateRequest, has_audio: bool) -> &'static str {
    match req.pattern.as_str() {
        investigate_pattern::ORBIT => investigate_pattern::ORBIT,
        investigate_pattern::FLYOVER => investigate_pattern::FLYOVER,
        _ => {
            let audio_only =
                !req.sensors.is_empty() && req.sensors.iter().all(|s| s == "audio");
            if audio_only && has_audio {
                investigate_pattern::FLYOVER
            } else {
                investigate_pattern::ORBIT
            }
        }
    }
}

/// Compass bearing (degrees) from `from` toward `to`; 0 when co-located
/// (the flyover's approach axis when the vehicle is already overhead).
fn bearing_deg(from: (f64, f64), to: (f64, f64)) -> f64 {
    let dn = (to.0 - from.0) * EARTH_M_PER_DEG_LAT;
    let de = (to.1 - from.1) * m_per_deg_lon(from.0);
    if dn.hypot(de) < 1.0 {
        0.0
    } else {
        de.atan2(dn).to_degrees().rem_euclid(360.0)
    }
}

/// Build the flyover waypoint list for an investigate request: approach
/// along the vehicle's natural inbound bearing at the requested AGL, dip to
/// the commandable floor (the closest legal pass for an omnidirectional
/// mic), `turns` passes rotating cross-axis.
fn flyover_points(
    shared: &AgentShared,
    req: &InvestigateRequest,
) -> Result<Vec<(f64, f64, f64)>, String> {
    let here = lock(&shared.backend)
        .as_dyn_ref()
        .position()
        .unwrap_or((req.lat_deg, req.lon_deg, 0.0));
    let approach = bearing_deg((here.0, here.1), (req.lat_deg, req.lon_deg));
    let dip_agl = shared.agl_bounds.min_agl_m;
    let passes = (req.turns.round() as usize).max(1);
    let targets = uas_flight::patterns::flyover_targets(
        Position::new(req.lat_deg, req.lon_deg, 0.0),
        approach,
        req.agl_m,
        dip_agl,
        req.radius_m.max(2.0),
        passes,
    )
    .map_err(|e| e.to_string())?;
    Ok(targets
        .into_iter()
        .map(|t| (t.position.lat, t.position.lon, t.position.alt))
        .collect())
}

// ---------------------------------------------------------------------------
// mission clock (freezes during sensor-override detours)
// ---------------------------------------------------------------------------

/// Mission-time clock: while a sensor-override detour owns the vehicle the
/// clock freezes, so cruise deadlines and re-send schedules do not burn
/// down against a suspended mission (the resumed cruise re-issues its
/// target within one re-send period — "re-issue the pre-empted target").
struct MissionClock {
    last: tokio::time::Instant,
    elapsed_s: f64,
}

impl MissionClock {
    fn start() -> Self {
        Self {
            last: tokio::time::Instant::now(),
            elapsed_s: 0.0,
        }
    }

    /// Advance and return mission time; a paused interval contributes 0.
    fn tick(&mut self, paused: bool) -> f64 {
        let now = tokio::time::Instant::now();
        if !paused {
            self.elapsed_s += (now - self.last).as_secs_f64();
        }
        self.last = now;
        self.elapsed_s
    }
}

// ---------------------------------------------------------------------------
// post-task idle policy (ROUND-3 §1)
// ---------------------------------------------------------------------------

/// Poll cadence while an `rtl-after` timer is armed.
const IDLE_POLL: Duration = Duration::from_millis(250);

/// Apply the configured post-task idle policy after a mission task ran to
/// its natural end (`completed`/`timeout` — an aborted task means another
/// command owns the vehicle). Called with the busy label already released;
/// re-checks idleness so a task acked between completion and this call
/// wins. Every branch journals its decision (`idle.policy`).
pub(crate) fn apply_idle_policy(shared: &Arc<AgentShared>, after_task: &'static str) {
    if shared.abort.load(Ordering::Relaxed) || !lock(&shared.busy).is_empty() {
        return; // something else already owns the vehicle
    }
    match shared.idle_policy {
        IdlePolicy::Hold => {
            // The pre-round-3 behavior, now an explicit journaled decision.
            shared.journal.event(
                "idle.policy",
                serde_json::json!({
                    "policy": "hold", "after_task": after_task, "action": "hover",
                }),
            );
        }
        IdlePolicy::SlotHold => match shared.slot_agl_m {
            Some(slot_agl_m) => {
                {
                    let mut backend = lock(&shared.backend);
                    if let Some((lat, lon, _)) = backend.as_dyn_ref().position() {
                        backend.as_dyn().goto(lat, lon, slot_agl_m, None);
                    }
                }
                shared.journal.event(
                    "idle.policy",
                    serde_json::json!({
                        "policy": "slot-hold", "after_task": after_task,
                        "action": "climb-to-slot", "slot_agl_m": slot_agl_m,
                    }),
                );
            }
            None => {
                shared.journal.event(
                    "idle.policy",
                    serde_json::json!({
                        "policy": "slot-hold", "after_task": after_task,
                        "action": "hover", "note": "no fleet slot; holding",
                    }),
                );
            }
        },
        IdlePolicy::RtlAfter(after_s) => {
            shared.journal.event(
                "idle.policy",
                serde_json::json!({
                    "policy": "rtl-after", "after_task": after_task,
                    "action": "armed", "after_s": after_s,
                }),
            );
            let shared = shared.clone();
            tokio::spawn(async move {
                let deadline =
                    tokio::time::Instant::now() + Duration::from_secs_f64(after_s);
                loop {
                    tokio::select! {
                        () = shared.cancel.cancelled() => return,
                        _ = tokio::time::sleep(IDLE_POLL) => {}
                    }
                    if shared.abort.load(Ordering::Relaxed) || !lock(&shared.busy).is_empty() {
                        shared.journal.event(
                            "idle.policy",
                            serde_json::json!({ "policy": "rtl-after", "action": "cancelled" }),
                        );
                        return;
                    }
                    if tokio::time::Instant::now() >= deadline {
                        break;
                    }
                }
                let smart = shared.smart_rtl;
                if smart {
                    *lock(&shared.busy) = "rtl".to_string();
                    let _ = shared.commands.send(crate::AgentCommand::SmartRtl);
                } else {
                    lock(&shared.backend).as_dyn().rtl();
                }
                shared.journal.event(
                    "idle.policy",
                    serde_json::json!({
                        "policy": "rtl-after", "action": "rtl",
                        "after_s": after_s, "smart": smart,
                    }),
                );
            });
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
    if !airborne {
        // task_abort("takeoff") mid-climb: same scoped-cancel handoff as
        // the mission runners (a plain refusal never set operator_abort).
        operator_abort_handoff(&shared, "takeoff");
    }
    // Takeoff stays OUTSIDE the task queue, but tasks accepted during the
    // climb queued behind it — start them now that the vehicle is free.
    crate::queue::kick(&shared);
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

/// A runner just ended `Aborted`. Ladder aborts (rtl/land/hold) mean the
/// interrupting command owns the vehicle — do nothing. A SCOPED operator
/// abort (`task_abort`, which raised `operator_abort` alongside `abort`)
/// means the vehicle is simply idle again: consume the flag, lower the
/// abort, and let the configured post-task idle policy take over — never
/// an automatic RTL (that remains the ladder's job).
pub(crate) fn operator_abort_handoff(shared: &Arc<AgentShared>, after_task: &'static str) {
    if shared.operator_abort.swap(false, Ordering::Relaxed) {
        shared.abort.store(false, Ordering::Relaxed);
        apply_idle_policy(shared, after_task);
    }
}

/// Publish one `search/status` sample into the latest-wins buffer.
struct SearchStatusPub {
    shared: Arc<AgentShared>,
    mission_id: String,
    legs_total: u64,
}

impl SearchStatusPub {
    fn push(&self, state: &str, leg: u64, frames: u64, note: &str, last_frames: &[String]) {
        let status = SearchStatus {
            detects_completed: 0,
            detects_pending: 0,
            frames_captured: frames,
            gps_time_ns: crate::telemetry::gps_time_ns(),
            // Newest-first published frame names (the GCS fans a detection
            // out per new name). Empty when no sensor feed is fitted — the
            // pre-v3.1 journal-only capture behavior.
            last_frames: last_frames.to_vec(),
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
/// set and the abort flag cleared by the ack handler / queue engine).
/// Runs the flight and its status/journal reporting; the CALLER owns the
/// vehicle-release epilogue (the [`run_raster`] wrapper keeps the legacy
/// clear-busy + idle-policy behavior; the queue driver hands off to the
/// next queue entry instead).
pub(crate) async fn raster_flight_loop(
    shared: &Arc<AgentShared>,
    req: &RasterRequest,
    plan: RasterPlan,
) -> FlightOutcome {
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
    status.push(search_state::TRANSIT, 0, 0, "starting", &[]);
    lock(&shared.backend).as_dyn().set_cruise_speed(plan.speed_m_s);

    let agl = plan.agl_m;
    if !ensure_airborne(shared, agl, LABEL).await {
        status.push(search_state::FAILED, 0, 0, "airborne failed", &[]);
        shared.journal.event(
            "search.finished",
            serde_json::json!({ "mission_id": req.mission_id, "outcome": "failed",
                               "note": "could not reach search altitude" }),
        );
        return if interrupted(shared, LABEL) {
            FlightOutcome::Aborted
        } else {
            FlightOutcome::TimedOut
        };
    }

    let mut clock = MissionClock::start();
    let mut flight = RasterFlight::new(plan, 0.0);
    let mut interval = tokio::time::interval(CONTROL_TICK);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut events = Vec::new();
    let mut last_status = tokio::time::Instant::now();
    // Published frame names, newest first (the GCS detection fan-out reads
    // these off search/status; cap mirrors the v2 last_frames window).
    let mut recent_frames: Vec<String> = Vec::new();
    let outcome = loop {
        interval.tick().await;
        if interrupted(shared, LABEL) {
            // Exact split snapshot: captures only fire inside tick, so the
            // state as of the last tick is the last word (split fidelity).
            let (leg, fired_in_leg) = flight.resume_state();
            crate::queue::save_resume(
                shared,
                crate::queue::ResumeSnapshot::Raster { leg, fired_in_leg },
            );
            break FlightOutcome::Aborted;
        }
        // A sensor-override detour owns the vehicle: freeze the state
        // machine (no target commands, mission clock paused) and keep the
        // status stream fresh; the next cruise re-send after resume
        // re-issues the pre-empted target (v2 override fly-capture-resume).
        let paused = shared.detour.load(Ordering::Relaxed);
        let now_s = clock.tick(paused);
        if paused {
            if last_status.elapsed() >= Duration::from_secs(1) {
                status.push(
                    search_state::SEARCHING,
                    flight.leg() as u64,
                    flight.frames,
                    "paused: sensor-override detour",
                    &recent_frames,
                );
                last_status = tokio::time::Instant::now();
            }
            continue;
        }
        let (outcome, here) = {
            let mut backend = lock(&shared.backend);
            let outcome = flight.tick(now_s, backend.as_dyn(), &mut events);
            let here = backend.as_dyn_ref().position().unwrap_or((0.0, 0.0, 0.0));
            (outcome, here)
        };
        // Queue-stream progress: legs done/total + frames, ETA from the
        // remaining path length ÷ commanded speed. The resume snapshot
        // rides along so a preemption between reports stays capture-exact
        // (the interrupted branch above re-saves it anyway).
        {
            let (pct, eta_s) = flight.progress((here.0, here.1));
            let (leg, fired_in_leg) = flight.resume_state();
            crate::queue::note_progress(
                shared,
                muas_contracts::tasks::TaskProgress {
                    pct,
                    detail: format!(
                        "leg {}/{}, {} frames",
                        (flight.leg() + 1).min(flight.legs_total()),
                        flight.legs_total(),
                        flight.frames
                    ),
                    eta_s,
                },
                Some(crate::queue::ResumeSnapshot::Raster { leg, fired_in_leg }),
            );
        }
        let mut pushed = false;
        for event in events.drain(..) {
            match event {
                MissionEvent::LegStarted { index } => {
                    shared
                        .journal
                        .event("search.leg", serde_json::json!({ "leg": index }));
                    status.push(search_state::SEARCHING, index as u64, flight.frames, "", &recent_frames);
                    pushed = true;
                }
                MissionEvent::Capture { point, lat_deg, lon_deg, agl_m, heading_deg } => {
                    // Render + publish the frame through the data plane
                    // when a sensor feed is fitted (None = the pre-v3.1
                    // journal-only capture).
                    let pose = crate::sensor::SensorPose { lat_deg, lon_deg, agl_m, heading_deg };
                    let published = crate::sensor::publish_raster_capture(
                        shared,
                        &req.mission_id,
                        flight.frames,
                        &pose,
                    )
                    .await;
                    if let Some(name) = &published {
                        recent_frames.insert(0, name.clone());
                        recent_frames.truncate(8);
                    }
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
                            "frame_name": published,
                        }),
                    );
                    status.push(
                        search_state::SEARCHING,
                        flight.leg() as u64,
                        flight.frames,
                        "",
                        &recent_frames,
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
            status.push(search_state::SEARCHING, flight.leg() as u64, flight.frames, "", &recent_frames);
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
    status.push(state, flight.leg() as u64, flight.frames, outcome.as_str(), &recent_frames);
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
    outcome
}

/// Fly an accepted raster request with the LEGACY release epilogue
/// (clear-busy + idle policy / operator handoff) — the pre-queue path,
/// kept for `--no-queue` and the direct-spawn unit tests. Queue-driven
/// rasters run [`raster_flight_loop`] under the queue driver instead.
pub(crate) async fn run_raster(shared: Arc<AgentShared>, req: RasterRequest, plan: RasterPlan) {
    const LABEL: &str = "raster-search";
    let outcome = raster_flight_loop(&shared, &req, plan).await;
    clear_busy(&shared, LABEL);
    if matches!(outcome, FlightOutcome::Completed | FlightOutcome::TimedOut) {
        apply_idle_policy(&shared, LABEL);
    } else {
        operator_abort_handoff(&shared, LABEL);
    }
}

/// v2 investigate cruise speed when the request carries no constraint.
const INVESTIGATE_SPEED_M_S: f64 = 3.0;

/// Fly an accepted investigate request (busy label `"investigate"` is
/// already set and the abort flag cleared by the ack handler). The flight
/// pattern is selected by [`select_investigate_pattern`]: carrot orbit for
/// camera work, acoustic flyover for audio-only jobs.
pub(crate) async fn investigate_flight_loop(
    shared: &Arc<AgentShared>,
    req: &InvestigateRequest,
) -> FlightOutcome {
    const LABEL: &str = "investigate";
    let pattern = select_investigate_pattern(req, shared.extras.iter().any(|e| e == "audio"));
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
            "pattern": pattern,
        }),
    );
    lock(&shared.backend).as_dyn().set_cruise_speed(INVESTIGATE_SPEED_M_S);

    let outcome = if ensure_airborne(shared, req.agl_m, LABEL).await {
        if pattern == investigate_pattern::FLYOVER {
            fly_flyover(shared, req, LABEL).await
        } else {
            fly_orbit(shared, req, LABEL).await
        }
    } else {
        shared.journal.event(
            "investigate.airborne_failed",
            serde_json::json!({ "agl_m": req.agl_m }),
        );
        if interrupted(shared, LABEL) {
            FlightOutcome::Aborted
        } else {
            FlightOutcome::TimedOut
        }
    };

    shared.journal.event(
        "investigate.finished",
        serde_json::json!({
            "mission_id": req.mission_id,
            "outcome": outcome.as_str(),
            "pattern": pattern,
        }),
    );
    info!(outcome = outcome.as_str(), pattern, "investigate finished");
    outcome
}

/// Fly an accepted investigate request with the LEGACY release epilogue —
/// see [`run_raster`] for the split; queue-driven investigations run
/// [`investigate_flight_loop`] under the queue driver.
pub(crate) async fn run_investigate(shared: Arc<AgentShared>, req: InvestigateRequest) {
    const LABEL: &str = "investigate";
    let outcome = investigate_flight_loop(&shared, &req).await;
    clear_busy(&shared, LABEL);
    if matches!(outcome, FlightOutcome::Completed | FlightOutcome::TimedOut) {
        apply_idle_policy(&shared, LABEL);
    } else {
        operator_abort_handoff(&shared, LABEL);
    }
}

/// The v2 carrot-orbit leg of [`run_investigate`].
async fn fly_orbit(shared: &Arc<AgentShared>, req: &InvestigateRequest, label: &str) -> FlightOutcome {
    let turns = if req.turns > 0.0 { req.turns } else { 1.0 };
    let mut flight = OrbitFlight::new(
        (req.lat_deg, req.lon_deg),
        req.radius_m,
        req.agl_m,
        turns,
        INVESTIGATE_SPEED_M_S,
    );
    let radius_m = req.radius_m.max(2.0);
    let t0 = tokio::time::Instant::now();
    let mut interval = tokio::time::interval(CONTROL_TICK);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut events = Vec::new();
    loop {
        interval.tick().await;
        if interrupted(shared, label) {
            // Split snapshot: remaining turns from the ACCUMULATED bearing
            // (swept_turns integrates measured motion, so the continuation
            // flies exactly what is left).
            crate::queue::save_resume(
                shared,
                crate::queue::ResumeSnapshot::Investigate {
                    remaining_turns: (turns - flight.swept_turns()).max(0.0),
                },
            );
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
        // Queue-stream progress: accumulated bearing ÷ target turns, ETA
        // from the remaining arc ÷ angular rate (commanded speed on the
        // circle).
        let swept = flight.swept_turns();
        let remaining = (turns - swept).max(0.0);
        crate::queue::note_progress(
            shared,
            muas_contracts::tasks::TaskProgress {
                pct: (100.0 * swept / turns.max(1e-6)).clamp(0.0, 100.0),
                detail: format!("orbit {swept:.2}/{turns:.2} turns"),
                eta_s: remaining * std::f64::consts::TAU * radius_m / INVESTIGATE_SPEED_M_S,
            },
            Some(crate::queue::ResumeSnapshot::Investigate {
                remaining_turns: remaining,
            }),
        );
        if let Some(outcome) = outcome {
            break outcome;
        }
    }
}

/// The acoustic-flyover leg of [`run_investigate`]: transit at the request
/// AGL, dip to the commandable floor over the target, climb out —
/// `turns` passes rotating cross-axis (uas-flight `flyover_targets`).
async fn fly_flyover(
    shared: &Arc<AgentShared>,
    req: &InvestigateRequest,
    label: &str,
) -> FlightOutcome {
    let points = match flyover_points(shared, req) {
        Ok(points) => points,
        Err(err) => {
            // Geometry rejected (should have been caught at ack): journaled
            // and treated like a failed flight, not a panic.
            shared
                .journal
                .event("investigate.flyover_rejected", serde_json::json!({ "error": err }));
            return FlightOutcome::TimedOut;
        }
    };
    shared.journal.event(
        "investigate.flyover",
        serde_json::json!({
            "waypoints": points.len(),
            "dip_agl_m": shared.agl_bounds.min_agl_m,
            "cruise_agl_m": req.agl_m,
        }),
    );
    // Remaining passes for split/resume: flyover geometry is a fixed
    // number of waypoints per pass, so remaining passes = ceil of the
    // un-flown waypoint fraction (a continuation re-flies whole passes).
    let passes = (req.turns.round() as usize).max(1);
    let per_pass = (points.len() / passes).max(1);
    let mut flight = WaypointFlight::new(points, INVESTIGATE_SPEED_M_S, 0.0);
    let t0 = tokio::time::Instant::now();
    let mut interval = tokio::time::interval(CONTROL_TICK);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let remaining_passes = |flight: &WaypointFlight| {
        let (idx, total) = flight.progress();
        total.saturating_sub(idx).div_ceil(per_pass)
    };
    loop {
        interval.tick().await;
        if interrupted(shared, label) {
            crate::queue::save_resume(
                shared,
                crate::queue::ResumeSnapshot::Investigate {
                    remaining_turns: remaining_passes(&flight) as f64,
                },
            );
            break FlightOutcome::Aborted;
        }
        let now_s = t0.elapsed().as_secs_f64();
        let outcome = {
            let mut backend = lock(&shared.backend);
            flight.tick(now_s, backend.as_dyn())
        };
        let (idx, total) = flight.progress();
        crate::queue::note_progress(
            shared,
            muas_contracts::tasks::TaskProgress {
                pct: if total > 0 { 100.0 * idx as f64 / total as f64 } else { 100.0 },
                detail: format!("flyover waypoint {idx}/{total}"),
                // Coarse: a fixed per-waypoint hop budget beats re-summing
                // the remaining path every tick.
                eta_s: total.saturating_sub(idx) as f64
                    * (2.0 * req.radius_m.max(2.0) / INVESTIGATE_SPEED_M_S + 2.0),
            },
            Some(crate::queue::ResumeSnapshot::Investigate {
                remaining_turns: remaining_passes(&flight) as f64,
            }),
        );
        if let Some(outcome) = outcome {
            break outcome;
        }
    }
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

    /// Split fidelity, scripted time: suspend a raster mid-leg, build the
    /// remainder from the exact resume snapshot, fly it to completion —
    /// remaining captures = total − fired, and the union of planned points
    /// is exactly the plan's capture set (no duplicates, none lost).
    #[test]
    fn raster_split_remainder_fires_exactly_the_unfired_captures() {
        let plan = plan_raster(&raster_req(120.0, 60.0, 20.0, 15.0)).unwrap();
        let total = plan.capture_count();
        let all_points: Vec<(u64, u64)> = plan
            .captures
            .iter()
            .flatten()
            .map(|c| (c.lat_deg.to_bits(), c.lon_deg.to_bits()))
            .collect();

        let mut sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        assert!(sim.ensure_airborne(8.0));
        sim.set_cruise_speed(plan.speed_m_s);

        // Fly the parent until roughly a third of the captures fired, then
        // "preempt" it between ticks (where every interruption lands).
        let mut flight = RasterFlight::new(plan.clone(), 0.0);
        let mut events = Vec::new();
        let mut now = 0.0;
        while (flight.frames as usize) < total / 3 {
            assert!(flight.tick(now, &mut sim, &mut events).is_none(), "ended too early");
            sim.advance(SIM_TICK_S);
            now += SIM_TICK_S;
            assert!(now < 600.0, "parent never reached a third of the captures");
        }
        let fired_first: Vec<(u64, u64)> = events
            .iter()
            .filter_map(|e| match e {
                MissionEvent::Capture { point, .. } => {
                    Some((point.lat_deg.to_bits(), point.lon_deg.to_bits()))
                }
                _ => None,
            })
            .collect();
        assert_eq!(fired_first.len(), flight.frames as usize);

        let (leg, fired_in_leg) = flight.resume_state();
        let remainder = plan.remainder(leg, fired_in_leg).expect("work remains");
        assert_eq!(
            remainder.capture_count(),
            total - fired_first.len(),
            "remaining captures = total - fired"
        );

        // The continuation flies from wherever the vehicle is now.
        let mut cont = RasterFlight::new(remainder, now);
        let mut cont_events = Vec::new();
        let outcome = loop {
            if let Some(outcome) = cont.tick(now, &mut sim, &mut cont_events) {
                break outcome;
            }
            sim.advance(SIM_TICK_S);
            now += SIM_TICK_S;
            assert!(now < 1200.0, "continuation never completed");
        };
        assert_eq!(outcome, FlightOutcome::Completed);

        let mut union: Vec<(u64, u64)> = fired_first;
        union.extend(cont_events.iter().filter_map(|e| match e {
            MissionEvent::Capture { point, .. } => {
                Some((point.lat_deg.to_bits(), point.lon_deg.to_bits()))
            }
            _ => None,
        }));
        assert_eq!(union.len(), total, "every capture fired across the split");
        let mut sorted = union.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), total, "no capture point fired twice");
        let mut expected = all_points;
        expected.sort_unstable();
        assert_eq!(sorted, expected, "the union is exactly the planned capture set");

        // Degenerate remainders: past the last leg there is nothing left.
        assert!(plan.remainder(plan.legs.len(), 0).is_none());
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
    /// Bench agent shared state over a sim backend, with the sim motion
    /// ticker the agent itself would spawn. `customize` runs before the
    /// shared state is frozen behind the `Arc`.
    fn bench_shared_with(
        vehicle_id: &str,
        busy: &str,
        log_dir: Option<std::path::PathBuf>,
        customize: impl FnOnce(&mut AgentShared),
    ) -> (Arc<AgentShared>, crate::SharedBackend) {
        let (journal, _task) = crate::journal::spawn(vehicle_id, log_dir, None, None);
        let (cmd_tx, _cmd_rx) = tokio::sync::mpsc::unbounded_channel();
        let sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        let backend: crate::SharedBackend =
            Arc::new(std::sync::Mutex::new(Box::new(sim) as Box<dyn crate::TickableBackend>));
        let mut shared = AgentShared::bench(vehicle_id, backend.clone(), journal, cmd_tx);
        customize(&mut shared);
        *lock(&shared.busy) = busy.to_string();
        let shared = Arc::new(shared);
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
        (shared, backend)
    }

    fn bench_shared(vehicle_id: &str, busy: &str) -> (Arc<AgentShared>, crate::SharedBackend) {
        bench_shared_with(vehicle_id, busy, None, |_| {})
    }

    /// Parse every journal line from a bench journal directory.
    fn journal_lines(dir: &std::path::Path) -> Vec<serde_json::Value> {
        let mut lines = Vec::new();
        for entry in std::fs::read_dir(dir).into_iter().flatten().flatten() {
            let text = std::fs::read_to_string(entry.path()).unwrap_or_default();
            lines.extend(text.lines().filter_map(|l| serde_json::from_str(l).ok()));
        }
        lines
    }

    fn temp_log_dir(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "muas-mission-test-{tag}-{}-{}",
            std::process::id(),
            crate::telemetry::gps_time_ns()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[tokio::test(start_paused = true)]
    async fn raster_runner_aborts_within_one_cycle() {
        let (shared, _backend) = bench_shared("wuas-77", "raster-search");
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

    /// Poll `predicate` every 100 ms of (paused) tokio time until it holds.
    async fn wait_until(budget_s: f64, mut predicate: impl FnMut() -> bool) -> bool {
        let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(budget_s);
        loop {
            if predicate() {
                return true;
            }
            if tokio::time::Instant::now() > deadline {
                return false;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    fn investigate_req(sensors: &[&str], pattern: &str, north_m: f64) -> InvestigateRequest {
        InvestigateRequest {
            lat_deg: ORIGIN.0 + north_m / EARTH_M_PER_DEG_LAT,
            lon_deg: ORIGIN.1,
            agl_m: 8.0,
            radius_m: 6.0,
            turns: 1.0,
            sensors: sensors.iter().map(|s| s.to_string()).collect(),
            mission_id: "m-test".into(),
            pattern: pattern.into(),
        }
    }

    #[test]
    fn pattern_selection_by_sensor_and_capability() {
        use muas_contracts::services::investigate_pattern as p;
        // auto + audio-only + mic fitted -> flyover
        assert_eq!(select_investigate_pattern(&investigate_req(&["audio"], "", 0.0), true), p::FLYOVER);
        assert_eq!(select_investigate_pattern(&investigate_req(&["audio"], p::AUTO, 0.0), true), p::FLYOVER);
        // camera (or mixed) keeps the carrot orbit
        assert_eq!(select_investigate_pattern(&investigate_req(&["camera"], "", 0.0), true), p::ORBIT);
        assert_eq!(
            select_investigate_pattern(&investigate_req(&["camera", "audio"], "", 0.0), true),
            p::ORBIT
        );
        // no sensors requested: orbit (legacy dispatch)
        assert_eq!(select_investigate_pattern(&investigate_req(&[], "", 0.0), true), p::ORBIT);
        // audio-only but no mic capability: orbit
        assert_eq!(select_investigate_pattern(&investigate_req(&["audio"], "", 0.0), false), p::ORBIT);
        // explicit pattern always wins
        assert_eq!(select_investigate_pattern(&investigate_req(&["camera"], p::FLYOVER, 0.0), false), p::FLYOVER);
        assert_eq!(select_investigate_pattern(&investigate_req(&["audio"], p::ORBIT, 0.0), true), p::ORBIT);
    }

    #[test]
    fn waypoint_flight_flies_the_flyover_profile_with_dips() {
        // Flyover 40 m north of the origin: cruise 8, dip 3.5, radius 6.
        let center_lat = ORIGIN.0 + 40.0 / EARTH_M_PER_DEG_LAT;
        let targets = uas_flight::patterns::flyover_targets(
            Position::new(center_lat, ORIGIN.1, 0.0),
            0.0,
            8.0,
            3.5,
            6.0,
            2,
        )
        .unwrap();
        let points: Vec<(f64, f64, f64)> = targets
            .into_iter()
            .map(|t| (t.position.lat, t.position.lon, t.position.alt))
            .collect();
        assert_eq!(points.len(), 10, "5 waypoints per pass, 2 passes");

        let mut sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        assert!(sim.ensure_airborne(8.0));
        sim.set_cruise_speed(4.0);
        let mut flight = WaypointFlight::new(points, 4.0, 0.0);
        let mut now = 0.0;
        let mut min_agl_near_center = f64::MAX;
        let outcome = loop {
            if let Some(outcome) = flight.tick(now, &mut sim) {
                break outcome;
            }
            sim.advance(SIM_TICK_S);
            now += SIM_TICK_S;
            let (lat, lon, agl) = sim.position().unwrap();
            if dist_m((lat, lon), (center_lat, ORIGIN.1)) < 2.0 {
                min_agl_near_center = min_agl_near_center.min(agl);
            }
            assert!(now < 600.0, "flyover never completed");
        };
        assert_eq!(outcome, FlightOutcome::Completed);
        // The vehicle actually dipped over the target...
        assert!(
            min_agl_near_center < 4.5,
            "no dip over the center (min {min_agl_near_center:.1} m)"
        );
        // ...and climbed back out to cruise by the end.
        let (_, _, agl) = sim.position().unwrap();
        assert!((agl - 8.0).abs() < 1.0, "ended at {agl:.1} m, expected cruise 8");
    }

    /// End-to-end: an audio-only investigate flies the flyover, and the
    /// rtl-after idle policy engages once the vehicle has idled the
    /// configured time (journaled decision, native RTL without a fleet).
    #[tokio::test(start_paused = true)]
    async fn audio_investigate_flies_flyover_then_rtl_after_idles_home() {
        let dir = temp_log_dir("flyover-rtl");
        let (shared, backend) = bench_shared_with("iuas-77", "investigate", Some(dir.clone()), |s| {
            s.extras = vec!["orbit".into(), "audio".into()];
            s.idle_policy = IdlePolicy::RtlAfter(2.0);
        });
        let req = investigate_req(&["audio"], "", 40.0);
        let runner = tokio::spawn(run_investigate(shared.clone(), req));
        assert!(
            wait_until(300.0, || runner.is_finished()).await,
            "investigate never finished"
        );
        assert_eq!(*lock(&shared.busy), "", "vehicle released after the flight");

        // The rtl-after timer runs while the vehicle idles.
        tokio::time::sleep(Duration::from_secs(3)).await;
        let mode = lock(&backend).as_dyn_ref().telemetry().mode;
        assert_eq!(mode, "RTL", "rtl-after engaged native RTL");

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        let started = lines.iter().find(|l| l["kind"] == "investigate.started").unwrap();
        assert_eq!(started["pattern"], "flyover");
        assert!(lines.iter().any(|l| l["kind"] == "investigate.flyover"));
        let finished = lines.iter().find(|l| l["kind"] == "investigate.finished").unwrap();
        assert_eq!(finished["outcome"], "completed");
        // The idle decision is journaled: armed, then fired.
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "idle.policy" && l["action"] == "armed"));
        let fired = lines
            .iter()
            .find(|l| l["kind"] == "idle.policy" && l["action"] == "rtl")
            .expect("rtl decision journaled");
        assert_eq!(fired["smart"], false);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test(start_paused = true)]
    async fn rtl_after_cancels_when_a_new_task_claims_the_vehicle() {
        let dir = temp_log_dir("rtl-cancel");
        let (shared, backend) = bench_shared_with("iuas-78", "", Some(dir.clone()), |s| {
            s.idle_policy = IdlePolicy::RtlAfter(5.0);
        });
        lock(&backend).as_dyn().ensure_airborne(8.0);
        apply_idle_policy(&shared, "investigate");
        // A new task arrives inside the window: the timer must stand down.
        tokio::time::sleep(Duration::from_secs(1)).await;
        *lock(&shared.busy) = "raster-search".to_string();
        tokio::time::sleep(Duration::from_secs(6)).await;
        assert_eq!(lock(&backend).as_dyn_ref().telemetry().mode, "GUIDED");
        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "idle.policy" && l["action"] == "cancelled"));
        assert!(!lines.iter().any(|l| l["kind"] == "idle.policy" && l["action"] == "rtl"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test(start_paused = true)]
    async fn slot_hold_climbs_to_the_fleet_slot() {
        let (shared, backend) = bench_shared_with("iuas-79", "", None, |s| {
            s.idle_policy = IdlePolicy::SlotHold;
            s.slot_agl_m = Some(11.0);
        });
        lock(&backend).as_dyn().ensure_airborne(8.0);
        apply_idle_policy(&shared, "investigate");
        assert!(
            wait_until(30.0, || {
                (lock(&backend).as_dyn_ref().telemetry().agl_m - 11.0).abs() < 0.5
            })
            .await,
            "never climbed to the 11 m slot"
        );
    }

    /// The v2 override contract, end to end: a running raster is suspended
    /// (not aborted), the vehicle flies to the picked point, captures
    /// there, and the raster resumes and completes with every planned
    /// capture — while the ack said exactly that up front.
    #[tokio::test(start_paused = true)]
    async fn override_detour_pauses_raster_captures_at_point_and_resumes() {
        use muas_contracts::services::{sensor_mode, SensorRequest, VehicleService};

        let dir = temp_log_dir("override");
        let (shared, backend) = bench_shared_with("wuas-88", "raster-search", Some(dir.clone()), |s| {
            let feed = crate::sensor::SyntheticFeed::new(
                &crate::sensor::SensorFeedConfig::synthetic(),
                false,
                30.0,
            )
            .unwrap();
            s.sensor_feed = Some(Arc::new(feed));
            // This test pins the LEGACY (--no-queue) semantics: a second
            // override while one is flying refuses busy. With the queue
            // engine on it queues instead (see queue.rs tests).
            s.queue_enabled = false;
        });
        let req = raster_req(120.0, 60.0, 20.0, 15.0);
        let plan = plan_raster(&req).unwrap();
        let capture_total = plan.capture_count();
        let runner = tokio::spawn(run_raster(shared.clone(), req, plan));

        // Let the search actually start sweeping.
        assert!(
            wait_until(60.0, || {
                lock(&shared.latest_search)
                    .as_ref()
                    .and_then(|b| serde_json::from_slice::<SearchStatus>(b).ok())
                    .is_some_and(|s| s.state == search_state::SEARCHING)
            })
            .await,
            "raster never started searching"
        );

        // Operator picks a point 30 m east: override = fly-capture-resume.
        let point = (ORIGIN.0, ORIGIN.1 + 30.0 / m_per_deg_lon(ORIGIN.0));
        let service = crate::service_impl::VehicleServiceImpl::new(shared.clone());
        let ack = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OVERRIDE.into(),
                lat_deg: point.0,
                lon_deg: point.1,
                ..SensorRequest::default()
            })
            .await;
        assert!(ack.accepted, "override refused: {}", ack.detail);
        assert!(
            ack.detail.contains("flying to point") && ack.detail.contains("resuming raster"),
            "ack must say what will happen: '{}'",
            ack.detail
        );
        assert!(shared.detour.load(Ordering::Relaxed), "detour armed");

        // A second override while one is flying is refused busy.
        let second = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OVERRIDE.into(),
                lat_deg: point.0,
                lon_deg: point.1,
                ..SensorRequest::default()
            })
            .await;
        assert!(!second.accepted);
        assert_eq!(second.code, "busy");

        // The capture happens AT the picked point (result lands on
        // sensor/last the moment the capture executes there).
        assert!(
            wait_until(120.0, || lock(&shared.latest_sensor).is_some()).await,
            "override capture never produced a result"
        );
        let (lat, lon, _) = lock(&backend).as_dyn_ref().position().unwrap();
        assert!(
            dist_m((lat, lon), point) < 6.0,
            "capture fired {:.1} m from the picked point",
            dist_m((lat, lon), point)
        );
        let result: serde_json::Value =
            serde_json::from_slice(lock(&shared.latest_sensor).as_ref().unwrap()).unwrap();
        assert_eq!(result["sensor"], "camera");

        // The raster RESUMED (not aborted) and completed every capture.
        assert!(
            wait_until(600.0, || runner.is_finished()).await,
            "raster never finished after the detour"
        );
        let status: SearchStatus =
            serde_json::from_slice(lock(&shared.latest_search).as_ref().unwrap()).unwrap();
        assert_eq!(status.state, search_state::DONE, "raster resumed to completion");
        assert_eq!(status.frames_captured as usize, capture_total);
        assert!(!shared.detour.load(Ordering::Relaxed), "detour released");
        assert_eq!(*lock(&shared.busy), "");

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert!(lines.iter().any(|l| l["kind"] == "sensor.override.started"));
        let finished = lines
            .iter()
            .find(|l| l["kind"] == "sensor.override.finished")
            .expect("override finish journaled");
        assert_eq!(finished["resumed_task"], "raster-search");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test(start_paused = true)]
    async fn override_is_rejected_mid_investigation_and_on_the_ground() {
        use muas_contracts::services::{sensor_mode, SensorRequest, VehicleService};
        // Mid-investigation: the pinned v2 rejection — the LEGACY
        // (--no-queue) behavior; with the queue engine on the override
        // queues behind the investigation instead (see queue.rs tests).
        let (shared, backend) =
            bench_shared_with("iuas-90", "investigate", None, |s| s.queue_enabled = false);
        lock(&backend).as_dyn().ensure_airborne(8.0);
        let service = crate::service_impl::VehicleServiceImpl::new(shared);
        let ack = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OVERRIDE.into(),
                lat_deg: ORIGIN.0,
                lon_deg: ORIGIN.1,
                ..SensorRequest::default()
            })
            .await;
        assert!(!ack.accepted);
        assert_eq!(ack.code, "busy");

        // Idle but on the ground: a detour must not launch a vehicle.
        // (Feed fitted — no-feed vehicles keep the documented stub ack.)
        let (shared, _backend) = bench_shared_with("iuas-91", "", None, |s| {
            let feed = crate::sensor::SyntheticFeed::new(
                &crate::sensor::SensorFeedConfig::synthetic(),
                false,
                30.0,
            )
            .unwrap();
            s.sensor_feed = Some(Arc::new(feed));
        });
        let service = crate::service_impl::VehicleServiceImpl::new(shared.clone());
        let ack = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OVERRIDE.into(),
                lat_deg: ORIGIN.0,
                lon_deg: ORIGIN.1,
                ..SensorRequest::default()
            })
            .await;
        assert!(!ack.accepted);
        assert_eq!(ack.code, "not-airborne");
        assert_eq!(*lock(&shared.busy), "", "refusal must not occupy the vehicle");
    }

    #[tokio::test(start_paused = true)]
    async fn watchpoint_fires_in_passing_and_expires_without_a_pass() {
        use muas_contracts::services::{sensor_mode, SensorRequest, VehicleService};

        // (a) fires when the vehicle passes within the radius
        let dir = temp_log_dir("watchpoint");
        let (shared, backend) = bench_shared_with("iuas-92", "", Some(dir.clone()), |s| {
            let feed = crate::sensor::SyntheticFeed::new(
                &crate::sensor::SensorFeedConfig::synthetic(),
                false,
                30.0,
            )
            .unwrap();
            s.sensor_feed = Some(Arc::new(feed));
        });
        lock(&backend).as_dyn().ensure_airborne(8.0);
        let service = crate::service_impl::VehicleServiceImpl::new(shared.clone());
        let watch = (ORIGIN.0 + 40.0 / EARTH_M_PER_DEG_LAT, ORIGIN.1);
        let ack = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OPPORTUNISTIC.into(),
                lat_deg: watch.0,
                lon_deg: watch.1,
                radius_m: 12.0,
                expiry_s: 300.0,
                ..SensorRequest::default()
            })
            .await;
        assert!(ack.accepted);
        assert!(
            ack.detail.contains("watchpoint") && ack.detail.contains("armed"),
            "detail: {}",
            ack.detail
        );
        // Fly a leg straight through the watchpoint.
        lock(&backend)
            .as_dyn()
            .goto(ORIGIN.0 + 80.0 / EARTH_M_PER_DEG_LAT, ORIGIN.1, 8.0, None);
        assert!(
            wait_until(120.0, || lock(&shared.latest_sensor).is_some()).await,
            "watchpoint never fired"
        );
        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert!(lines.iter().any(|l| l["kind"] == "sensor.watchpoint.armed"));
        assert!(lines.iter().any(|l| l["kind"] == "sensor.watchpoint.fired"));
        let _ = std::fs::remove_dir_all(&dir);

        // (b) expires without a pass; the outcome is surfaced
        let dir = temp_log_dir("watchpoint-exp");
        let (shared, backend) = bench_shared_with("iuas-93", "", Some(dir.clone()), |s| {
            let feed = crate::sensor::SyntheticFeed::new(
                &crate::sensor::SensorFeedConfig::synthetic(),
                false,
                30.0,
            )
            .unwrap();
            s.sensor_feed = Some(Arc::new(feed));
        });
        lock(&backend).as_dyn().ensure_airborne(8.0);
        let service = crate::service_impl::VehicleServiceImpl::new(shared.clone());
        let ack = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OPPORTUNISTIC.into(),
                lat_deg: ORIGIN.0 + 200.0 / EARTH_M_PER_DEG_LAT,
                lon_deg: ORIGIN.1,
                radius_m: 10.0,
                expiry_s: 2.0,
                ..SensorRequest::default()
            })
            .await;
        assert!(ack.accepted);
        assert!(
            wait_until(30.0, || lock(&shared.latest_sensor).is_some()).await,
            "expiry never surfaced"
        );
        let result: serde_json::Value =
            serde_json::from_slice(lock(&shared.latest_sensor).as_ref().unwrap()).unwrap();
        assert_eq!(result["status"], "expired");
        shared.journal.sync().await;
        assert!(journal_lines(&dir)
            .iter()
            .any(|l| l["kind"] == "sensor.watchpoint.expired"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Scoped cancel (`task_abort`): aborting the named raster terminates
    /// it within one control cycle, releases the vehicle, journals the
    /// operator abort, and hands over to the idle policy — NO automatic
    /// RTL (that stays the ladder's job). A mismatched label refuses with
    /// `no-such-task` and leaves the task flying.
    #[tokio::test(start_paused = true)]
    async fn task_abort_cancels_only_the_named_task_without_rtl() {
        use muas_contracts::services::VehicleService;

        let dir = temp_log_dir("task-abort");
        let (shared, backend) =
            bench_shared_with("wuas-95", "raster-search", Some(dir.clone()), |_| {});
        let req = raster_req(300.0, 200.0, 25.0, 20.0); // big: stays running
        let plan = plan_raster(&req).unwrap();
        let runner = tokio::spawn(run_raster(shared.clone(), req, plan));
        assert!(
            wait_until(60.0, || {
                lock(&shared.latest_search)
                    .as_ref()
                    .and_then(|b| serde_json::from_slice::<SearchStatus>(b).ok())
                    .is_some_and(|s| s.state == search_state::SEARCHING)
            })
            .await,
            "raster never started searching"
        );
        let service = crate::service_impl::VehicleServiceImpl::new(shared.clone());

        // Wrong label: refused, the raster keeps flying.
        let miss = service.task_abort("investigate".into()).await;
        assert!(!miss.accepted);
        assert_eq!(miss.code, "no-such-task");
        assert!(miss.detail.contains("raster-search"), "detail: {}", miss.detail);
        assert!(!runner.is_finished(), "mismatched label must not abort");
        // Idle vehicles refuse too.
        assert_eq!(*lock(&shared.busy), "raster-search");

        // Right label: the task dies within one cycle, the vehicle idles.
        let ack = service.task_abort("raster-search".into()).await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        tokio::time::sleep(CONTROL_TICK * 2).await;
        assert!(runner.is_finished(), "abort honored within one cycle");
        assert_eq!(*lock(&shared.busy), "", "vehicle released");
        assert!(
            !shared.abort.load(Ordering::Relaxed),
            "scoped abort is consumed — the vehicle is cleanly idle"
        );
        let mode = lock(&backend).as_dyn_ref().telemetry().mode;
        assert_ne!(mode, "RTL", "task_abort must never RTL (ladder's job)");
        let status: SearchStatus =
            serde_json::from_slice(lock(&shared.latest_search).as_ref().unwrap()).unwrap();
        assert_eq!(status.state, search_state::ABORTED);

        // A now-idle vehicle refuses a second abort of the same label.
        let idle = service.task_abort("raster-search".into()).await;
        assert!(!idle.accepted);
        assert_eq!(idle.code, "no-such-task");

        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        let aborted = lines
            .iter()
            .find(|l| l["kind"] == "task.aborted")
            .expect("task.aborted journaled");
        assert_eq!(aborted["label"], "raster-search");
        assert_eq!(aborted["by"], "operator");
        // The idle policy took over (bench default: hold, journaled).
        assert!(
            lines
                .iter()
                .any(|l| l["kind"] == "idle.policy" && l["after_task"] == "raster-search"),
            "idle policy must take over after a scoped abort"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `task_abort("watchpoint:<id>")` removes ONE armed watchpoint by id
    /// while the active task keeps flying untouched; unknown ids refuse.
    #[tokio::test(start_paused = true)]
    async fn watchpoint_is_removable_by_id_without_touching_the_active_task() {
        use muas_contracts::services::{sensor_mode, SensorRequest, VehicleService};

        let dir = temp_log_dir("wp-abort");
        let (shared, _backend) =
            bench_shared_with("wuas-96", "raster-search", Some(dir.clone()), |s| {
                let feed = crate::sensor::SyntheticFeed::new(
                    &crate::sensor::SensorFeedConfig::synthetic(),
                    false,
                    30.0,
                )
                .unwrap();
                s.sensor_feed = Some(Arc::new(feed));
            });
        let req = raster_req(300.0, 200.0, 25.0, 20.0);
        let plan = plan_raster(&req).unwrap();
        let runner = tokio::spawn(run_raster(shared.clone(), req, plan));

        // Arm a far-away watchpoint that will neither fire nor expire soon.
        let service = crate::service_impl::VehicleServiceImpl::new(shared.clone());
        let ack = service
            .sensor_capture(SensorRequest {
                sensor: "camera".into(),
                mode: sensor_mode::OPPORTUNISTIC.into(),
                lat_deg: ORIGIN.0 + 900.0 / EARTH_M_PER_DEG_LAT,
                lon_deg: ORIGIN.1,
                radius_m: 10.0,
                expiry_s: 600.0,
                ..SensorRequest::default()
            })
            .await;
        assert!(ack.accepted);
        // The ack names the id (`watchpoint wp-1 armed: …`) — the UI's ✕
        // parses it back out of the detail.
        assert!(ack.detail.contains("watchpoint wp-1 armed"), "detail: {}", ack.detail);
        assert!(
            wait_until(10.0, || !lock(&shared.watchpoints).is_empty()).await,
            "watchpoint never registered"
        );

        // Unknown id refuses; nothing changes.
        let miss = service.task_abort("watchpoint:wp-999".into()).await;
        assert!(!miss.accepted);
        assert_eq!(miss.code, "no-such-task");

        // Cancel by id: the watchpoint unregisters and surfaces its
        // outcome; the raster never notices.
        let ack = service.task_abort("watchpoint:wp-1".into()).await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        assert!(
            wait_until(10.0, || lock(&shared.watchpoints).is_empty()).await,
            "watchpoint never unregistered"
        );
        let result: serde_json::Value =
            serde_json::from_slice(lock(&shared.latest_sensor).as_ref().unwrap()).unwrap();
        assert_eq!(result["status"], "cancelled");
        assert!(!runner.is_finished(), "the active raster must keep flying");
        assert_eq!(*lock(&shared.busy), "raster-search");

        shared.abort.store(true, Ordering::Relaxed); // wind the bench down
        shared.journal.sync().await;
        let lines = journal_lines(&dir);
        assert!(lines.iter().any(|l| l["kind"] == "sensor.watchpoint.cancelled"));
        assert!(lines
            .iter()
            .any(|l| l["kind"] == "task.aborted" && l["label"] == "watchpoint:wp-1"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
