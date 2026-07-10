//! Raster preview built on uas-flight's pattern geometry — the v3 form of
//! v2's "what you preview is what flies" rule (`raster.py` was shared by
//! agent and dashboard; v3 shares [`uas_flight::patterns::RasterPath`]).
//!
//! The preview response keeps the v2 `raster_preview` JSON shape (center,
//! corners NW/NE/SE/SW, legs, capture points, duration estimate) so the
//! ported frontend renders it unchanged.
//!
//! Documented deviation from v2 `raster.py`: lane placement follows
//! uas-flight's `lane_offsets` (lanes span the rectangle edge-to-edge,
//! `ceil(w/s)+1` lanes, minimum 2) instead of v2's half-spacing-inset lanes,
//! because that is the geometry the v3 agent flies. Legs still run along
//! the longer axis with serpentine ordering, and captures are spaced evenly
//! along each leg including both endpoints, exactly like v2.

use serde_json::{json, Value};
use uas_flight::geo::{m_per_deg_lon, Position, EARTH_M_PER_DEG_LAT};
use uas_flight::motion::MotionTarget;
use uas_flight::patterns::{RasterBounds, RasterPath};

/// A resolved preview plan (v2 `RasterPlan` facts).
#[derive(Debug, Clone, PartialEq)]
pub struct PreviewPlan {
    pub center_lat: f64,
    pub center_lon: f64,
    /// East–west extent, metres.
    pub width_m: f64,
    /// North–south extent, metres.
    pub height_m: f64,
    /// Serpentine legs as `[(lat, lon) start, (lat, lon) end]`.
    pub legs: Vec<[(f64, f64); 2]>,
    /// Capture points `(lat, lon, heading_deg, leg_index)`.
    pub captures: Vec<(f64, f64, f64, usize)>,
}

/// v2 `resolve_area`: a `SearchArea` dict (either mode) →
/// `(center_lat, center_lon, width_m, height_m)`.
pub fn resolve_area(area: &Value) -> (f64, f64, f64, f64) {
    let f = |key: &str| area.get(key).and_then(Value::as_f64).unwrap_or(0.0);
    let mode = area.get("mode").and_then(Value::as_str).unwrap_or("center");
    if mode == "corners" {
        let corner = |key: &str| -> Option<(f64, f64)> {
            let c = area.get(key)?.as_array()?;
            Some((c.first()?.as_f64()?, c.get(1)?.as_f64()?))
        };
        if let (Some((lat_a, lon_a)), Some((lat_b, lon_b))) =
            (corner("corner_a"), corner("corner_b"))
        {
            let center_lat = (lat_a + lat_b) / 2.0;
            let center_lon = (lon_a + lon_b) / 2.0;
            let height_m = (lat_a - lat_b).abs() * EARTH_M_PER_DEG_LAT;
            let width_m = (lon_a - lon_b).abs() * m_per_deg_lon(center_lat);
            return (center_lat, center_lon, width_m, height_m);
        }
    }
    (f("center_lat"), f("center_lon"), f("width_m"), f("height_m"))
}

/// Rectangle corners NW, NE, SE, SW (render-ready polygon order, v2
/// `RasterPlan.corners`).
pub fn area_corners(center_lat: f64, center_lon: f64, width_m: f64, height_m: f64) -> [(f64, f64); 4] {
    let dlat = (height_m / 2.0) / EARTH_M_PER_DEG_LAT;
    let dlon = (width_m / 2.0) / m_per_deg_lon(center_lat);
    let (n, s) = (center_lat + dlat, center_lat - dlat);
    let (w, e) = (center_lon - dlon, center_lon + dlon);
    [(n, w), (n, e), (s, e), (s, w)]
}

/// Corners for a v3 `RasterRequest` from a v2 area dict.
pub fn corners_for_area(area: &Value) -> Vec<(f64, f64)> {
    let (clat, clon, w, h) = resolve_area(area);
    area_corners(clat, clon, w, h).to_vec()
}

/// The uas-flight raster path for an area — the SAME structure the agent
/// flies. Legs run along the longer axis (fewest turns): east–west when the
/// area is wider than tall (bearing 90°), north–south otherwise (0°).
pub fn flight_path(
    center_lat: f64,
    center_lon: f64,
    width_m: f64,
    height_m: f64,
    leg_spacing_m: f64,
) -> RasterPath {
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
    path.lane_spacing_m = leg_spacing_m.max(0.5);
    path
}

/// Build the preview plan by pairing the flight path's lane-endpoint
/// targets into legs and spacing captures along each leg (v2 capture rule:
/// `max(1, floor(len/step)+1)` points, endpoints included).
pub fn build_preview(
    area: &Value,
    leg_spacing_m: f64,
    capture_every_m: f64,
) -> Result<PreviewPlan, String> {
    let (center_lat, center_lon, width_m, height_m) = resolve_area(area);
    let path = flight_path(center_lat, center_lon, width_m, height_m, leg_spacing_m);
    let targets: Vec<MotionTarget> = path.targets().map_err(|e| e.to_string())?;
    let capture_every_m = capture_every_m.max(0.5);

    let mut legs = Vec::new();
    let mut captures = Vec::new();
    for (leg_index, pair) in targets.chunks(2).enumerate() {
        let [a, b] = pair else { continue };
        let start = (a.position.lat, a.position.lon);
        let end = (b.position.lat, b.position.lon);
        legs.push([start, end]);

        // Leg heading from the travel direction (flat-earth bearing).
        let dn = (end.0 - start.0) * EARTH_M_PER_DEG_LAT;
        let de = (end.1 - start.1) * m_per_deg_lon(center_lat);
        let heading = de.atan2(dn).to_degrees().rem_euclid(360.0);
        let leg_len = dn.hypot(de);

        let n_caps = usize::max(1, (leg_len / capture_every_m).floor() as usize + 1);
        for i in 0..n_caps {
            let frac = if n_caps > 1 {
                i as f64 / (n_caps - 1) as f64
            } else {
                0.5
            };
            captures.push((
                start.0 + (end.0 - start.0) * frac,
                start.1 + (end.1 - start.1) * frac,
                heading,
                leg_index,
            ));
        }
    }
    Ok(PreviewPlan {
        center_lat,
        center_lon,
        width_m,
        height_m,
        legs,
        captures,
    })
}

/// v2 `estimate_duration_s`: coarse flight-time estimate for UI display.
pub fn estimate_duration_s(plan: &PreviewPlan, speed_m_s: f64) -> f64 {
    const TURN_PENALTY_S: f64 = 3.0;
    let along = plan.width_m.max(plan.height_m);
    let legs = plan.legs.len() as f64;
    let across_travel = plan.width_m.min(plan.height_m);
    let distance = legs * along + across_travel;
    distance / speed_m_s.max(0.1) + (legs - 1.0).max(0.0) * TURN_PENALTY_S
}

/// The full `raster_preview` message (v2 wire shape).
pub fn preview_message(area: &Value, leg_spacing_m: f64, capture_every_m: f64, speed_m_s: f64) -> Value {
    match build_preview(area, leg_spacing_m, capture_every_m) {
        Ok(plan) => {
            let estimate = estimate_duration_s(&plan, speed_m_s);
            json!({
                "type": "raster_preview",
                "plan": plan_to_json(&plan),
                "estimate_s": (estimate * 10.0).round() / 10.0,
            })
        }
        Err(error) => json!({ "type": "raster_preview", "error": error }),
    }
}

/// v2 `RasterPlan.as_dict`.
pub fn plan_to_json(plan: &PreviewPlan) -> Value {
    json!({
        "center": { "lat": plan.center_lat, "lon": plan.center_lon },
        "width_m": plan.width_m,
        "height_m": plan.height_m,
        "corners": area_corners(plan.center_lat, plan.center_lon, plan.width_m, plan.height_m)
            .iter()
            .map(|(lat, lon)| json!({ "lat": lat, "lon": lon }))
            .collect::<Vec<_>>(),
        "legs": plan
            .legs
            .iter()
            .map(|leg| {
                json!([
                    { "lat": leg[0].0, "lon": leg[0].1 },
                    { "lat": leg[1].0, "lon": leg[1].1 },
                ])
            })
            .collect::<Vec<_>>(),
        "captures": plan
            .captures
            .iter()
            .enumerate()
            .map(|(index, (lat, lon, heading, leg))| {
                json!({
                    "lat": lat,
                    "lon": lon,
                    "heading_deg": heading,
                    "leg": leg,
                    "index": index,
                })
            })
            .collect::<Vec<_>>(),
    })
}
