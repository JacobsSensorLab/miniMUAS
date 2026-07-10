//! Flat-earth lat/lon/alt helpers.
//!
//! Valid over the few-kilometre scales of a single mission area; not suitable
//! for long-range navigation. Faithful port of UAS-IPBRC `relay/core/geo.py`;
//! a geodesic implementation can swap in behind the same surface later.

use serde::{Deserialize, Serialize};

/// Metres per degree of latitude on the flat-earth model.
pub const EARTH_M_PER_DEG_LAT: f64 = 111_320.0;

/// A geographic position. `alt` is metres AMSL unless the surrounding
/// context has pinned the frame to AGL (the MAVLink adapter works all-AGL
/// with `home_alt_m = 0`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

impl Position {
    pub fn new(lat: f64, lon: f64, alt: f64) -> Self {
        Self { lat, lon, alt }
    }

    /// (lat * 1e7, lon * 1e7, alt_mm) as used by MAVLink global-int messages.
    pub fn to_mav(&self) -> (i32, i32, i32) {
        (
            (self.lat * 1e7) as i32,
            (self.lon * 1e7) as i32,
            (self.alt * 1000.0) as i32,
        )
    }
}

/// Metres per degree of longitude at the given latitude.
pub fn m_per_deg_lon(lat_deg: f64) -> f64 {
    EARTH_M_PER_DEG_LAT * lat_deg.to_radians().cos()
}

/// Horizontal (2D) distance in metres.
pub fn horizontal_distance_m(a: &Position, b: &Position) -> f64 {
    let dlat_m = (b.lat - a.lat) * EARTH_M_PER_DEG_LAT;
    let dlon_m = (b.lon - a.lon) * m_per_deg_lon(a.lat);
    (dlat_m * dlat_m + dlon_m * dlon_m).sqrt()
}

/// Straight-line (3D) distance in metres.
pub fn distance_m(a: &Position, b: &Position) -> f64 {
    let h = horizontal_distance_m(a, b);
    let dv = b.alt - a.alt;
    (h * h + dv * dv).sqrt()
}

/// Linear interpolation from `a` to `b` at fraction `t`.
pub fn interpolate(a: &Position, b: &Position, t: f64) -> Position {
    Position {
        lat: a.lat + (b.lat - a.lat) * t,
        lon: a.lon + (b.lon - a.lon) * t,
        alt: a.alt + (b.alt - a.alt) * t,
    }
}

/// Move from `cur` toward `tgt` by at most `max_m` metres.
pub fn step_toward(cur: &Position, tgt: &Position, max_m: f64) -> Position {
    let d = distance_m(cur, tgt);
    if d <= max_m || d == 0.0 {
        *tgt
    } else {
        interpolate(cur, tgt, max_m / d)
    }
}

/// Project `p` onto segment `a`--`b`, returning `(fraction, distance_m)`.
///
/// `fraction` is clamped to [0, 1]; 0 = at `a`, 1 = at `b`. `distance_m` is
/// the straight-line distance from `p` to the closest point on the segment.
/// Uses flat-earth local coordinates centred at `a`.
pub fn project_point_onto_segment(p: &Position, a: &Position, b: &Position) -> (f64, f64) {
    let m_per_lon = m_per_deg_lon(a.lat);
    let bx = (b.lon - a.lon) * m_per_lon;
    let by = (b.lat - a.lat) * EARTH_M_PER_DEG_LAT;
    let bz = b.alt - a.alt;
    let px = (p.lon - a.lon) * m_per_lon;
    let py = (p.lat - a.lat) * EARTH_M_PER_DEG_LAT;
    let pz = p.alt - a.alt;

    let len_sq = bx * bx + by * by + bz * bz;
    if len_sq < 1e-6 {
        return (0.0, distance_m(p, a));
    }

    let dot = px * bx + py * by + pz * bz;
    let t = (dot / len_sq).clamp(0.0, 1.0);
    let closest = interpolate(a, b, t);
    (t, distance_m(p, &closest))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn one_degree_of_latitude() {
        let a = Position::new(0.0, 0.0, 0.0);
        let b = Position::new(1.0, 0.0, 0.0);
        assert!(close(horizontal_distance_m(&a, &b), EARTH_M_PER_DEG_LAT, 1e-6));
    }

    #[test]
    fn longitude_shrinks_with_latitude() {
        let a = Position::new(60.0, 0.0, 0.0);
        let b = Position::new(60.0, 1.0, 0.0);
        let expected = EARTH_M_PER_DEG_LAT * 60.0_f64.to_radians().cos();
        assert!(close(horizontal_distance_m(&a, &b), expected, 1e-6));
    }

    #[test]
    fn distance_includes_vertical() {
        let a = Position::new(0.0, 0.0, 0.0);
        let b = Position::new(0.0, 0.0, 10.0);
        assert!(close(distance_m(&a, &b), 10.0, 1e-9));
        assert!(close(horizontal_distance_m(&a, &b), 0.0, 1e-9));
    }

    #[test]
    fn step_toward_reaches_and_clamps() {
        let a = Position::new(0.0, 0.0, 0.0);
        let b = Position::new(0.0, 0.0, 100.0);
        // Within reach: snaps to target.
        assert_eq!(step_toward(&a, &b, 150.0), b);
        // Out of reach: moves exactly max_m.
        let mid = step_toward(&a, &b, 25.0);
        assert!(close(mid.alt, 25.0, 1e-9));
        // Zero distance: returns target.
        assert_eq!(step_toward(&b, &b, 5.0), b);
    }

    #[test]
    fn interpolate_endpoints() {
        let a = Position::new(1.0, 2.0, 3.0);
        let b = Position::new(4.0, 5.0, 6.0);
        assert_eq!(interpolate(&a, &b, 0.0), a);
        assert_eq!(interpolate(&a, &b, 1.0), b);
        let m = interpolate(&a, &b, 0.5);
        assert!(close(m.lat, 2.5, 1e-12) && close(m.lon, 3.5, 1e-12) && close(m.alt, 4.5, 1e-12));
    }

    #[test]
    fn projection_onto_segment() {
        let a = Position::new(0.0, 0.0, 0.0);
        let b = Position::new(0.0, 0.01, 0.0); // ~1113 m east at the equator
        // Point abeam the midpoint, offset north.
        let p = Position::new(0.001, 0.005, 0.0);
        let (t, d) = project_point_onto_segment(&p, &a, &b);
        assert!(close(t, 0.5, 1e-6));
        assert!(close(d, 0.001 * EARTH_M_PER_DEG_LAT, 0.5));
        // Beyond the end: fraction clamps to 1.
        let q = Position::new(0.0, 0.02, 0.0);
        let (t2, _) = project_point_onto_segment(&q, &a, &b);
        assert!(close(t2, 1.0, 1e-9));
        // Degenerate segment.
        let (t3, d3) = project_point_onto_segment(&p, &a, &a);
        assert_eq!(t3, 0.0);
        assert!(d3 > 0.0);
    }

    #[test]
    fn to_mav_scaling() {
        let p = Position::new(35.3632621, 149.1652374, 12.5);
        assert_eq!(p.to_mav(), (353632621, 1491652374, 12500));
    }
}
