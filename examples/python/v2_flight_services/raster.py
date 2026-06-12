"""Lawnmower raster geometry for the WUAS search service.

Pure geometry, no NDN dependencies: the drone agent flies these legs and
the dashboard renders exactly the same ones before the operator commits,
so what you preview is what flies.

Conventions: axis-aligned rectangles in a local flat-earth frame around
the area center (fine at search scales of tens to hundreds of meters);
legs run along the rectangle's LONGER axis (fewest turns), serpentine
ordering; capture points are spaced along each leg and inherit the leg's
heading so the GCS can geo-project detections.

Smoke test:
    python3 raster.py            # sample area -> legs/captures JSON
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from contracts import SearchArea

EARTH_M_PER_DEG_LAT = 111_111.0


def m_per_deg_lon(lat_deg: float) -> float:
    return EARTH_M_PER_DEG_LAT * max(math.cos(math.radians(lat_deg)), 1e-6)


@dataclass(frozen=True)
class CapturePoint:
    lat_deg: float
    lon_deg: float
    heading_deg: float
    leg: int
    index: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "lat": self.lat_deg,
            "lon": self.lon_deg,
            "heading_deg": self.heading_deg,
            "leg": self.leg,
            "index": self.index,
        }


@dataclass(frozen=True)
class RasterPlan:
    """Resolved raster: rectangle corners, serpentine legs, capture points."""

    center_lat: float
    center_lon: float
    width_m: float   # east-west extent
    height_m: float  # north-south extent
    legs: list[list[tuple[float, float]]]          # [(lat, lon) start, end]
    captures: list[CapturePoint]

    @property
    def corners(self) -> list[tuple[float, float]]:
        """NW, NE, SE, SW (lat, lon) — render-ready polygon order."""
        dlat = (self.height_m / 2.0) / EARTH_M_PER_DEG_LAT
        dlon = (self.width_m / 2.0) / m_per_deg_lon(self.center_lat)
        n, s = self.center_lat + dlat, self.center_lat - dlat
        w, e = self.center_lon - dlon, self.center_lon + dlon
        return [(n, w), (n, e), (s, e), (s, w)]

    def as_dict(self) -> dict[str, Any]:
        return {
            "center": {"lat": self.center_lat, "lon": self.center_lon},
            "width_m": self.width_m,
            "height_m": self.height_m,
            "corners": [{"lat": a, "lon": b} for a, b in self.corners],
            "legs": [
                [{"lat": a, "lon": b} for a, b in leg] for leg in self.legs
            ],
            "captures": [c.as_dict() for c in self.captures],
        }


def resolve_area(area: SearchArea) -> tuple[float, float, float, float]:
    """SearchArea (either mode) -> (center_lat, center_lon, width_m, height_m)."""

    if area.mode == "corners" and len(area.corner_a) == 2 and len(area.corner_b) == 2:
        lat_a, lon_a = area.corner_a
        lat_b, lon_b = area.corner_b
        center_lat = (lat_a + lat_b) / 2.0
        center_lon = (lon_a + lon_b) / 2.0
        height_m = abs(lat_a - lat_b) * EARTH_M_PER_DEG_LAT
        width_m = abs(lon_a - lon_b) * m_per_deg_lon(center_lat)
        return center_lat, center_lon, width_m, height_m
    return area.center_lat, area.center_lon, area.width_m, area.height_m


def build_raster(
    area: SearchArea,
    *,
    leg_spacing_m: float = 5.0,
    capture_every_m: float = 4.0,
) -> RasterPlan:
    """Serpentine lawnmower covering the rectangle.

    Legs run along the longer axis; spacing is shrunk slightly when needed
    so legs cover the full perpendicular extent symmetrically (first and
    last legs sit half a spacing inside the boundary, matching the camera
    footprint center, not its edge).
    """

    center_lat, center_lon, width_m, height_m = resolve_area(area)
    leg_spacing_m = max(float(leg_spacing_m), 0.5)
    capture_every_m = max(float(capture_every_m), 0.5)

    along_ew = width_m >= height_m  # legs run east-west when wider than tall
    along_m = width_m if along_ew else height_m
    across_m = height_m if along_ew else width_m

    leg_count = max(1, int(math.ceil(across_m / leg_spacing_m)))
    # symmetric offsets across the perpendicular axis, centered on 0
    if leg_count == 1:
        across_offsets = [0.0]
    else:
        step = across_m / leg_count
        start = -(across_m / 2.0) + step / 2.0
        across_offsets = [start + i * step for i in range(leg_count)]

    half_along = along_m / 2.0
    lat_per_m = 1.0 / EARTH_M_PER_DEG_LAT
    lon_per_m = 1.0 / m_per_deg_lon(center_lat)

    def to_latlon(along: float, across: float) -> tuple[float, float]:
        if along_ew:
            east, north = along, across
        else:
            east, north = across, along
        return (
            center_lat + north * lat_per_m,
            center_lon + east * lon_per_m,
        )

    legs: list[list[tuple[float, float]]] = []
    captures: list[CapturePoint] = []
    for leg_index, across in enumerate(across_offsets):
        forward = leg_index % 2 == 0  # serpentine
        a, b = (-half_along, half_along) if forward else (half_along, -half_along)
        start = to_latlon(a, across)
        end = to_latlon(b, across)
        legs.append([start, end])

        if along_ew:
            heading = 90.0 if forward else 270.0
        else:
            heading = 0.0 if forward else 180.0

        n_caps = max(1, int(math.floor(along_m / capture_every_m)) + 1)
        for cap_index in range(n_caps):
            frac = (cap_index / (n_caps - 1)) if n_caps > 1 else 0.5
            along_pos = a + (b - a) * frac
            lat, lon = to_latlon(along_pos, across)
            captures.append(
                CapturePoint(
                    lat_deg=lat,
                    lon_deg=lon,
                    heading_deg=heading,
                    leg=leg_index,
                    index=len(captures),
                )
            )

    return RasterPlan(
        center_lat=center_lat,
        center_lon=center_lon,
        width_m=width_m,
        height_m=height_m,
        legs=legs,
        captures=captures,
    )


def estimate_duration_s(
    plan: RasterPlan, *, speed_m_s: float, turn_penalty_s: float = 3.0
) -> float:
    """Coarse flight-time estimate for UI display."""

    along = max(plan.width_m, plan.height_m)
    legs = len(plan.legs)
    across_travel = min(plan.width_m, plan.height_m)
    distance = legs * along + across_travel
    return distance / max(speed_m_s, 0.1) + max(legs - 1, 0) * turn_penalty_s


if __name__ == "__main__":
    import json

    sample = SearchArea(
        mode="center",
        center_lat=35.1208,
        center_lon=-89.9347,
        width_m=40.0,
        height_m=24.0,
    )
    plan = build_raster(sample, leg_spacing_m=5.0, capture_every_m=4.0)
    print(json.dumps(
        {
            **plan.as_dict(),
            "legs_count": len(plan.legs),
            "captures_count": len(plan.captures),
            "estimate_s": round(estimate_duration_s(plan, speed_m_s=2.0), 1),
        },
        indent=2,
    ))
