#!/usr/bin/env python3
"""Prefetch satellite tiles into the dashboard's offline cache.

The mission console serves tiles from a local directory; in the field there
is no internet, so coverage of the operating area must be cached ahead of
time. Two ways to warm the cache:

  1. Implicit: pan/zoom the dashboard while node 03 has internet — the
     backend proxies and caches every tile it serves.
  2. Deliberate (this tool): bulk-download a bounding box across a zoom
     range, e.g. for the field site:

       muas-v2-fetch-tiles --bbox 35.1190,-89.9365,35.1225,-89.9330 \
           --zooms 15 19 --tiles-dir /var/lib/minimuas/tiles

Stdlib only (urllib); skips tiles already cached; gentle politeness delay.
A z15-19 cache of a ~400x400 m site is a few hundred tiles, a few MB.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import urllib.request
from pathlib import Path

DEFAULT_UPSTREAM = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/"
    "World_Imagery/MapServer/tile/{z}/{y}/{x}"
)
USER_AGENT = "miniMUAS-v2-tile-prefetch/1.0"


def lon2tx(lon: float, z: int) -> float:
    return (lon + 180.0) / 360.0 * (2 ** z)


def lat2ty(lat: float, z: int) -> float:
    r = math.radians(lat)
    return (1 - math.log(math.tan(r) + 1 / math.cos(r)) / math.pi) / 2 * (2 ** z)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bbox", required=True,
        help="latS,lonW,latN,lonE of the area to cache",
    )
    parser.add_argument(
        "--zooms", nargs=2, type=int, default=[15, 19], metavar=("MIN", "MAX"),
    )
    parser.add_argument("--tiles-dir", default="/var/lib/minimuas/tiles")
    parser.add_argument("--upstream", default=DEFAULT_UPSTREAM)
    parser.add_argument("--delay-s", type=float, default=0.05)
    args = parser.parse_args()

    try:
        lat_s, lon_w, lat_n, lon_e = (float(v) for v in args.bbox.split(","))
    except ValueError:
        print("--bbox must be latS,lonW,latN,lonE", file=sys.stderr)
        return 2
    if lat_s > lat_n:
        lat_s, lat_n = lat_n, lat_s
    if lon_w > lon_e:
        lon_w, lon_e = lon_e, lon_w

    root = Path(args.tiles_dir)
    fetched = skipped = failed = 0
    for z in range(args.zooms[0], args.zooms[1] + 1):
        x0, x1 = int(lon2tx(lon_w, z)), int(lon2tx(lon_e, z))
        y0, y1 = int(lat2ty(lat_n, z)), int(lat2ty(lat_s, z))
        total = (x1 - x0 + 1) * (y1 - y0 + 1)
        print(f"z{z}: {total} tiles")
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                path = root / str(z) / str(x) / f"{y}.jpg"
                if path.exists():
                    skipped += 1
                    continue
                url = args.upstream.format(z=z, x=x, y=y)
                request = urllib.request.Request(
                    url, headers={"User-Agent": USER_AGENT}
                )
                try:
                    with urllib.request.urlopen(request, timeout=10) as r:
                        body = r.read()
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(body)
                    fetched += 1
                except Exception as exc:
                    failed += 1
                    print(f"  failed {z}/{x}/{y}: {exc}", file=sys.stderr)
                time.sleep(args.delay_s)
    print(f"done: fetched={fetched} skipped={skipped} failed={failed}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
