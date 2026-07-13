"""Mission data bundle: assemble + import the one-click field-mission archive.

After a field mission the operator clicks "Download mission data" on the
dashboard and the ENTIRE mission is pulled to their computer over the NDN
fabric (no per-node SSH): every node's journal (events + metrics + logs), all
captured artifacts (video + audio) with their MUASFRAME1 pose/time/hfov
metadata, and the dashboard's own recording. Importing that archive into a
sim-mode dashboard replays the full mission with `/artifact` resolving from the
bundle instead of the live fabric.

This module is deliberately NDN-free and dependency-light so it unit-tests
without the fabric: the caller injects `journal_fetcher(node_id) -> bytes|None`
and `artifact_fetcher(name) -> bytes|None` callables (real NDN `fetch_segmented`
in production, local-file readers in tests). Frame payloads are parsed with
`dataplane.parse_frame`/`frame_body` so the persisted media + `.meta.json`
carry the genuine capture header, not a re-derived guess.

Archive layout  ``mission-<session>-<ts>.tar.gz``::

    manifest.json               schema_version, session, fleet, wall-time range,
                                per-node fetch status (ok/missing), counts
    dashboard.jsonl             the dashboard mission recording (dash-*.jsonl)
    nodes/<node>/journal.jsonl  each vehicle's + gcs agent journal
    artifacts/index.json        ndn-name -> {path, kind, content_type, vid,
                                time_ns, pose, hfov}
    artifacts/<vid>/<safe>.jpg|.wav          media body (decoded from the frame)
    artifacts/<vid>/<safe>.jpg.meta.json     time, pose, hfov, kind, full header
    metrics/summary.csv         optional aggregate_latency.py output (if supplied)
"""

from __future__ import annotations

import hashlib
import io
import json
import re
import tarfile
import time
from pathlib import Path
from typing import Any, Callable, Iterable

from dataplane import frame_body, parse_frame


SCHEMA_VERSION = 1

# Media file extension per declared frame `kind`. Anything unrecognized is
# stored as .bin so the body is never lost even if we cannot name it.
_EXT_BY_KIND = {
    "image/jpeg": "jpg",
    "image/jpg": "jpg",
    "image/png": "png",
    "audio/wav": "wav",
    "audio/x-wav": "wav",
    "synthetic": "bin",
}


def _ext_for_kind(kind: str) -> str:
    return _EXT_BY_KIND.get((kind or "").lower(), "bin")


def _content_type_for_kind(kind: str) -> str:
    """A real MIME the browser can consume; mirrors Dashboard.fetch_artifact."""
    kind = (kind or "").strip()
    if "/" in kind:
        return kind
    return "image/jpeg"


def _safe_token(name: str) -> str:
    """Filesystem-safe, collision-free token for an NDN data name.

    The last two path components stay human-readable (a frame's timestamp/seq
    tail) and a short hash of the full name guarantees uniqueness across the
    whole tree even when two names share a tail.
    """
    tail = "-".join(p for p in name.strip("/").split("/")[-2:] if p) or "artifact"
    tail = re.sub(r"[^A-Za-z0-9._-]", "_", tail)[:48]
    digest = hashlib.sha1(name.encode()).hexdigest()[:8]
    return f"{tail}-{digest}"


def _num(value: Any):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _pose_from_header(header: dict) -> dict:
    """Normalize the drone pose the frame carried into a flat dict.

    Producers stash pose in the frame `metadata` sub-dict as strings
    (lat_deg/lon_deg/agl_m/heading_deg, plus roll_deg/pitch_deg for the GCS
    projection). Missing fields are simply absent — never fabricated.
    """
    meta = header.get("metadata") or {}
    pose: dict[str, float] = {}
    for key in (
        "lat_deg", "lon_deg", "alt_m", "agl_m",
        "heading_deg", "roll_deg", "pitch_deg", "yaw_deg",
    ):
        if key in meta:
            v = _num(meta[key])
            if v is not None:
                pose[key] = v
    return pose


def _hfov_from_header(header: dict):
    meta = header.get("metadata") or {}
    for key in ("hfov_deg", "hfov"):
        if key in meta:
            v = _num(meta[key])
            if v is not None:
                return v
    return None


def assemble_bundle(
    staging_dir: Path | str,
    *,
    session: str,
    fleet: Iterable[str],
    artifacts: Iterable[dict],
    journal_fetcher: Callable[[str], bytes | None],
    artifact_fetcher: Callable[[str], bytes | None],
    dashboard_jsonl_path: Path | str | None = None,
    dashboard_jsonl_bytes: bytes | None = None,
    metrics_csv: str | None = None,
    extra_journal_nodes: Iterable[str] = (),
) -> dict:
    """Populate ``staging_dir`` with the mission bundle tree; return manifest.

    ``artifacts`` are the dashboard's sensor-data catalog entries
    ({name, kind, vehicle, lat, lon, t, ...}) — every artifact the mission
    referenced, not just the ones on screen. ``fleet`` + ``extra_journal_nodes``
    are the node ids whose journals to sweep. Fetch failures never raise: a
    powered-down node is recorded as ``missing`` in the manifest (SSH stays the
    escape hatch) and assembly still produces a coherent archive.
    """
    staging = Path(staging_dir)
    staging.mkdir(parents=True, exist_ok=True)

    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "session": session,
        "created_wall": time.time(),
        "fleet": list(fleet),
        "nodes": {},
        "counts": {},
        "wall_range": None,
    }

    # ---- dashboard recording ------------------------------------------------
    dash_ok = False
    if dashboard_jsonl_bytes is not None:
        (staging / "dashboard.jsonl").write_bytes(dashboard_jsonl_bytes)
        dash_ok = True
    elif dashboard_jsonl_path and Path(dashboard_jsonl_path).exists():
        (staging / "dashboard.jsonl").write_bytes(
            Path(dashboard_jsonl_path).read_bytes()
        )
        dash_ok = True
    manifest["dashboard_recording"] = "ok" if dash_ok else "missing"

    # ---- per-node journals (over NDN in production) -------------------------
    nodes_dir = staging / "nodes"
    node_ids: list[str] = []
    for n in list(fleet) + list(extra_journal_nodes):
        if n not in node_ids:
            node_ids.append(n)
    for node in node_ids:
        status = "missing"
        try:
            data = journal_fetcher(node)
        except Exception:
            data = None
        if data:
            node_dir = nodes_dir / node
            node_dir.mkdir(parents=True, exist_ok=True)
            (node_dir / "journal.jsonl").write_bytes(data)
            status = "ok"
        manifest["nodes"][node] = {"journal": status}
    manifest["counts"]["journals_ok"] = sum(
        1 for v in manifest["nodes"].values() if v["journal"] == "ok"
    )

    # ---- artifacts: media + per-item metadata + catalog index ---------------
    art_dir = staging / "artifacts"
    art_dir.mkdir(parents=True, exist_ok=True)
    index: dict[str, dict] = {}
    seen: set[str] = set()
    counts = {"artifacts_ok": 0, "artifacts_missing": 0, "audio": 0, "video": 0}
    wall_lo = wall_hi = None

    for item in artifacts:
        name = item.get("name")
        if not name or name in seen:
            continue
        seen.add(name)
        vid = item.get("vehicle") or "unknown"
        try:
            payload = artifact_fetcher(name)
        except Exception:
            payload = None
        if not payload:
            counts["artifacts_missing"] += 1
            index[name] = {
                "status": "missing",
                "vid": vid,
                "kind": item.get("kind", ""),
            }
            continue
        try:
            header = parse_frame(payload)
            body = frame_body(payload)
        except Exception:
            counts["artifacts_missing"] += 1
            index[name] = {"status": "corrupt", "vid": vid,
                           "kind": item.get("kind", "")}
            continue

        kind = str(header.get("kind") or item.get("kind") or "image/jpeg")
        ext = _ext_for_kind(kind)
        safe = _safe_token(name)
        rel = f"artifacts/{_safe_token(vid) if '/' in vid else vid}/{safe}.{ext}"
        media_path = staging / rel
        media_path.parent.mkdir(parents=True, exist_ok=True)
        media_path.write_bytes(body)

        time_ns = int(header.get("gps_time_ns") or 0) or None
        pose = _pose_from_header(header)
        hfov = _hfov_from_header(header)
        meta = {
            "name": name,
            "kind": kind,
            "content_type": _content_type_for_kind(kind),
            "vid": vid,
            "time_ns": time_ns,
            "pose": pose,
            "hfov": hfov,
            "header": header,
            "catalog": {k: item.get(k) for k in
                        ("sensor", "source", "label", "lat", "lon", "t")},
        }
        (media_path.parent / (media_path.name + ".meta.json")).write_text(
            json.dumps(meta, indent=2, sort_keys=True)
        )
        index[name] = {
            "status": "ok",
            "path": rel,
            "kind": kind,
            "content_type": _content_type_for_kind(kind),
            "vid": vid,
            "time_ns": time_ns,
            "pose": pose,
            "hfov": hfov,
        }
        counts["artifacts_ok"] += 1
        if ext == "wav" or kind.startswith("audio"):
            counts["audio"] += 1
        else:
            counts["video"] += 1
        # wall-time range from the capture catalog timestamps
        t = _num(item.get("t"))
        if t is not None:
            wall_lo = t if wall_lo is None else min(wall_lo, t)
            wall_hi = t if wall_hi is None else max(wall_hi, t)

    (art_dir / "index.json").write_text(
        json.dumps(index, indent=2, sort_keys=True)
    )
    manifest["counts"].update(counts)
    if wall_lo is not None:
        manifest["wall_range"] = [wall_lo, wall_hi]

    # ---- optional baked metrics summary ------------------------------------
    if metrics_csv:
        metrics_dir = staging / "metrics"
        metrics_dir.mkdir(parents=True, exist_ok=True)
        (metrics_dir / "summary.csv").write_text(metrics_csv)
        manifest["metrics_summary"] = "metrics/summary.csv"

    (staging / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True)
    )
    return manifest


def tar_gz_bytes(staging_dir: Path | str, *, arcroot: str = "") -> bytes:
    """Pack a populated staging dir into an in-memory .tar.gz."""
    staging = Path(staging_dir)
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for path in sorted(staging.rglob("*")):
            if path.is_file():
                arcname = str(path.relative_to(staging))
                if arcroot:
                    arcname = f"{arcroot}/{arcname}"
                tar.add(path, arcname=arcname)
    return buf.getvalue()


def bundle_filename(session: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9._-]", "-", session or "mission") or "mission"
    return f"mission-{safe}-{time.strftime('%Y%m%d-%H%M%S')}.tar.gz"


# ---------------------------------------------------------------------------
# Import side: unpack an archive and resolve /artifact from it.
# ---------------------------------------------------------------------------


class BundleView:
    """A loaded mission bundle: dashboard recording + artifact resolver.

    ``artifact(name)`` returns ``(body_bytes, content_type)`` for a stored
    capture or ``None`` if the bundle never carried it — exactly the shape the
    dashboard's live ``fetch_artifact`` returns, so the `/artifact` handler can
    prefer the bundle and fall through to the fabric unchanged.
    """

    def __init__(self, root: Path) -> None:
        self.root = Path(root)
        self.manifest = self._load_json("manifest.json") or {}
        self.index = self._load_json("artifacts/index.json") or {}
        dash = self.root / "dashboard.jsonl"
        self.dashboard_jsonl_path = dash if dash.exists() else None
        self.session = str(self.manifest.get("session") or "imported")

    def _load_json(self, rel: str):
        p = self.root / rel
        if not p.exists():
            return None
        try:
            return json.loads(p.read_text())
        except Exception:
            return None

    def artifact(self, name: str) -> tuple[bytes, str] | None:
        entry = self.index.get(name)
        if not entry or entry.get("status") != "ok":
            return None
        rel = entry.get("path")
        if not rel:
            return None
        media = self.root / rel
        if not media.exists():
            return None
        try:
            body = media.read_bytes()
        except Exception:
            return None
        return body, str(entry.get("content_type") or "image/jpeg")

    def dashboard_jsonl_text(self) -> str:
        if self.dashboard_jsonl_path is None:
            return ""
        try:
            return self.dashboard_jsonl_path.read_text()
        except Exception:
            return ""


def extract_bundle(source: bytes | str | Path, dest_dir: Path | str) -> BundleView:
    """Unpack a .tar.gz (bytes or path) into ``dest_dir``; return a BundleView.

    Rejects entries that escape the destination (path traversal) — a bundle can
    arrive from anywhere, so extraction is hardened even though we authored the
    writer.
    """
    dest = Path(dest_dir)
    dest.mkdir(parents=True, exist_ok=True)
    if isinstance(source, (bytes, bytearray)):
        fileobj: Any = io.BytesIO(source)
        opener = tarfile.open(fileobj=fileobj, mode="r:gz")
    else:
        opener = tarfile.open(str(source), mode="r:gz")
    with opener as tar:
        members = []
        for m in tar.getmembers():
            target = (dest / m.name).resolve()
            if not str(target).startswith(str(dest.resolve())):
                continue  # skip traversal attempts
            if m.isfile() or m.isdir():
                members.append(m)
        tar.extractall(dest, members=members)
    # A bundle may unpack under a single arcroot dir; find the manifest.
    root = dest
    if not (root / "manifest.json").exists():
        for cand in dest.iterdir():
            if cand.is_dir() and (cand / "manifest.json").exists():
                root = cand
                break
    return BundleView(root)
