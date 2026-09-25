#!/usr/bin/env python3
"""Flight-readiness check against the live miniMUAS dashboard WebSocket.

Measures what an operator actually depends on in the air: telemetry cadence,
video frame delivery, and command round-trips — from the same socket the
browser uses, so it proves the real operator path end to end.

Usage:
  flightcheck.py --seconds 60
  flightcheck.py --seconds 60 --video iuas-01 --transport segmented
  flightcheck.py --seconds 60 --video iuas-01 --transport stream
  flightcheck.py --probe --json armed.json          # passive: who is armed?
  flightcheck.py --seconds 90 --video iuas-01,iuas-02,wuas-01 --video-all \\
      --stop-before --stop-after --json run.json    # one ndn-fleet sample
Stdlib only (no websockets package on this host).

Verdicts: FLIGHT-READY (exit 0), NOT FLIGHT-READY (exit 1), INVALID (exit 3):
the sample itself cannot be trusted (streams would not stop before it, a
stream was still draining backlog when it started, or the dashboard never
said hello) and must be re-taken, not reported.
"""
import argparse, base64, json, os, socket, struct, sys, time
from collections import defaultdict

# A stream counts as stopped once no binary frame has arrived for this long.
# A fixed quiet gap alone is not enough (the fleet record has a 40 s gap that
# did not stop a stream), which is why --stop-before also sends enable:false.
QUIET_S = 3.0
STOP_BEFORE_BUDGET_S = 20.0
STOP_AFTER_BUDGET_S = 20.0  # >= the dashboard's 15 s command deadline
HELLO_BUDGET_S = 10.0

class WS:
    def __init__(self, host, port, path="/ws", timeout=10):
        self.s = socket.create_connection((host, port), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.s.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            d = self.s.recv(4096)
            if not d: raise RuntimeError("handshake closed")
            buf += d
        if b"101" not in buf.split(b"\r\n")[0]:
            raise RuntimeError(f"handshake failed: {buf.split(chr(13).encode())[0]}")
        self.buf = buf.split(b"\r\n\r\n", 1)[1]

    def _fill(self, n):
        while len(self.buf) < n:
            d = self.s.recv(65536)
            if not d: raise ConnectionError("closed")
            self.buf += d

    def recv(self):
        """-> (opcode, payload). Handles fragmentation + control frames.

        A frame is consumed only once it is complete: a socket timeout in the
        middle of a frame leaves the buffer intact, so the caller can simply
        retry (the quiet waits of --stop-before/--stop-after rely on timeouts).
        """
        while True:
            self._fill(2)
            op, ln, off = self.buf[0] & 0x0F, self.buf[1] & 0x7F, 2
            if ln == 126:
                self._fill(4); ln = struct.unpack("!H", self.buf[2:4])[0]; off = 4
            elif ln == 127:
                self._fill(10); ln = struct.unpack("!Q", self.buf[2:10])[0]; off = 10
            self._fill(off + ln)
            pay, self.buf = self.buf[off:off + ln], self.buf[off + ln:]
            if op == 0x9:  # ping -> pong
                self.send_raw(0xA, pay); continue
            if op == 0x8: raise ConnectionError("server close")
            if op == 0xA: continue
            return op, pay

    def send_raw(self, op, payload=b""):
        m = os.urandom(4)
        masked = bytes(b ^ m[i % 4] for i, b in enumerate(payload))
        n = len(payload)
        hdr = bytes([0x80 | op])
        if n < 126: hdr += bytes([0x80 | n])
        elif n < 65536: hdr += bytes([0x80 | 126]) + struct.pack("!H", n)
        else: hdr += bytes([0x80 | 127]) + struct.pack("!Q", n)
        self.s.sendall(hdr + m + masked)

    def send_json(self, obj): self.send_raw(0x1, json.dumps(obj).encode())

def pct(v, q):
    if not v: return None
    sv = sorted(v); return round(sv[min(len(sv) - 1, int(q * (len(sv) - 1)))], 2)

def armed_of(m):
    """The vehicle's armed flag from a telemetry message, or None if absent.
    Live telemetry nests it as sample.armed; accept a top-level flag too."""
    s = m.get("sample")
    v = s.get("armed") if isinstance(s, dict) else None
    if v is None: v = m.get("armed")
    return v if isinstance(v, bool) else None

def note_telemetry(m, armed):
    f = armed_of(m)
    if f is not None: armed[m.get("vehicle", "?")] = f

def write_json(path, obj):
    if not path: return
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2, sort_keys=True); f.write("\n")
    os.replace(tmp, path)

def quiesce(ws, vehicles, budget, armed):
    """Disable video on every vehicle; quiet once each vehicle has CONFIRMED the
    disable and no binary frame has arrived for QUIET_S, giving up after
    `budget` seconds. -> record dict.

    Frames stopping at the dashboard is not enough: the dashboard stops
    relaying as soon as it unsubscribes, so a vehicle whose disable was lost
    kept streaming across runs while this reported quiet (fleet 2026-09-23).
    The dashboard re-issues each command until answered and reports the
    outcome as a `command.ok` / `command.failed` event (label "video"); ok
    means the vehicle executed the disable."""
    rec = {"vehicles": list(vehicles), "quiet": False, "waited_s": None,
           "frames_drained": 0, "confirmed": [], "failed": []}
    pending = set(vehicles)
    sent = time.time(); last_frame = sent
    try:
        for v in vehicles:
            ws.send_json({"cmd": "video", "vehicle": v, "params": {"enable": False}})
        while True:
            now = time.time()
            if not pending and now - last_frame >= QUIET_S:
                rec.update(quiet=True, waited_s=round(now - sent, 2)); return rec
            if now - sent >= budget:
                why = (f"stop not confirmed by {','.join(sorted(pending))}" if pending
                       else f"frames still arriving after {budget:.0f}s")
                rec.update(waited_s=round(now - sent, 2), error=why); return rec
            try:
                op, pay = ws.recv()
            except socket.timeout:
                continue
            if op == 0x2:
                if pay: rec["frames_drained"] += 1; last_frame = time.time()
                continue
            try: m = json.loads(pay)
            except Exception: continue
            if m.get("type") == "telemetry": note_telemetry(m, armed)
            if (m.get("type") == "event" and m.get("label") == "video"
                    and m.get("vehicle") in pending):
                if m.get("kind") == "command.ok" and m.get("status", True):
                    pending.discard(m["vehicle"]); rec["confirmed"].append(m["vehicle"])
                elif m.get("kind") == "command.failed":
                    pending.discard(m["vehicle"]); rec["failed"].append(m["vehicle"])
                    rec.update(waited_s=round(time.time() - sent, 2),
                               error=f"stop failed on {m['vehicle']}"); return rec
    except (OSError, ConnectionError) as e:
        rec.update(waited_s=round(time.time() - sent, 2), error=str(e)); return rec

def quiesce_line(name, r):
    if r["quiet"]:
        return (f"{name}: video disabled and confirmed by {','.join(r['confirmed'])}, "
                f"quiet after {r['waited_s']:.1f}s ({r['frames_drained']} frames drained)")
    return f"!! {name}: NOT quiet ({r.get('error')}; {r['frames_drained']} frames drained)"

def probe(a):
    """Passive: who is advertised, who reports telemetry, who is armed.
    The ndn-fleet server asks this before every mutation (PROTOCOL.md I2)."""
    started = time.time()
    res = {"label": a.label, "probe": True, "seconds": a.seconds,
           "started_unix": round(started, 3), "vehicles": [], "telemetry": {},
           "armed": {}, "missing": [], "ok": False}
    try:
        ws = WS(a.host, a.port)
    except (OSError, RuntimeError) as e:
        res["error"] = f"cannot connect to ws://{a.host}:{a.port}/ws: {e}"
        print(f"PROBE FAILED {res['error']}"); write_json(a.json, res); return 2
    ws.s.settimeout(3.0)
    hello = None; tele = defaultdict(int); armed = {}
    while time.time() - started < a.seconds:
        try:
            op, pay = ws.recv()
        except socket.timeout:
            continue
        except (OSError, ConnectionError) as e:
            res["error"] = f"connection lost: {e}"; break
        if op != 0x1: continue
        try: m = json.loads(pay)
        except Exception: continue
        t = m.get("type")
        if t == "hello": hello = m
        elif t == "telemetry":
            tele[m.get("vehicle", "?")] += 1; note_telemetry(m, armed)
    vehicles = list((hello or {}).get("vehicles", []))
    known = vehicles + sorted(v for v in tele if v not in vehicles)
    missing = [v for v in vehicles if not tele.get(v)]
    ok = hello is not None and not missing and "error" not in res
    if hello is None and "error" not in res: res["error"] = "no hello from the dashboard"
    res.update(vehicles=vehicles, missing=missing, ok=ok,
               telemetry={v: {"n": tele[v], "rate": round(tele[v] / a.seconds, 2)} for v in tele},
               armed={v: armed.get(v) for v in known})
    is_armed = [v for v in known if armed.get(v) is True]
    unknown = [v for v in known if armed.get(v) is None]
    print(f"PROBE {'OK' if ok else 'INCOMPLETE'} {a.seconds:.0f}s "
          f"vehicles={','.join(vehicles) or '-'} "
          f"telemetry={len(vehicles) - len(missing)}/{len(vehicles)} "
          f"armed={','.join(is_armed) or 'none'} unknown={','.join(unknown) or 'none'}"
          + (f" missing={','.join(missing)}" if missing else "")
          + (f" error={res['error']}" if "error" in res else ""))
    write_json(a.json, res)
    return 0 if ok else 2

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="minidronesys-03.uom.memphis.edu")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--seconds", type=float, default=None,
                    help="measured window (default 60; 5 with --probe)")
    ap.add_argument("--video", default="", help="vehicle id to enable video on")
    ap.add_argument("--transport", default="stream", choices=["segmented", "stream"])
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--quality", type=int, default=40)
    ap.add_argument("--label", default="run")
    ap.add_argument("--passive", action="store_true", help="report video but never send commands")
    ap.add_argument("--expect", default="",
                    help="comma-separated vehicles that MUST be healthy "
                         "(default: the --video vehicle, else every vehicle "
                         "the dashboard advertises). Vehicles outside this set "
                         "that report nothing are shown as OFFLINE, not failed "
                         "— a powered-down airframe is not a system fault.")
    ap.add_argument("--video-all", action="store_true",
                    help="report video for EVERY vehicle that delivered frames, "
                         "plus an aggregate line. A network should carry as many "
                         "streams as it has capacity for; judging one stream in "
                         "isolation hides both the aggregate capacity and whether "
                         "the streams share it gracefully.")
    ap.add_argument("--audio", default="",
                    help="vehicle to task an AUDIO capture on (the mic airframe, "
                         "e.g. iuas-02). Video is the wrong payload test for a "
                         "node that carries a microphone and a synthetic camera; "
                         "this drives sensor/capture and waits for the result.")
    ap.add_argument("--audio-seconds", type=float, default=6.0,
                    help="duration of the tasked audio capture (default 6)")
    ap.add_argument("--warmup", type=float, default=15.0,
                    help="seconds after the first frame treated as live-stream "
                         "subscription warm-up: gaps there are printed but do "
                         "not fail the verdict (default 15)")
    ap.add_argument("--json", default="", metavar="PATH",
                    help="also write the structured result to PATH (ndn-fleet "
                         "reads this; the text report is for humans)")
    ap.add_argument("--probe", action="store_true",
                    help="passive: collect --seconds of telemetry, report each "
                         "vehicle's armed flag in one line, exit 0 iff every "
                         "advertised vehicle reported telemetry, else 2")
    ap.add_argument("--stop-before", action="store_true",
                    help="after hello, disable video on every vehicle and wait "
                         f"for {QUIET_S:.0f}s without a frame before the measured "
                         "window starts (INVALID if not quiet within "
                         f"{STOP_BEFORE_BUDGET_S:.0f}s): a quiet gap alone does "
                         "not stop a stream, and a sample that starts on "
                         "another run's backlog measures the backlog")
    ap.add_argument("--stop-after", action="store_true",
                    help="at the end, disable video on every vehicle and confirm "
                         f"quiet within {STOP_AFTER_BUDGET_S:.0f}s, so the next "
                         "sample does not start on this one's stream")
    a = ap.parse_args()
    if a.seconds is None: a.seconds = 5.0 if a.probe else 60.0
    if a.passive and (a.stop_before or a.stop_after):
        ap.error("--passive never sends commands; it cannot --stop-before/--stop-after")
    if a.probe: return probe(a)

    started = time.time()
    try:
        ws = WS(a.host, a.port)
    except (OSError, RuntimeError) as e:
        print(f"!! cannot connect to ws://{a.host}:{a.port}/ws: {e}")
        print("===== INVALID =====")
        write_json(a.json, {"label": a.label, "seconds": a.seconds,
                            "started_unix": round(started, 3), "verdict": "INVALID",
                            "invalid_reasons": [f"cannot connect: {e}"]})
        return 3
    ws.s.settimeout(3.0)
    hello = None; armed = {}; invalid = []
    vid_list = [x for x in a.video.split(",") if x]
    stop_before = stop_after = None

    if a.stop_before:
        # The measured window must not begin until every stream is down: a
        # sample once reported 40 fps against a 15 fps request because it was
        # draining the previous run's backlog (ndn-fleet PROTOCOL.md I6).
        t0 = time.time()
        while hello is None and time.time() - t0 < HELLO_BUDGET_S:
            try:
                op, pay = ws.recv()
            except socket.timeout:
                continue
            except (OSError, ConnectionError):
                break
            if op != 0x1: continue
            try: m = json.loads(pay)
            except Exception: continue
            if m.get("type") == "hello":
                hello = m
                print(f"[{time.time()-t0:5.1f}s] hello: vehicles={m.get('vehicles')} "
                      f"caps={{bundle:{m.get('bundle')}, video_transport:{m.get('video_transport')}, sim:{m.get('sim')}}}")
            elif m.get("type") == "telemetry": note_telemetry(m, armed)
        if hello is None:
            stop_before = {"vehicles": [], "quiet": False, "waited_s": None,
                           "frames_drained": 0, "error": f"no hello within {HELLO_BUDGET_S:.0f}s"}
        else:
            targets = list(hello.get("vehicles", [])) + [v for v in vid_list if v not in hello.get("vehicles", [])]
            stop_before = quiesce(ws, targets, STOP_BEFORE_BUDGET_S, armed)
        print(f"[{time.time()-t0:5.1f}s] {quiesce_line('stop-before', stop_before)}")
        if not stop_before["quiet"]:
            invalid.append(f"stop-before: {stop_before.get('error')}")

    # The measured window starts here: nothing received during stop-before counts.
    tele = defaultdict(list); vstats = defaultdict(list)
    frames = defaultdict(list); events = defaultdict(int); frame_bytes = defaultdict(int)
    clock_ms = defaultdict(list); sample_age_ms = defaultdict(list)
    t_start = time.time(); video_sent_at = None
    audio_sent_at = None; audio_result = None

    while time.time() - t_start < a.seconds and not (a.stop_before and not stop_before["quiet"]):
        # enable video ~3s in, once we have hello/vehicle list
        if a.video and not a.passive and video_sent_at is None and time.time() - t_start > 3:
            for _v in vid_list:
                ws.send_json({"cmd": "video", "vehicle": _v, "params": {
                    "enable": True, "transport": a.transport, "width": a.width,
                    "height": int(a.width*3/4), "fps": a.fps, "quality": a.quality}})
            video_sent_at = time.time()
            print(f"[{time.time()-t_start:5.1f}s] -> video enable {','.join(vid_list)} "
                  f"transport={a.transport} fps={a.fps} w={a.width} q={a.quality}")
        # task the mic airframe a few seconds in, after video, so the two
        # payload paths are exercised on the same run
        if a.audio and not a.passive and audio_sent_at is None and time.time() - t_start > 6:
            ws.send_json({"cmd": "sensor", "vehicle": a.audio, "params": {
                "sensor": "audio", "mode": "now", "duration_s": a.audio_seconds}})
            audio_sent_at = time.time()
            print(f"[{time.time()-t_start:5.1f}s] -> audio capture {a.audio} ({a.audio_seconds}s)")
        try:
            op, pay = ws.recv()
        except socket.timeout:
            continue
        except ConnectionError as e:
            print(f"!! connection lost after {time.time()-t_start:.1f}s: {e}"); break
        now = time.time()
        if op == 0x2:  # binary video frame: [vehicleIdx][jpeg]
            if pay:
                frames[pay[0]].append(now); frame_bytes[pay[0]] += len(pay) - 1
            continue
        try: m = json.loads(pay)
        except Exception: continue
        t = m.get("type")
        if t == "hello":
            hello = m
            print(f"[{now-t_start:5.1f}s] hello: vehicles={m.get('vehicles')} "
                  f"caps={{bundle:{m.get('bundle')}, video_transport:{m.get('video_transport')}, sim:{m.get('sim')}}}")
        elif t == "telemetry":
            tele[m.get("vehicle","?")].append(now); note_telemetry(m, armed)
            if m.get("clock_ms") is not None: clock_ms[m.get("vehicle","?")].append(m["clock_ms"])
            if m.get("sample_age_ms") is not None: sample_age_ms[m.get("vehicle","?")].append(m["sample_age_ms"])
        elif t == "video_stats":
            vstats[m.get("vehicle","?")].append((now, m.get("fps"), m.get("kbps"), m.get("lag_ms")))
            print(f"[{now-t_start:5.1f}s] video_stats {m.get('vehicle')}: fps={m.get('fps')} kbps={m.get('kbps')} seq={m.get('seq')} lag_ms={m.get('lag_ms')}")
        elif t == "event":
            k = m.get("kind","?"); events[k] += 1
            if k in ("sensor.result", "sensor.rejected") and m.get("vehicle") == a.audio:
                audio_result = (now - (audio_sent_at or now), dict(m))
            if any(s in k for s in ("video", "fail", "timeout", "error", "stale")):
                print(f"[{now-t_start:5.1f}s] EVENT {k}: { {kk:vv for kk,vv in m.items() if kk not in ('type','kind','t')} }")

    vehicles = (hello or {}).get("vehicles", [])
    if a.stop_after:
        targets = list(vehicles) + [v for v in vid_list if v not in vehicles]
        stop_after = quiesce(ws, targets, STOP_AFTER_BUDGET_S, armed)
        print(f"[{time.time()-t_start:5.1f}s] {quiesce_line('stop-after', stop_after)}")
    if hello is None:
        invalid.append("no hello from the dashboard: the vehicle set is unknown")

    print(f"\n===== FLIGHT CHECK [{a.label}] {a.seconds:.0f}s =====")
    ok = True
    res_tele = {}
    for v, ts in sorted(tele.items()):
        ts.sort(); gaps = [b - a_ for a_, b in zip(ts, ts[1:])]
        rate = len(ts) / a.seconds
        verdict = "OK " if rate >= 2.0 and (not gaps or max(gaps) < 3.0) else "BAD"
        if verdict == "BAD": ok = False
        print(f" {verdict} TELEM {v}: rate={rate:.2f}/s p50={pct(gaps,.5)}s p95={pct(gaps,.95)}s "
              f"max={round(max(gaps),2) if gaps else '-'}s gaps>2s={sum(1 for g in gaps if g>2)}"
              f" clock={pct(clock_ms[v], .5)}ms (|max| {max((abs(c) for c in clock_ms[v]), default='-')}ms)"
              f" sample_age={pct(sample_age_ms[v], .5)}ms")
        res_tele[v] = {"clock_ms_p50": pct(clock_ms[v], .5),
                       "clock_ms_absmax": max((abs(c) for c in clock_ms[v]), default=None),
                       "sample_age_ms_p50": pct(sample_age_ms[v], .5),
                       "sample_age_ms_p95": pct(sample_age_ms[v], .95),"n": len(ts), "rate": round(rate, 3), "gaps_p50": pct(gaps, .5),
                       "gaps_p95": pct(gaps, .95), "gaps_max": round(max(gaps), 2) if gaps else None,
                       "gaps_over_2s": sum(1 for g in gaps if g > 2), "status": verdict.strip()}
        for i, g in enumerate(gaps):
            if g > 2.0:
                print(f"      gap @ t+{ts[i]-t_start:.1f}s for {g:.2f}s")
    expect = [x for x in a.expect.split(",") if x] or (vid_list if a.video else list(vehicles))
    for v in vehicles:
        if v not in tele:
            empty = {"n": 0, "rate": 0.0, "gaps_p50": None, "gaps_p95": None,
                     "gaps_max": None, "gaps_over_2s": 0}
            if v in expect:
                print(f" BAD TELEM {v}: NO DATA"); ok = False
                res_tele[v] = {**empty, "status": "BAD"}
            else:
                print(f" --  TELEM {v}: offline (not in --expect; ignored)")
                res_tele[v] = {**empty, "status": "OFFLINE"}
    res_audio = {}
    if a.audio:
        res_audio = {"vehicle": a.audio, "tasked": audio_sent_at is not None,
                     "status": None, "seconds": None, "ok": False}
        if audio_result is None:
            print(f" BAD AUDIO {a.audio}: no sensor.result "
                  f"({'not tasked' if audio_sent_at is None else 'tasked, no reply'})")
            ok = False
        else:
            dt, m = audio_result
            st = m.get("status") or m.get("reason") or "?"
            res_audio.update(status=st, seconds=round(dt, 2), ok=st == "captured")
            if st == "captured":
                print(f" OK  AUDIO {a.audio}: captured in {dt:.1f}s "
                      f"sensor={m.get('sensor')} at {m.get('lat')},{m.get('lon')}")
            else:
                print(f" BAD AUDIO {a.audio}: status={st} {m.get('message','')}")
                res_audio["message"] = m.get("message", "")
                ok = False
    res_video = {}; res_agg = None
    if vid_list or a.video_all:
        # Report EVERY vehicle that delivered frames, not just the one asked
        # for: the question a shared network has to answer is how much total
        # video it carries and whether concurrent streams share it gracefully,
        # which a single-stream number cannot show.
        report = sorted(set(vid_list) | ({v for v in vehicles
                        if frames.get(vehicles.index(v))} if a.video_all else set()))
        agg_fps = 0.0; agg_kbps = 0.0; worst = 0.0
        for v in report:
            idx = vehicles.index(v) if v in vehicles else -1
            fr = frames.get(idx, [])
            if not fr:
                print(f" BAD VIDEO {v}: ZERO frames received"); ok = False
                res_video[v] = {"n": 0, "fps": 0.0, "kbps": 0.0, "first_frame_s": None,
                                "gap_p50": None, "gap_p95": None, "gap_max": None,
                                "stutters_over_1s": 0, "warmup_stutters": 0,
                                "valid": True, "status": "BAD"}
                continue
            span = fr[-1] - fr[0] if len(fr) > 1 else 1
            gaps = [b - a_ for a_, b in zip(fr, fr[1:])]
            fps = len(fr) / max(span, 0.001)
            kbps = frame_bytes[idx] * 8 / max(span, .001) / 1000
            first = fr[0] - video_sent_at if video_sent_at else -1
            warm = [g for g, t in zip(gaps, fr) if t - fr[0] < a.warmup]
            steady = [g for g, t in zip(gaps, fr) if t - fr[0] >= a.warmup]
            stutter = sum(1 for g in steady if g > 1.0)
            warm_stutter = sum(1 for g in warm if g > 1.0)
            agg_fps += fps; agg_kbps += kbps; worst = max(worst, max(gaps) if gaps else 0)
            verdict = "OK " if fps >= 3.0 and stutter == 0 else "BAD"
            if verdict == "BAD": ok = False
            print(f" {verdict} VIDEO {v}: n={len(fr)} fps={fps:.1f} kbps={kbps:.0f} "
                  f"first_frame={first:.1f}s gap p50={pct(gaps,.5)}s p95={pct(gaps,.95)}s "
                  f"max={round(max(gaps),2) if gaps else '-'}s stutters>1s={stutter}"
                  + (f" (+{warm_stutter} in first {a.warmup:.0f}s warm-up)" if warm_stutter else ""))
            # Backlog guard: a stream we enabled whose first frame precedes
            # that enable is another run's backlog draining, not this sample
            # (the fleet record: 40 fps measured against a 15 fps request).
            valid = not (v in vid_list and video_sent_at is not None and first < 0)
            if not valid:
                print(f" !!  VIDEO {v}: first frame {-first:.1f}s BEFORE its enable "
                      f"(backlog) -- sample rejected")
                invalid.append(f"video {v}: first frame {-first:.1f}s before its enable (backlog)")
            res_video[v] = {"n": len(fr), "fps": round(fps, 2), "kbps": round(kbps, 1),
                            "first_frame_s": round(first, 2) if video_sent_at else None,
                            "gap_p50": pct(gaps, .5), "gap_p95": pct(gaps, .95),
                            "gap_max": round(max(gaps), 2) if gaps else None,
                            "stutters_over_1s": stutter, "warmup_stutters": warm_stutter,
                            # how far behind live the dashboard fell (its lag-triggered
                            # resubscribe fires at 1.5 s); None from a pre-lag dashboard
                            "lag_ms_max": max((x[3] for x in vstats.get(v, []) if x[3] is not None),
                                              default=None),
                            "valid": valid, "status": verdict.strip()}
            for i, g in enumerate(gaps):
                if g > 1.0:
                    tag = "warm-up" if fr[i] - fr[0] < a.warmup else "STALL  "
                    print(f"      {tag} @ t+{fr[i]-t_start:.1f}s for {g:.2f}s (resumed t+{fr[i+1]-t_start:.1f}s)")
        if len(report) > 1:
            print(f" ==  VIDEO AGGREGATE: {len(report)} streams, {agg_fps:.1f} fps total, "
                  f"{agg_kbps:.0f} kbps total, worst single gap {worst:.2f}s")
        res_agg = {"streams": len(report), "fps": round(agg_fps, 2),
                   "kbps": round(agg_kbps, 1), "worst_gap": round(worst, 2)}
    if events: print(f" events: {dict(sorted(events.items(), key=lambda kv:-kv[1])[:8])}")
    if invalid:
        print(f" !! INVALID: {'; '.join(invalid)}")
    final = "INVALID" if invalid else ("FLIGHT-READY" if ok else "NOT FLIGHT-READY")
    print(f"===== {final} =====")
    write_json(a.json, {
        "label": a.label, "seconds": a.seconds, "started_unix": round(t_start, 3),
        "vehicles": list(vehicles), "expect": expect,
        "params": {"video": vid_list, "transport": a.transport, "width": a.width,
                   "height": int(a.width * 3 / 4), "fps": a.fps, "quality": a.quality,
                   "audio": a.audio, "video_all": a.video_all, "warmup": a.warmup},
        "video_enabled_s": round(video_sent_at - t_start, 2) if video_sent_at else None,
        "telemetry": res_tele, "video": res_video, "aggregate": res_agg,
        "audio": res_audio, "events": dict(events),
        "armed": {v: armed.get(v) for v in list(vehicles) + sorted(set(armed) - set(vehicles))},
        "stop_before": stop_before, "stop_after": stop_after,
        "invalid_reasons": invalid, "verdict": final})
    return 0 if final == "FLIGHT-READY" else (3 if invalid else 1)

if __name__ == "__main__":
    sys.exit(main())
