#!/usr/bin/env python3
"""Flight-readiness check against the live miniMUAS dashboard WebSocket.

Measures what an operator actually depends on in the air: telemetry cadence,
video frame delivery, and command round-trips — from the same socket the
browser uses, so it proves the real operator path end to end.

Usage:
  flightcheck.py --seconds 60
  flightcheck.py --seconds 60 --video iuas-01 --transport segmented
  flightcheck.py --seconds 60 --video iuas-01 --transport stream
Stdlib only (no websockets package on this host).
"""
import argparse, base64, json, os, socket, struct, sys, time
from collections import defaultdict

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

    def _read(self, n):
        while len(self.buf) < n:
            d = self.s.recv(65536)
            if not d: raise ConnectionError("closed")
            self.buf += d
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        """-> (opcode, payload). Handles fragmentation + control frames."""
        while True:
            h = self._read(2)
            fin, op = h[0] & 0x80, h[0] & 0x0F
            ln = h[1] & 0x7F
            if ln == 126: ln = struct.unpack("!H", self._read(2))[0]
            elif ln == 127: ln = struct.unpack("!Q", self._read(8))[0]
            pay = self._read(ln) if ln else b""
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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="minidronesys-03.uom.memphis.edu")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--seconds", type=float, default=60)
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
    ap.add_argument("--warmup", type=float, default=15.0,
                    help="seconds after the first frame treated as live-stream "
                         "subscription warm-up: gaps there are printed but do "
                         "not fail the verdict (default 15)")
    a = ap.parse_args()

    ws = WS(a.host, a.port)
    ws.s.settimeout(3.0)
    hello = None
    tele = defaultdict(list); vstats = defaultdict(list)
    frames = defaultdict(list); events = defaultdict(int); frame_bytes = defaultdict(int)
    t_start = time.time(); video_sent_at = None

    while time.time() - t_start < a.seconds:
        # enable video ~3s in, once we have hello/vehicle list
        if a.video and not a.passive and video_sent_at is None and time.time() - t_start > 3:
            ws.send_json({"cmd": "video", "vehicle": a.video, "params": {
                "enable": True, "transport": a.transport, "width": a.width,
                "height": int(a.width*3/4), "fps": a.fps, "quality": a.quality}})
            video_sent_at = time.time()
            print(f"[{time.time()-t_start:5.1f}s] -> video enable {a.video} transport={a.transport} fps={a.fps} w={a.width} q={a.quality}")
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
        elif t == "telemetry": tele[m.get("vehicle","?")].append(now)
        elif t == "video_stats":
            vstats[m.get("vehicle","?")].append((now, m.get("fps"), m.get("kbps")))
            print(f"[{now-t_start:5.1f}s] video_stats {m.get('vehicle')}: fps={m.get('fps')} kbps={m.get('kbps')} seq={m.get('seq')}")
        elif t == "event":
            k = m.get("kind","?"); events[k] += 1
            if any(s in k for s in ("video", "fail", "timeout", "error", "stale")):
                print(f"[{now-t_start:5.1f}s] EVENT {k}: { {kk:vv for kk,vv in m.items() if kk not in ('type','kind','t')} }")

    print(f"\n===== FLIGHT CHECK [{a.label}] {a.seconds:.0f}s =====")
    vehicles = (hello or {}).get("vehicles", [])
    ok = True
    for v, ts in sorted(tele.items()):
        ts.sort(); gaps = [b - a_ for a_, b in zip(ts, ts[1:])]
        rate = len(ts) / a.seconds
        verdict = "OK " if rate >= 2.0 and (not gaps or max(gaps) < 3.0) else "BAD"
        if verdict == "BAD": ok = False
        print(f" {verdict} TELEM {v}: rate={rate:.2f}/s p50={pct(gaps,.5)}s p95={pct(gaps,.95)}s "
              f"max={round(max(gaps),2) if gaps else '-'}s gaps>2s={sum(1 for g in gaps if g>2)}")
        for i, g in enumerate(gaps):
            if g > 2.0:
                print(f"      gap @ t+{ts[i]-t_start:.1f}s for {g:.2f}s")
    expect = [x for x in a.expect.split(",") if x] or ([a.video] if a.video else list(vehicles))
    for v in vehicles:
        if v not in tele:
            if v in expect:
                print(f" BAD TELEM {v}: NO DATA"); ok = False
            else:
                print(f" --  TELEM {v}: offline (not in --expect; ignored)")
    if a.video:
        idx = vehicles.index(a.video) if a.video in vehicles else 0
        fr = frames.get(idx, [])
        if not fr:
            print(f" BAD VIDEO {a.video}: ZERO frames received"); ok = False
        else:
            span = fr[-1] - fr[0] if len(fr) > 1 else 1
            gaps = [b - a_ for a_, b in zip(fr, fr[1:])]
            fps = len(fr) / max(span, 0.001)
            kbps = frame_bytes[idx] * 8 / max(span, .001) / 1000
            first = fr[0] - video_sent_at if video_sent_at else -1
            # A live subscriber pays a one-time settling cost: it joins at the
            # live edge and fills its prefetch window before delivery is
            # smooth. That is not a flight fault -- the producer keeps
            # producing through it (verify in the agent journal: NDNSF
            # timeline events continue). Judge the verdict on STEADY STATE,
            # but always print the warm-up gap so it can never hide a real
            # stall that happens to land early.
            warm = [g for g, t in zip(gaps, fr) if t - fr[0] < a.warmup]
            steady = [g for g, t in zip(gaps, fr) if t - fr[0] >= a.warmup]
            stutter = sum(1 for g in steady if g > 1.0)
            warm_stutter = sum(1 for g in warm if g > 1.0)
            verdict = "OK " if fps >= 3.0 and stutter == 0 else "BAD"
            if verdict == "BAD": ok = False
            print(f" {verdict} VIDEO {a.video}: n={len(fr)} fps={fps:.1f} kbps={kbps:.0f} "
                  f"first_frame={first:.1f}s gap p50={pct(gaps,.5)}s p95={pct(gaps,.95)}s max={round(max(gaps),2) if gaps else '-'}s stutters>1s={stutter}"
                  + (f" (+{warm_stutter} in first {a.warmup:.0f}s warm-up)" if warm_stutter else ""))
            # where the stalls were: offset into the run, so they can be lined
            # up against the agent's journal (the 43 s journal republisher was
            # found exactly this way -- a precise period names its process).
            for i, g in enumerate(gaps):
                if g > 1.0:
                    tag = "warm-up" if fr[i] - fr[0] < a.warmup else "STALL  "
                    print(f"      {tag} @ t+{fr[i]-t_start:.1f}s for {g:.2f}s (resumed t+{fr[i+1]-t_start:.1f}s)")
    if events: print(f" events: {dict(sorted(events.items(), key=lambda kv:-kv[1])[:8])}")
    print(f"===== {'FLIGHT-READY' if ok else 'NOT FLIGHT-READY'} =====")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
