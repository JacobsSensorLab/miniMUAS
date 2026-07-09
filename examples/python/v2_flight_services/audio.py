"""Audio capture sources for the IUAS microphone capability.

An IUAS carrying a microphone advertises the `audio` sensor in its
capability profile; an investigation whose `sensor_plan` includes
`audio` records a WAV clip from the orbit and publishes it as a mission
artifact (same frame container as camera captures, kind="audio/wav").

`--audio <spec>` selects the source:

    none                 no microphone; the agent does not advertise audio
    synthetic[:hz]       generated test tone (default 880 Hz) — bench use,
                         no hardware or extra dependencies
    alsa:<device>[?rate=16000]
                         real capture via `arecord` (alsa-utils must be on
                         PATH). Device is an ALSA name, e.g. "default",
                         "hw:1,0", or "plughw:CARD=Device,DEV=0".

Both sources share one method:

    record_wav(seconds) -> bytes     complete WAV file contents
"""

from __future__ import annotations

import io
import math
import struct
import subprocess
import wave
from typing import Any


class AudioSourceError(RuntimeError):
    """An audio source could not be created or could not record."""


class SyntheticAudioSource:
    """Deterministic sine-tone WAV; the no-hardware bench default."""

    def __init__(self, tone_hz: float = 880.0, rate: int = 16000) -> None:
        self.spec = f"synthetic:{tone_hz:g}"
        self._tone_hz = float(tone_hz)
        self._rate = int(rate)

    def record_wav(self, seconds: float) -> bytes:
        n = max(1, int(self._rate * float(seconds)))
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(self._rate)
            amp = 0.3 * 32767
            w.writeframes(b"".join(
                struct.pack(
                    "<h",
                    int(amp * math.sin(
                        2.0 * math.pi * self._tone_hz * i / self._rate
                    )),
                )
                for i in range(n)
            ))
        return buf.getvalue()

    def describe(self) -> dict[str, Any]:
        return {"audio": "synthetic", "tone_hz": self._tone_hz}

    def close(self) -> None:
        pass


class AlsaAudioSource:
    """Real capture through `arecord` (mono 16-bit WAV on stdout)."""

    def __init__(self, spec: str) -> None:
        self.spec = f"alsa:{spec}"
        device, params = spec, {}
        if "?" in spec:
            device, query = spec.split("?", 1)
            for pair in query.split("&"):
                if "=" in pair:
                    key, value = pair.split("=", 1)
                    params[key.strip()] = value.strip()
        self._device = device or "default"
        self._rate = int(params.get("rate", 16000))
        # fail at construction, not mid-mission, when arecord is absent
        try:
            subprocess.run(
                ["arecord", "--version"], capture_output=True, timeout=5
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise AudioSourceError(
                "arecord not runnable (install alsa-utils)"
            ) from exc

    def record_wav(self, seconds: float) -> bytes:
        seconds = max(1.0, float(seconds))
        cmd = [
            "arecord", "-q",
            "-D", self._device,
            "-f", "S16_LE",
            "-r", str(self._rate),
            "-c", "1",
            "-d", str(int(math.ceil(seconds))),
            "-t", "wav",
            "-",  # stdout
        ]
        try:
            proc = subprocess.run(
                cmd, capture_output=True, timeout=seconds + 10.0
            )
        except subprocess.TimeoutExpired as exc:
            raise AudioSourceError(f"arecord timed out on {self._device}") from exc
        if proc.returncode != 0 or not proc.stdout:
            raise AudioSourceError(
                f"arecord failed on {self._device}: "
                f"{proc.stderr.decode(errors='replace').strip()[:200]}"
            )
        return proc.stdout

    def describe(self) -> dict[str, Any]:
        return {"audio": "alsa", "device": self._device, "rate": self._rate}

    def close(self) -> None:
        pass


def audio_source_from_spec(spec: str | None):
    """Build an audio source from an `--audio` spec; None for `none`."""

    spec = (spec or "none").strip()
    if spec in ("", "none"):
        return None
    if spec == "synthetic" or spec.startswith("synthetic:"):
        tone = spec.partition(":")[2]
        return SyntheticAudioSource(tone_hz=float(tone) if tone else 880.0)
    if spec.startswith("alsa:"):
        return AlsaAudioSource(spec[len("alsa:"):])
    raise AudioSourceError(
        f"unknown audio spec {spec!r} "
        f"(expected none, synthetic[:hz], or alsa:<device>)"
    )


if __name__ == "__main__":
    import json
    import sys

    specs = sys.argv[1:] or ["synthetic"]
    for s in specs:
        try:
            src = audio_source_from_spec(s)
            if src is None:
                print(json.dumps({"spec": s, "ok": True, "audio": "none"}))
                continue
            data = src.record_wav(1.0)
            print(json.dumps({
                "spec": s, "ok": True, **src.describe(), "bytes": len(data),
            }))
        except Exception as exc:
            print(json.dumps({"spec": s, "ok": False, "error": str(exc)}))
