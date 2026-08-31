"""Limit-activation tracking: make every silent safety clamp visible.

The teleop stack clamps commands in many places (workspace box, per-tick
velocity caps, pitch guard, deviation tether, IK joint limits/steps). All of
them are silent by design — the arm just stops following, and the operator
cannot tell which guard is holding it back. Control loops call hit() when a
guard actually alters a command; UI threads call snapshot()/active_str() to
display which limits fired, how often, and what they clamped.
"""

from __future__ import annotations

import threading
import time


class LimitTracker:
    """Thread-safe record of limit-guard activations.

    A limit reads as *active* for hold_s after its last hit, so events from a
    100 Hz control loop stay visible at UI poll rates (a few Hz)."""

    def __init__(self, hold_s: float = 0.8):
        self.hold_s = float(hold_s)
        self._lock = threading.Lock()
        self._events: dict[str, list] = {}  # name -> [count, last_mono, detail]

    def hit(self, name: str, detail: str = "") -> None:
        now = time.monotonic()
        with self._lock:
            ev = self._events.get(name)
            if ev is None:
                self._events[name] = [1, now, detail]
            else:
                ev[0] += 1
                ev[1] = now
                ev[2] = detail

    def snapshot(self) -> dict:
        """{name: {"count": int, "active": bool, "detail": str}} (JSON-able)."""
        now = time.monotonic()
        with self._lock:
            return {n: {"count": c, "active": (now - t) < self.hold_s, "detail": d}
                    for n, (c, t, d) in self._events.items()}

    def active_str(self) -> str:
        """Compact one-liner of currently-active limits, e.g. 'ws:x+ z- pitch'."""
        parts = []
        for name, s in self.snapshot().items():
            if s["active"]:
                parts.append(f"{name}:{s['detail']}" if s["detail"] else name)
        return " ".join(parts)


def merged_active_str(groups: dict) -> str:
    """Flatten {group: snapshot} into one active-limits string."""
    parts = []
    for snap in groups.values():
        for name, s in snap.items():
            if s["active"]:
                parts.append(f"{name}:{s['detail']}" if s["detail"] else name)
    return " ".join(parts)
