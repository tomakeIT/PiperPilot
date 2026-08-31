"""3Dconnexion SpaceMouse input — direct HID read (python `hidapi` package).

No spacenavd / libhidapi-via-easyhid needed: we parse the HID reports
ourselves (same approach as robosuite/DROID). Report layout (verified against
pyspacemouse device specs for the SpaceMouse Compact, axis_scale = 350):

    report id 1: translation  x=int16[1:3], y=int16[3:5], z=int16[5:7]
                 (wireless models pack trans+rot into one 13-byte report)
    report id 2: rotation     pitch=int16[1:3], roll=int16[3:5], yaw=int16[5:7]
    report id 3: buttons      byte 1 bitmask (bit0=LEFT, bit1=RIGHT)

Normalized state axes (all in [-1, 1]):
    x: puck right   y: puck forward (away)   z: puck pulled up
    roll/pitch/yaw: puck tilts/twist

Device access needs a udev rule for hidraw (install/04_spacemouse_setup.sh).
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np

AXIS_SCALE = 350.0

# vid, pid -> name; all share the report layout above.
SUPPORTED = {
    (0x256F, 0xC635): "SpaceMouse Compact",
    (0x256F, 0xC62E): "SpaceMouse Wireless (cabled)",
    (0x256F, 0xC63A): "SpaceMouse Wireless (new)",
    (0x256F, 0xC652): "3Dconnexion Universal Receiver",
    (0x046D, 0xC626): "SpaceNavigator",
    (0x046D, 0xC628): "SpaceNavigator for Notebooks",
}

# Sign conventions from the pyspacemouse SpaceMouseCompact spec.
TRANS_SIGNS = (1.0, -1.0, -1.0)   # raw x, y, z
ROT_SIGNS = (-1.0, -1.0, 1.0)     # raw pitch, roll, yaw (report order!)


def parse_report(data: bytes, twist: np.ndarray, buttons: list) -> bool:
    """Update twist [x,y,z,roll,pitch,yaw] / buttons from one HID report.
    Returns True if anything was updated."""
    if not data:
        return False
    rid = data[0]
    if rid == 1 and len(data) >= 13:
        # Wireless: combined translation + rotation.
        vals = struct.unpack_from("<6h", bytes(data), 1)
        for i in range(3):
            twist[i] = np.clip(vals[i] * TRANS_SIGNS[i] / AXIS_SCALE, -1, 1)
        pitch, roll, yaw = vals[3], vals[4], vals[5]
        twist[3] = np.clip(roll * ROT_SIGNS[1] / AXIS_SCALE, -1, 1)
        twist[4] = np.clip(pitch * ROT_SIGNS[0] / AXIS_SCALE, -1, 1)
        twist[5] = np.clip(yaw * ROT_SIGNS[2] / AXIS_SCALE, -1, 1)
        return True
    if rid == 1 and len(data) >= 7:
        vals = struct.unpack_from("<3h", bytes(data), 1)
        for i in range(3):
            twist[i] = np.clip(vals[i] * TRANS_SIGNS[i] / AXIS_SCALE, -1, 1)
        return True
    if rid == 2 and len(data) >= 7:
        pitch, roll, yaw = struct.unpack_from("<3h", bytes(data), 1)
        twist[3] = np.clip(roll * ROT_SIGNS[1] / AXIS_SCALE, -1, 1)
        twist[4] = np.clip(pitch * ROT_SIGNS[0] / AXIS_SCALE, -1, 1)
        twist[5] = np.clip(yaw * ROT_SIGNS[2] / AXIS_SCALE, -1, 1)
        return True
    if rid == 3 and len(data) >= 2:
        mask = data[1]
        buttons[0] = (mask >> 0) & 1
        buttons[1] = (mask >> 1) & 1
        return True
    return False


@dataclass
class SpaceMouseState:
    twist: np.ndarray = field(default_factory=lambda: np.zeros(6))  # x,y,z,roll,pitch,yaw
    buttons: tuple = (0, 0)
    mono: float = 0.0


class SpaceMouseReader:
    """Background HID reader; exposes the latest normalized twist + buttons."""

    def __init__(self, device: str = ""):
        import hid

        target = None
        for d in hid.enumerate():
            key = (d["vendor_id"], d["product_id"])
            if key in SUPPORTED and (not device or SUPPORTED[key] == device):
                target = d
                break
        if target is None:
            found = {(hex(d['vendor_id']), hex(d['product_id'])) for d in hid.enumerate()}
            raise RuntimeError(
                f"no supported SpaceMouse found (HID devices: {sorted(found)}). "
                "Is it plugged in?")

        self.name = SUPPORTED[(target["vendor_id"], target["product_id"])]
        self._dev = hid.device()
        try:
            self._dev.open_path(target["path"])
        except OSError as e:
            raise RuntimeError(
                f"cannot open {self.name}: {e}. Missing hidraw permissions — "
                "run install/04_spacemouse_setup.sh (udev rule), then re-plug "
                "the device. If spacenavd is running, stop it: "
                "sudo systemctl stop spacenavd") from e
        self._dev.set_nonblocking(1)
        print(f"[spacemouse] opened: {self.name}")

        self._lock = threading.Lock()
        self._twist = np.zeros(6)
        self._buttons = [0, 0]
        self._mono = 0.0
        self._times: list[float] = []
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="spacemouse", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            got_any = False
            for _ in range(8):  # drain queued reports
                data = self._dev.read(64)
                if not data:
                    break
                with self._lock:
                    if parse_report(data, self._twist, self._buttons):
                        got_any = True
                        self._mono = time.monotonic()
            if got_any:
                with self._lock:
                    self._times.append(self._mono)
                    if len(self._times) > 200:
                        self._times = self._times[-200:]
            time.sleep(0.002)

    def get(self) -> SpaceMouseState:
        with self._lock:
            return SpaceMouseState(twist=self._twist.copy(),
                                   buttons=tuple(self._buttons), mono=self._mono)

    def age(self) -> float:
        """Time since last report. NOTE: the device only reports on activity,
        so treat 'stale' as 'idle', not as an error."""
        with self._lock:
            if self._mono == 0.0:
                return float("inf")
            return time.monotonic() - self._mono

    def rate_hz(self) -> float:
        with self._lock:
            t = self._times
            if len(t) < 2:
                return 0.0
            dt = t[-1] - t[0]
            return (len(t) - 1) / dt if dt > 0 else 0.0

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        try:
            self._dev.close()
        except Exception:
            pass
