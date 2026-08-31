"""Camera capture: Intel RealSense color streams (+ fake source for dry runs).

Each camera runs its own grab thread and exposes the latest RGB frame with a
host-monotonic timestamp and an incrementing frame id (so consumers can
detect new frames without re-encoding duplicates).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    rgb: np.ndarray          # (H, W, 3) uint8
    mono: float              # host time.monotonic() at arrival
    frame_id: int


class CameraBase:
    name: str = "cam"

    def latest(self) -> Frame | None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError


class RealSenseCamera(CameraBase):
    def __init__(self, name: str, serial: str, width: int, height: int, fps: int):
        import pyrealsense2 as rs

        self.name = name
        self.width, self.height, self.fps = width, height, fps
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        if serial:
            cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        profile = self._pipeline.start(cfg)
        dev = profile.get_device()
        self.serial = dev.get_info(rs.camera_info.serial_number)
        print(f"[camera] {name}: RealSense {self.serial} {width}x{height}@{fps}")

        self._lock = threading.Lock()
        self._frame: Frame | None = None
        self._running = True
        self._n = 0
        self._thread = threading.Thread(target=self._loop, name=f"cam-{name}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        import pyrealsense2 as rs  # noqa: F401  (thread-local import safety)

        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError:
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            rgb = np.asanyarray(color.get_data()).copy()
            with self._lock:
                self._n += 1
                self._frame = Frame(rgb=rgb, mono=time.monotonic(), frame_id=self._n)

    def latest(self) -> Frame | None:
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        try:
            self._pipeline.stop()
        except Exception:
            pass


class FakeCamera(CameraBase):
    """Synthetic moving pattern, for --sim testing without hardware."""

    def __init__(self, name: str, width: int = 640, height: int = 480, fps: int = 30):
        self.name = name
        self.width, self.height, self.fps = width, height, fps
        self.serial = f"fake-{name}"
        self._lock = threading.Lock()
        self._frame: Frame | None = None
        self._running = True
        self._n = 0
        self._thread = threading.Thread(target=self._loop, name=f"cam-{name}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        t0 = time.monotonic()
        yy, xx = np.mgrid[0 : self.height, 0 : self.width].astype(np.float32)
        period = 1.0 / self.fps
        while self._running:
            t = time.monotonic() - t0
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            img[..., 0] = (127 + 127 * np.sin(xx / 40.0 + 3 * t)).astype(np.uint8)
            img[..., 1] = (127 + 127 * np.sin(yy / 40.0 + 2 * t)).astype(np.uint8)
            img[..., 2] = int(127 + 127 * np.sin(t))
            with self._lock:
                self._n += 1
                self._frame = Frame(rgb=img, mono=time.monotonic(), frame_id=self._n)
            time.sleep(period)

    def latest(self) -> Frame | None:
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)


def list_realsense_serials() -> list[str]:
    import pyrealsense2 as rs

    return [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]


def make_cameras(cfg, fake: bool = False) -> list[CameraBase]:
    cams: list[CameraBase] = []
    cam_cfgs = cfg.to_dict().get("cameras", [])
    if fake:
        return [FakeCamera(c["name"], c["width"], c["height"], c["fps"]) for c in cam_cfgs]

    available = list_realsense_serials()
    used: set[str] = set()
    for c in cam_cfgs:
        serial = c.get("serial") or ""
        if not serial:
            free = [s for s in available if s not in used]
            if not free:
                print(f"[camera] WARNING: no RealSense left for '{c['name']}', skipping")
                continue
            serial = free[0]
        used.add(serial)
        cams.append(RealSenseCamera(c["name"], serial, c["width"], c["height"], c["fps"]))
    return cams
