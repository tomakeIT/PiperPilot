"""Stream camera frames to the Quest headset as MJPEG over a DEDICATED TCP
connection (port 8736 via `adb forward`), shown in-headset as floating
panels over passthrough.

Priority design: the pose/action stream (port 8735) is never touched —
separate socket, separate adb forward. This sender is non-blocking and
latest-frame-only: if the link back-pressures, frames are DROPPED, never
queued, so video can only reduce its own frame rate, not add latency
elsewhere.

Framing (little endian): [u32 magic 'PCV1'][u8 cam][u8 pad*3][u32 len][JPEG]
"""

from __future__ import annotations

import socket
import struct
import threading
import time

import cv2

from .cameras import CameraBase

VIDEO_MAGIC = 0x31564350  # 'PCV1'
MAX_CAM_PANELS = 3        # quest app panel layout: 0..2 cameras, 3 = HUD
STATUS_PANEL_IDX = 3


class QuestVideoStreamer:
    def __init__(self, cameras: list[CameraBase], host: str = "127.0.0.1",
                 port: int = 8736, fps: float = 15.0, quality: int = 70,
                 max_width: int = 640, status_fn=None):
        """status_fn: optional () -> RGB ndarray; sent ~5 Hz as the HUD panel."""
        self.cameras = cameras[:MAX_CAM_PANELS]
        self.status_fn = status_fn
        self.host = host
        self.port = port
        self.period = 1.0 / fps
        self.quality = int(quality)
        self.max_width = int(max_width)
        self._sock: socket.socket | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_ids: dict[str, int] = {}
        self._last_status = 0.0
        self.frames_sent = 0
        self.frames_dropped = 0

    def start(self) -> None:
        if not self.cameras and self.status_fn is None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="quest-video", daemon=True)
        self._thread.start()
        print(f"[quest-video] streaming {len(self.cameras)} camera(s) "
              f"@{1 / self.period:.0f}fps q{self.quality} -> {self.host}:{self.port}")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @staticmethod
    def _draw_label(bgr, name: str) -> None:
        """Camera name banner across the top of the frame (travels with the
        video, so the headset needs no text rendering)."""
        h = max(26, bgr.shape[0] // 16)
        bgr[:h] = (bgr[:h] * 0.35).astype(bgr.dtype)  # darken banner strip
        cv2.putText(bgr, name, (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    h / 40.0, (255, 255, 255), 2, cv2.LINE_AA)

    # -- internals ----------------------------------------------------------

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Small send buffer: back-pressure surfaces immediately and we
            # drop frames instead of building a latency queue.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
            sock.setblocking(False)
            self._sock = sock
            return True
        except OSError:
            self._sock = None
            return False

    def _send_frame(self, cam_idx: int, jpeg: bytes) -> bool:
        """Non-blocking all-or-nothing send; False = dropped/link dead."""
        if self._sock is None:
            return False
        payload = struct.pack("<IB3xI", VIDEO_MAGIC, cam_idx, len(jpeg)) + jpeg
        try:
            sent = self._sock.send(payload)
        except BlockingIOError:
            self.frames_dropped += 1
            return True  # link alive, just congested — drop this frame
        except OSError:
            self._sock.close()
            self._sock = None
            return False
        # Partial header/frame must be completed or the stream desyncs:
        # finish it with short blocking sends (bounded by SO_SNDBUF).
        while sent < len(payload):
            try:
                self._sock.setblocking(True)
                self._sock.settimeout(1.0)
                sent += self._sock.send(payload[sent:])
            except OSError:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
                return False
            finally:
                if self._sock is not None:
                    self._sock.setblocking(False)
        self.frames_sent += 1
        return True

    def _loop(self) -> None:
        next_t = time.monotonic()
        while self._running:
            if self._sock is None:
                if not self._connect():
                    time.sleep(1.0)
                    continue
                print("[quest-video] connected")
            next_t = max(next_t + self.period, time.monotonic() - self.period)
            for idx, cam in enumerate(self.cameras):
                fr = cam.latest()
                if fr is None or self._last_ids.get(cam.name) == fr.frame_id:
                    continue
                self._last_ids[cam.name] = fr.frame_id
                img = fr.rgb
                if img.shape[1] > self.max_width:
                    scale = self.max_width / img.shape[1]
                    img = cv2.resize(img, (self.max_width, int(img.shape[0] * scale)))
                bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self._draw_label(bgr, cam.name)
                ok, jpeg = cv2.imencode(
                    ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    continue
                if not self._send_frame(idx, jpeg.tobytes()):
                    break  # reconnect on next loop iteration

            # Status HUD -> STATUS_PANEL_IDX at ~5 Hz.
            if (self.status_fn is not None and self._sock is not None
                    and time.monotonic() - self._last_status > 0.2):
                self._last_status = time.monotonic()
                try:
                    img = self.status_fn()
                except Exception:
                    img = None
                if img is not None:
                    ok, jpeg = cv2.imencode(
                        ".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok:
                        self._send_frame(STATUS_PANEL_IDX, jpeg.tobytes())

            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
