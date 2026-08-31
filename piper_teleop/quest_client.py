"""TCP client for the Quest teleop APK stream (see quest_app/).

Connects to 127.0.0.1:8735 (routed to the headset by `adb forward`),
maintains the latest controller/head state on a background thread, and can
send haptic / background-color commands back to the headset.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class HandState:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    valid: bool = False
    tracked: bool = False
    trigger: float = 0.0
    squeeze: float = 0.0
    stick: np.ndarray = field(default_factory=lambda: np.zeros(2))
    stick_click: bool = False
    primary: bool = False    # X (left) / A (right)
    secondary: bool = False  # Y (left) / B (right)
    menu: bool = False

    @classmethod
    def from_json(cls, d: dict) -> "HandState":
        return cls(
            pos=np.array(d.get("pos", [0, 0, 0]), dtype=float),
            quat_xyzw=np.array(d.get("quat", [0, 0, 0, 1]), dtype=float),
            valid=bool(d.get("valid", False)),
            tracked=bool(d.get("tracked", False)),
            trigger=float(d.get("trigger", 0.0)),
            squeeze=float(d.get("squeeze", 0.0)),
            stick=np.array(d.get("stick", [0, 0]), dtype=float),
            stick_click=bool(d.get("stick_click", False)),
            primary=bool(d.get("primary", False)),
            secondary=bool(d.get("secondary", False)),
            menu=bool(d.get("menu", False)),
        )


@dataclass
class QuestState:
    t: float = 0.0            # headset predicted display time (s)
    mono: float = 0.0         # headset CLOCK_MONOTONIC (s)
    frame: int = 0
    session_state: str = "UNKNOWN"
    space: str = "stage"      # reporting frame: "anchor" | "stage" | "local"
    anchor_have: bool = False     # a persistent anchor is saved on the headset
    anchor_tracked: bool = False  # ...and poses are pinned to it this frame
    calib_step: int = 0       # X-axis calibration: 0=off, 1/2=awaiting click N
                              # (hand state is masked by the app while active)
    head: HandState = field(default_factory=HandState)
    left: HandState = field(default_factory=HandState)
    right: HandState = field(default_factory=HandState)
    recv_mono: float = 0.0    # host time.monotonic() at receipt

    def hand(self, side: str) -> HandState:
        return self.left if side == "left" else self.right


class QuestReader:
    """Background reader with auto-reconnect. Thread-safe latest-state access."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8735, on_line=None,
                 stall_reconnect_s: float = 6.0):
        self.host = host
        self.port = port
        # Reconnect after this many seconds with NO bytes at all: an adb/USB
        # hiccup leaves the TCP connection "open" but silent, and recv() then
        # times out forever without ever tripping the reconnect path — the
        # input-0.0Hz mid-episode stalls. A fresh connect re-establishes the
        # adb-forwarded stream instead of waiting the hiccup out.
        self.stall_reconnect_s = stall_reconnect_s
        self._on_line = on_line  # optional raw-line callback (recording/debug)
        # Whole-session stall census: every pose gap > stall_log_threshold_s
        # (default = the 0.25s stale_timeout that freezes the arm) is appended
        # to stall_log_path with a connection-event trail, and stop() prints a
        # summary. Episode raw dumps only cover recording time — this covers
        # the idle gaps between episodes where "start recording ignored"
        # complaints live, and survives episode deletion.
        self.stall_log_threshold_s = 0.25
        self.stall_log_path = time.strftime('/tmp/quest_stalls_%Y%m%d_%H%M%S.log')
        self._last_pose_mono: float | None = None
        self._stall_counts = {'0.25-1s': 0, '1-6s': 0, '>6s': 0}
        self._stall_max = 0.0
        self._t_start = time.monotonic()
        self._sock: socket.socket | None = None
        self._sock_lock = threading.Lock()
        self._state: QuestState | None = None
        self._state_lock = threading.Lock()
        self._hello: dict | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._n_frames = 0
        self._rate_window: list[float] = []
        self.connected = False

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        self._running = True
        self._stall_log(f'session start (pid {__import__("os").getpid()})')
        print(f'[quest] stall log: {self.stall_log_path}')
        self._thread = threading.Thread(target=self._run, name="quest-reader", daemon=True)
        self._thread.start()

    def _stall_log(self, msg: str) -> None:
        try:
            with open(self.stall_log_path, 'a') as f:
                f.write(f'{time.strftime("%H:%M:%S")} +{time.monotonic() - self._t_start:8.1f}s  {msg}\n')
        except OSError:
            pass

    def stall_summary(self) -> str:
        n = sum(self._stall_counts.values())
        return (f'{n} pose stalls >{self.stall_log_threshold_s}s '
                f'({self._stall_counts}), longest {self._stall_max:.1f}s')

    def stop(self) -> None:
        summary = self.stall_summary()
        self._stall_log('session end: ' + summary)
        print(f'[quest] {summary}  (detail: {self.stall_log_path})')
        self._running = False
        with self._sock_lock:
            if self._sock:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self._sock.close()
                self._sock = None
        if self._thread:
            self._thread.join(timeout=2.0)

    # -- consumer API --------------------------------------------------------

    def get(self) -> QuestState | None:
        with self._state_lock:
            return self._state

    def age(self) -> float:
        """Seconds since the last received pose frame (inf if never)."""
        with self._state_lock:
            if self._state is None:
                return float("inf")
            return time.monotonic() - self._state.recv_mono

    def rate_hz(self) -> float:
        with self._state_lock:
            w = self._rate_window
            if len(w) < 2 or time.monotonic() - w[-1] > 1.0:
                return 0.0  # stream dead/stale -> report 0, not the last value
            dt = w[-1] - w[0]
            return (len(w) - 1) / dt if dt > 0 else 0.0

    @property
    def hello(self) -> dict | None:
        return self._hello

    # -- commands to the headset ----------------------------------------------

    def send_haptic(self, hand: str = "both", amp: float = 0.6, ms: int = 120) -> None:
        self._send({"cmd": "haptic", "hand": hand, "amp": amp, "ms": ms})

    def set_color(self, r: float, g: float, b: float, a: float = 0.35) -> None:
        """Background tint. With passthrough active, `a` is the tint opacity
        over the camera feed (0 = invisible)."""
        self._send({"cmd": "color", "r": r, "g": g, "b": b, "a": a})

    def send_anchor_save(self) -> None:
        """Pin the CURRENT world frame as a persistent spatial anchor on the
        headset. Send once teleop axes feel correct (yaw_offset_deg tuned):
        poses are then reported in this exact frame forever — across headset
        reboots and guardian redraws — so the calibration stops drifting.
        Watch QuestState.space flip to "anchor" for confirmation."""
        self._send({"cmd": "anchor_save"})

    def send_anchor_clear(self) -> None:
        """Erase the persistent anchor; poses revert to the session stage."""
        self._send({"cmd": "anchor_clear"})

    def send_calibrate_x(self, hand: str = "right") -> None:
        """Start the in-headset two-point X-axis calibration: trigger-click
        at point 1, move the controller along the desired robot +X, click
        point 2. The line becomes robot +X and is saved as the persistent
        anchor (survives reboots). Teleop input is masked while calibrating;
        a faint arrow in the headset previews and then pins the axis."""
        self._send({"cmd": "calibrate_x", "hand": hand})

    def send_calibrate_cancel(self) -> None:
        self._send({"cmd": "calibrate_cancel"})

    def _send(self, obj: dict) -> None:
        data = (json.dumps(obj) + "\n").encode()
        with self._sock_lock:
            if self._sock is None:
                return
            try:
                self._sock.sendall(data)
            except OSError:
                pass  # reconnect loop will handle it

    # -- reader thread ---------------------------------------------------------

    def _run(self) -> None:
        while self._running:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(2.0)
                with self._sock_lock:
                    self._sock = sock
                self.connected = True
                self._stall_log('connected')
                self._read_loop(sock)
            except OSError as exc:
                self._stall_log(f'connect/read error: {exc!r}')
            finally:
                if self.connected:
                    self._stall_log('disconnected')
                self.connected = False
                with self._sock_lock:
                    if self._sock is not None:
                        try:
                            self._sock.close()
                        except OSError:
                            pass
                        self._sock = None
            if self._running:
                time.sleep(1.0)  # reconnect backoff

    def _read_loop(self, sock: socket.socket) -> None:
        buf = b""
        last_data = time.monotonic()
        warned = False
        while self._running:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                silent = time.monotonic() - last_data
                if silent > self.stall_reconnect_s:
                    print(f"\n[quest] stream silent {silent:.1f}s — reconnecting "
                          "(headset asleep? adb/USB hiccup?)", flush=True)
                    self._stall_log(f'forced reconnect after {silent:.1f}s silence')
                    return  # _run closes the socket and reconnects
                if silent > 2.0 and not warned:
                    warned = True
                    print(f"\n[quest] stream silent {silent:.1f}s...", flush=True)
                continue
            if not chunk:
                return  # closed by peer
            if warned:
                warned = False
                print(f"\n[quest] stream recovered after "
                      f"{time.monotonic() - last_data:.1f}s", flush=True)
            last_data = time.monotonic()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line:
                    self._handle_line(line)

    def _handle_line(self, line: bytes) -> None:
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            return
        if self._on_line is not None:
            try:
                self._on_line(d, time.monotonic())
            except Exception:
                pass
        if "hello" in d:
            self._hello = d["hello"]
            return
        if "heartbeat" in d:
            with self._state_lock:
                if self._state is not None:
                    self._state.session_state = d.get("state", "UNKNOWN")
            return
        if "head" not in d:
            return
        now = time.monotonic()
        if self._last_pose_mono is not None:
            gap = now - self._last_pose_mono
            if gap > self.stall_log_threshold_s:
                band = '0.25-1s' if gap < 1.0 else '1-6s' if gap < 6.0 else '>6s'
                self._stall_counts[band] += 1
                self._stall_max = max(self._stall_max, gap)
                self._stall_log(f'pose gap {gap:6.2f}s')
        self._last_pose_mono = now
        anchor = d.get("anchor") or {}
        st = QuestState(
            t=float(d.get("t", 0.0)),
            mono=float(d.get("mono", 0.0)),
            frame=int(d.get("frame", 0)),
            session_state=str(d.get("state", "UNKNOWN")),
            space=str(d.get("space", "stage")),
            anchor_have=bool(anchor.get("have", False)),
            anchor_tracked=bool(anchor.get("tracked", False)),
            calib_step=int(d.get("calib", 0)),
            head=HandState.from_json(d.get("head", {})),
            left=HandState.from_json(d.get("left", {})),
            right=HandState.from_json(d.get("right", {})),
            recv_mono=now,
        )
        with self._state_lock:
            self._state = st
            self._n_frames += 1
            self._rate_window.append(now)
            if len(self._rate_window) > 90:
                self._rate_window = self._rate_window[-90:]
