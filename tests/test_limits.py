"""Limit-visibility tests: trackers + controller instrumentation."""

import time

import numpy as np

from piper_teleop.config import load_config
from piper_teleop.limits import LimitTracker, merged_active_str
from piper_teleop.piper_arm import FakeArm
from piper_teleop.spacemouse_controller import SpaceMouseController
from test_spacemouse_controller import FakeSMReader


def test_tracker_counts_hold_and_expiry():
    t = LimitTracker(hold_s=0.05)
    t.hit("ws", "x+")
    t.hit("ws", "z-")
    s = t.snapshot()
    assert s["ws"]["count"] == 2
    assert s["ws"]["active"]
    assert s["ws"]["detail"] == "z-"     # latest detail wins
    assert t.active_str() == "ws:z-"
    time.sleep(0.08)
    assert not t.snapshot()["ws"]["active"]  # expired but count retained
    assert t.snapshot()["ws"]["count"] == 2
    assert t.active_str() == ""


def test_merged_active_str_flattens_groups():
    a, b = LimitTracker(), LimitTracker()
    a.hit("ws", "x+")
    b.hit("ik")
    s = merged_active_str({"teleop": a.snapshot(), "arm": b.snapshot()})
    assert "ws:x+" in s and "ik" in s


def test_spacemouse_workspace_clamp_is_reported():
    cfg = load_config(overrides={
        "arm": {"backend": "fake"}, "input": "spacemouse",
        "teleop": {"limits": {"workspace_max": [0.32, 0.35, 0.50]}}})
    arm = FakeArm(cfg)  # EEF starts at x=0.30 -> the 0.32 wall is 2 cm away
    sm = FakeSMReader()
    ctl = SpaceMouseController(sm, arm, cfg)
    ctl.start()
    try:
        time.sleep(0.15)                              # latch initial pose
        sm.twist = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])  # drive +x
        time.sleep(0.6)
        snap = ctl.limits.snapshot()
        assert "ws" in snap and snap["ws"]["active"], snap
        assert "x+" in snap["ws"]["detail"]
        pose, _ = ctl.latest_action()
        assert pose[0] <= 0.32 + 1e-9                 # still actually clamped
    finally:
        ctl.stop()
