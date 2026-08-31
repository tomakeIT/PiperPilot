"""SpaceMouse rate-control tests with an injected fake reader (no HID)."""

import time

import numpy as np

from piper_teleop.config import load_config
from piper_teleop.piper_arm import FakeArm
from piper_teleop.spacemouse_client import SpaceMouseState
from piper_teleop.spacemouse_controller import SpaceMouseController


class FakeSMReader:
    def __init__(self):
        self.twist = np.zeros(6)
        self.buttons = (0, 0)

    def get(self):
        return SpaceMouseState(twist=self.twist.copy(), buttons=self.buttons,
                               mono=time.monotonic())

    def age(self):
        return 0.0

    def rate_hz(self):
        return 100.0

    def stop(self):
        pass


def make_stack():
    cfg = load_config(overrides={"arm": {"backend": "fake"}, "input": "spacemouse"})
    arm = FakeArm(cfg)
    sm = FakeSMReader()
    ctl = SpaceMouseController(sm, arm, cfg)
    return cfg, arm, sm, ctl


def test_integration_moves_and_stops():
    cfg, arm, sm, ctl = make_stack()
    ctl.start()
    try:
        time.sleep(0.1)  # latch initial pose
        p0 = ctl.status.target_pose6[:3].copy() if ctl.status.engaged else None

        sm.twist = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])  # puck forward
        time.sleep(0.5)
        pose, grip = ctl.latest_action()
        assert pose is not None
        # forward puck -> +x in robot frame
        assert pose[0] > 0.30, pose

        sm.twist = np.zeros(6)
        time.sleep(0.1)
        pose1, _ = ctl.latest_action()
        time.sleep(0.2)
        pose2, _ = ctl.latest_action()
        assert np.allclose(pose1, pose2), "must hold still when puck released"

        # workspace clamp
        sm.twist = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        ws_max_x = cfg.teleop.limits.workspace_max[0]
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            pose, _ = ctl.latest_action()
            assert pose[0] <= ws_max_x + 1e-9
            if pose[0] >= ws_max_x - 1e-6:
                break
            time.sleep(0.05)
        assert pose[0] <= ws_max_x + 1e-9
    finally:
        ctl.stop()


def test_deadzone_ignored():
    _, arm, sm, ctl = make_stack()
    ctl.start()
    try:
        time.sleep(0.1)
        sm.twist = np.full(6, 0.05)  # below deadzone 0.10
        time.sleep(0.3)
        pose, _ = ctl.latest_action()
        assert pose is None or not ctl.status.engaged
    finally:
        ctl.stop()


CHORD = 0.2  # matches configs/default.yaml spacemouse.chord_window_s
SETTLE = CHORD + 0.15


def test_buttons_gripper_and_record():
    cfg, arm, sm, ctl = make_stack()
    ctl.start()
    try:
        time.sleep(0.1)
        # gripper toggle on left button (fires after the chord window)
        sm.buttons = (1, 0)
        time.sleep(SETTLE)
        _, grip = ctl.latest_action()
        assert grip == 0.0  # closed
        sm.buttons = (0, 0)
        time.sleep(0.1)
        sm.buttons = (1, 0)
        time.sleep(SETTLE)
        _, grip = ctl.latest_action()
        assert grip == cfg.gripper.max_width  # open again

        # record toggle: short press fires on RELEASE
        sm.buttons = (0, 0)
        time.sleep(0.1)
        ctl.poll_events()  # clear
        sm.buttons = (0, 1)
        time.sleep(0.2)
        assert not ctl.poll_events().primary  # nothing while held
        sm.buttons = (0, 0)
        time.sleep(0.1)
        ev = ctl.poll_events()
        assert ev.primary and not ev.secondary
        # simultaneous press -> discard
        sm.buttons = (1, 1)
        time.sleep(SETTLE)
        ev = ctl.poll_events()
        assert ev.secondary
        sm.buttons = (0, 0)
        time.sleep(0.1)
        assert not ctl.poll_events().primary  # release after chord: no record
    finally:
        ctl.stop()


def test_long_press_record_button_homes():
    import numpy as np
    from piper_teleop.piper_arm import DEFAULT_HOME_JOINTS

    cfg, arm, sm, ctl = make_stack()
    ctl.start()
    try:
        time.sleep(0.1)
        ctl.poll_events()
        sm.buttons = (0, 1)
        time.sleep(1.3)  # > home_hold_s (1.0)
        sm.buttons = (0, 0)
        time.sleep(0.3)  # homing executes in the control loop
        ev = ctl.poll_events()
        assert not ev.primary, "long press must not also toggle recording"
        st = arm.get_state()
        assert np.allclose(st.joints, DEFAULT_HOME_JOINTS), "arm should be homed"
    finally:
        ctl.stop()


def test_staggered_chord_is_discard_not_save():
    """Humans press chords 30-150 ms apart: record-then-gripper within the
    window must produce ONLY discard (no record toggle, no gripper motion)."""
    cfg, arm, sm, ctl = make_stack()
    ctl.start()
    try:
        time.sleep(0.1)
        ctl.poll_events()
        sm.buttons = (0, 1)      # record button first
        time.sleep(0.05)         # 50 ms later …
        sm.buttons = (1, 1)      # … gripper button joins -> chord
        time.sleep(SETTLE)
        ev = ctl.poll_events()
        assert ev.secondary
        assert not ev.primary
        _, grip = ctl.latest_action()
        assert grip is None or grip == cfg.gripper.max_width  # gripper untouched
    finally:
        ctl.stop()
