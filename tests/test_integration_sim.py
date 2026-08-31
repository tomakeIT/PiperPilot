"""End-to-end dry run: synthetic quest states -> controller -> fake arm ->
recorder -> LeRobot dataset. No hardware, no network."""

import time

import numpy as np

from piper_teleop.cameras import FakeCamera
from piper_teleop.config import load_config
from piper_teleop.lerobot_writer import LeRobotWriter
from piper_teleop.piper_arm import FakeArm
from piper_teleop.quest_client import HandState, QuestReader, QuestState
from piper_teleop.recorder import Recorder
from piper_teleop.teleop_controller import TeleopController


class FakeQuest(QuestReader):
    """QuestReader that synthesizes a moving right controller instead of
    reading from a socket."""

    def __init__(self):
        super().__init__(host="none", port=0)
        self._t0 = time.monotonic()
        self.haptics = []

    def start(self):
        pass

    def stop(self):
        pass

    def send_haptic(self, hand="both", amp=0.6, ms=120):
        self.haptics.append((hand, amp, ms))

    def set_color(self, r, g, b, a=0.35):
        pass

    def get(self):
        t = time.monotonic() - self._t0
        right = HandState(
            pos=np.array([0.1 * np.sin(t), 1.2, -0.3 - 0.05 * t]),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            valid=True,
            tracked=True,
            trigger=0.5,
            squeeze=1.0,  # clutch always engaged
        )
        return QuestState(
            t=t, mono=t, frame=int(t * 90), session_state="FOCUSED",
            head=HandState(valid=True, tracked=True),
            left=HandState(valid=True, tracked=True),
            right=right, recv_mono=time.monotonic(),
        )

    def age(self):
        return 0.0

    def rate_hz(self):
        return 90.0


def test_engage_latches_held_target_not_sagged_pose():
    """Impedance backends sag below their held target under gravity. The
    clutch must latch the TARGET, else every engage ratchets the arm down."""
    cfg = load_config(overrides={"arm": {"backend": "fake"}})

    class SaggyArm(FakeArm):
        def held_target_pose6(self):
            st = self.get_state()
            held = st.eef_pose6.copy()
            held[2] += 0.05  # target is 5 cm ABOVE the sagged measured pose
            return held

    quest = FakeQuest()
    arm = SaggyArm(cfg)
    z_sagged = arm.get_state().eef_pose6[2]  # measured ("sagged") z pre-engage
    ctl = TeleopController(quest, arm, cfg)
    ctl.start()
    try:
        time.sleep(0.4)
        assert ctl.status.engaged
        pose, _ = ctl.latest_action()
        # Commanded z must start from the held target (+5 cm), not from the
        # sagged measured pose (the old behavior latched z_sagged).
        assert pose[2] > z_sagged + 0.03, (
            f"engage latched sagged pose: cmd z={pose[2]:.3f} "
            f"pre-engage measured z={z_sagged:.3f}")
    finally:
        ctl.stop()


def test_sim_pipeline(tmp_path):
    cfg = load_config(overrides={
        "arm": {"backend": "fake"},
        "recording": {"root": str(tmp_path), "task": "sim test", "save_raw_quest": True},
    })

    quest = FakeQuest()
    arm = FakeArm(cfg)
    controller = TeleopController(quest, arm, cfg)
    cam = FakeCamera("cam_front", 64, 48, 30)
    writer = LeRobotWriter(
        root=tmp_path / "ds", fps=30,
        video_specs=[{"name": "cam_front", "width": 64, "height": 48, "fps": 30}],
        video_cfg={"preset": "ultrafast", "crf": 30},
        save_raw_quest=True,
    )
    recorder = Recorder(quest, arm, controller, [cam], writer, cfg)

    controller.start()
    recorder.start()
    time.sleep(0.5)  # let the controller engage and command

    # Controller must be engaged and commanding inside the workspace.
    st = controller.status
    assert st.engaged, st.last_error
    ws_min = np.array(cfg.teleop.limits.workspace_min)
    ws_max = np.array(cfg.teleop.limits.workspace_max)
    assert np.all(st.target_pose6[:3] >= ws_min - 1e-9)
    assert np.all(st.target_pose6[:3] <= ws_max + 1e-9)

    recorder.toggle_recording()
    time.sleep(1.2)
    recorder.toggle_recording()

    recorder.stop()
    controller.stop()
    cam.stop()

    assert len(writer.episodes) == 1
    row = writer.episodes[0]
    # ~30 Hz (recording.fps) for ~1.2 s
    assert 25 <= row["length"] <= 50
    assert row["video_frame_length"] >= 20  # plain int by convention
    assert row["video_frame_lengths"]["observation.images.cam_front"] >= 20
    assert (writer.root / "extras/episode_000000.quest.jsonl").exists()
    # Haptics fired on start + stop
    assert len(quest.haptics) >= 2

    # Gripper action recorded: trigger 0.5 -> half-closed width
    import pandas as pd
    df = pd.read_parquet(next(writer.root.glob("data/**/*.parquet")))
    grip = np.stack(df["action.gripper"].to_numpy())
    assert np.allclose(grip, 0.5 * cfg.gripper.max_width, atol=0.01)
