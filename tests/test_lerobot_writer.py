import json

import numpy as np
import pandas as pd
import pytest

from piper_teleop.lerobot_writer import FEATURE_COLUMNS, LeRobotWriter


@pytest.fixture
def writer(tmp_path):
    return LeRobotWriter(
        root=tmp_path / "ds",
        fps=50,
        video_specs=[{"name": "cam_front", "width": 64, "height": 48, "fps": 30}],
        video_cfg={"codec": "libx264", "crf": 30, "preset": "ultrafast"},
    )


def fill_episode(writer, task, n_frames=25, n_video=10):
    ep = writer.start_episode(task)
    rng = np.random.default_rng(ep.episode_index)
    for i in range(n_frames):
        t = i / 50.0
        ep.add_frame(t, {
            "observation.arm_joint": rng.normal(size=6),
            "observation.arm_eef": np.concatenate([rng.normal(size=3), [1, 0, 0, 0]]),
            "observation.gripper": np.array([0.03]),
            "action.arm_eef": np.concatenate([rng.normal(size=3), [1, 0, 0, 0]]),
            "action.gripper": np.array([0.02]),
        })
    img = np.zeros((48, 64, 3), dtype=np.uint8)
    for k in range(n_video):
        img[:] = k * 20
        ep.add_video_frame("cam_front", img, k / 30.0)
    return ep


def test_episode_layout(writer):
    fill_episode(writer, "task A")
    row = writer.finish_episode()
    assert row["length"] == 25

    root = writer.root
    assert (root / "data/chunk-000/episode_000000.parquet").exists()
    assert (root / "videos/chunk-000/observation.images.cam_front/episode_000000.mp4").exists()
    for meta in ("info.json", "modality.json", "episodes.jsonl", "tasks.jsonl", "stats.json"):
        assert (root / "meta" / meta).exists(), meta

    df = pd.read_parquet(root / "data/chunk-000/episode_000000.parquet")
    assert len(df) == 25
    for col, dim in FEATURE_COLUMNS.items():
        assert col in df.columns
        assert np.asarray(df[col][0]).shape == (dim,)
        assert np.asarray(df[col][0]).dtype == np.float32
    assert df["frame_index"].tolist() == list(range(25))
    assert (df["episode_index"] == 0).all()
    assert df["index"].tolist() == list(range(25))

    info = json.loads((root / "meta/info.json").read_text())
    assert info["total_frames"] == 25
    assert info["fps"] == 50
    assert "observation.images.cam_front" in info["features"]
    assert info["features"]["observation.arm_eef"]["shape"] == [7]

    modality = json.loads((root / "meta/modality.json").read_text())
    assert modality["state"]["eef_rot"]["rotation_type"] == "quat_wxyz"
    assert modality["video"]["cam_front"]["original_key"] == "observation.images.cam_front"


def test_multi_episode_and_task_index(writer):
    fill_episode(writer, "task A")
    writer.finish_episode()
    fill_episode(writer, "task B")
    writer.finish_episode()
    fill_episode(writer, "task A")
    writer.finish_episode()

    eps = [json.loads(x) for x in
           (writer.root / "meta/episodes.jsonl").read_text().splitlines()]
    assert [e["episode_index"] for e in eps] == [0, 1, 2]
    assert eps[0]["task_index"] == eps[2]["task_index"] == 0
    assert eps[1]["task_index"] == 1
    assert eps[1]["tasks"] == ["task B"]

    # global index continues across episodes
    df1 = pd.read_parquet(writer.root / "data/chunk-000/episode_000001.parquet")
    assert df1["index"].tolist() == list(range(25, 50))

    tasks = [json.loads(x) for x in
             (writer.root / "meta/tasks.jsonl").read_text().splitlines()]
    assert {t["task"] for t in tasks} == {"task A", "task B"}


def test_discard(writer):
    fill_episode(writer, "task A")
    writer.discard_episode()
    assert writer.episodes == []
    assert not list(writer.root.glob("data/**/*.parquet"))
    assert not list(writer.root.glob("videos/**/*.mp4"))
    # Next episode reuses index 0
    fill_episode(writer, "task A")
    row = writer.finish_episode()
    assert row["episode_index"] == 0


def test_delete_last_episode(writer):
    fill_episode(writer, "task A")
    writer.finish_episode()
    fill_episode(writer, "task A")
    writer.finish_episode()
    assert writer.next_episode_index == 2

    row = writer.delete_last_episode()
    assert row["episode_index"] == 1
    assert writer.next_episode_index == 1
    assert writer.total_frames == 25
    assert not (writer.root / "data/chunk-000/episode_000001.parquet").exists()
    assert not (writer.root / "videos/chunk-000/observation.images.cam_front"
                / "episode_000001.mp4").exists()
    # Meta files rewritten consistently.
    lines = [json.loads(x) for x in
             (writer.root / "meta/episodes.jsonl").read_text().splitlines() if x]
    assert [e["episode_index"] for e in lines] == [0]
    info = json.loads((writer.root / "meta/info.json").read_text())
    assert info["total_episodes"] == 1

    # The freed index is reused, and the global frame index stays contiguous.
    fill_episode(writer, "task A")
    row = writer.finish_episode()
    assert row["episode_index"] == 1
    df = pd.read_parquet(writer.root / "data/chunk-000/episode_000001.parquet")
    assert df["index"].tolist() == list(range(25, 50))

    # No in-flight episode requirement: with one active, delete is refused.
    fill_episode(writer, "task A")
    assert writer.delete_last_episode() is None
    writer.discard_episode()


def test_delete_last_episode_protects_previous_sessions(tmp_path):
    kwargs = dict(
        fps=50,
        video_specs=[{"name": "cam_front", "width": 64, "height": 48, "fps": 30}],
        video_cfg={"preset": "ultrafast", "crf": 30},
    )
    w1 = LeRobotWriter(root=tmp_path / "ds", **kwargs)
    fill_episode(w1, "task A")
    w1.finish_episode()

    # A resuming writer must not delete the previous session's episode ...
    w2 = LeRobotWriter(root=tmp_path / "ds", **kwargs)
    assert w2.delete_last_episode() is None
    # ... but may delete one it saved itself.
    fill_episode(w2, "task A")
    w2.finish_episode()
    row = w2.delete_last_episode()
    assert row["episode_index"] == 1
    assert w2.delete_last_episode() is None  # back at the protection boundary
    assert (tmp_path / "ds/data/chunk-000/episode_000000.parquet").exists()


def test_resume(tmp_path):
    kwargs = dict(
        fps=50,
        video_specs=[{"name": "cam_front", "width": 64, "height": 48, "fps": 30}],
        video_cfg={"preset": "ultrafast", "crf": 30},
    )
    w1 = LeRobotWriter(root=tmp_path / "ds", **kwargs)
    fill_episode(w1, "task A")
    w1.finish_episode()

    w2 = LeRobotWriter(root=tmp_path / "ds", **kwargs)
    assert w2.next_episode_index == 1
    fill_episode(w2, "task A")
    row = w2.finish_episode()
    assert row["episode_index"] == 1
    df = pd.read_parquet(tmp_path / "ds/data/chunk-000/episode_000001.parquet")
    assert df["index"].tolist() == list(range(25, 50))


def test_stats_shapes(writer):
    fill_episode(writer, "task A")
    writer.finish_episode()
    stats = json.loads((writer.root / "meta/stats.json").read_text())
    assert len(stats["observation.arm_eef"]["mean"]) == 7
    assert len(stats["action.gripper"]["q99"]) == 1
    for key in ("mean", "std", "min", "max", "q01", "q99"):
        assert key in stats["observation.arm_joint"]


def test_record_session_and_binding_warning(writer, capsys):
    meta1 = {
        "time": "2026-07-23T15:00:00", "start_episode_index": 0, "task": "t",
        "control": {"backend": "piper_mit",
                    "impedance": {"kp": [5, 25, 5, 8, 8, 5], "kd": [0.3] * 6,
                                  "t_ff": [0] * 6, "gravity_ff": True}},
        "arm": {"firmware": {"software_version": "S-V1.8-9"}},
        "recording": {"fps": 30},
    }
    writer.record_session(meta1)
    # Lazy root: a session that never records leaves NOTHING on disk; the
    # provenance record is flushed when the first episode starts.
    assert not writer.root.exists()
    fill_episode(writer, "t")
    writer.finish_episode()
    assert (writer.root / "meta/collection_meta.json").exists()
    lines = (writer.root / "meta/collection_sessions.jsonl").read_text().splitlines()
    assert len(lines) == 1
    capsys.readouterr()

    # Same params (dataset now exists -> written immediately) -> no warning.
    writer.record_session(dict(meta1, time="2026-07-23T16:00:00"))
    assert "WARNING" not in capsys.readouterr().out

    # Changed gains -> loud warning, session still appended.
    meta2 = json.loads(json.dumps(meta1))
    meta2["control"]["impedance"]["kp"] = [15, 40, 25, 10, 10, 6]
    writer.record_session(meta2)
    out = capsys.readouterr().out
    assert "WARNING" in out and "kp" in out
    lines = (writer.root / "meta/collection_sessions.jsonl").read_text().splitlines()
    assert len(lines) == 3
    latest = json.loads((writer.root / "meta/collection_meta.json").read_text())
    assert latest["control"]["impedance"]["kp"] == [15, 40, 25, 10, 10, 6]


def test_video_pts_alignment(writer):
    import av

    fill_episode(writer, "task A", n_video=10)
    writer.finish_episode()
    path = writer.root / "videos/chunk-000/observation.images.cam_front/episode_000000.mp4"
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        times = [float(f.pts * f.time_base) for f in container.decode(stream)]
    assert len(times) == 10
    # PTS should be k/30 within container timebase rounding
    expected = [k / 30.0 for k in range(10)]
    assert np.allclose(times, expected, atol=0.005)
