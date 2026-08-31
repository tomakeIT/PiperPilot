import json
import math

import numpy as np
import pandas as pd
import pytest

from piper_teleop.apps.replay import (available_episodes, build_targets,
                                      load_episode, map_gripper)

YAW90_XYZW = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
IDENT_XYZW = [0.0, 0.0, 0.0, 1.0]


def _write_modality_ds(root, episode=38, n=4, joints=False):
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    part = lambda s, e, rot=False: dict(
        {"start": s, "end": e, "dtype": "float64", "absolute": True},
        **({"rotation_type": "quat_xyzw"} if rot else {}))
    groups = {}
    for g, key in (("state", "observation.state"), ("action", "action")):
        groups[g] = {
            "left_position": part(0, 3), "left_orientation": part(3, 7, True),
            "left_gripper": part(7, 8),
            "right_position": part(8, 11), "right_orientation": part(11, 15, True),
            "right_gripper": part(15, 16),
        }
        for v in groups[g].values():
            v["original_key"] = key
    (root / "meta" / "modality.json").write_text(json.dumps(groups))
    (root / "meta" / "info.json").write_text(json.dumps({"chunks_size": 1000}))

    if joints:
        for g in groups.values():
            g["left_arm_joint"] = part(16, 22)
            g["right_arm_joint"] = part(22, 28)
        (root / "meta" / "modality.json").write_text(json.dumps(groups))

    rows = []
    for i in range(n):
        left = [9.0, 9.0, 9.0, *IDENT_XYZW, 0.5]        # sentinel: wrong side
        right = [0.1 * i, 0.0, 0.05, *YAW90_XYZW, 5.0 * i / max(n - 1, 1)]
        row = left + right
        if joints:
            row = row + [8.0] * 6 + [0.01 * i] * 6      # left sentinel, right
        rows.append(row)
    a = np.array(rows, dtype=float)
    df = pd.DataFrame({
        "observation.state": list(a),
        "action": list(a),
        "timestamp": np.arange(n) / 30.0,
    })
    df.to_parquet(root / "data" / "chunk-000" / f"episode_{episode:06d}.parquet")
    return root


def test_modality_loader_slices_side_and_converts_quat(tmp_path):
    root = _write_modality_ds(tmp_path / "ds")
    actions, grip, obs, joints, ts, schema = load_episode(root, 38, "right")
    assert schema == "modality"
    assert joints is None  # EEF-only dataset: --joints must be refused
    assert actions.shape == (4, 7)
    np.testing.assert_allclose(actions[1, :3], [0.1, 0.0, 0.05])
    # xyzw yaw-90 -> wxyz [cos45, 0, 0, sin45]
    np.testing.assert_allclose(
        actions[0, 3:7],
        [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)], atol=1e-9)
    assert grip[-1] == pytest.approx(5.0)
    assert not np.any(actions[:, :3] == 9.0)  # left-arm sentinel not picked up


def test_modality_loader_joint_channel(tmp_path):
    root = _write_modality_ds(tmp_path / "ds", joints=True)
    actions, grip, obs, joints, ts, schema = load_episode(root, 38, "right")
    assert joints.shape == (4, 6)
    np.testing.assert_allclose(joints[2], [0.02] * 6)  # right channel
    assert not np.any(joints == 8.0)                   # not the left sentinel


def test_native_loader_returns_joints(tmp_path):
    root = tmp_path / "native"
    (root / "data" / "chunk-000").mkdir(parents=True)
    n = 3
    df = pd.DataFrame({
        "action.arm_eef": [np.array([0.3, 0, 0.2, 1, 0, 0, 0])] * n,
        "action.gripper": [np.array([0.05])] * n,
        "observation.arm_eef": [np.array([0.3, 0, 0.2, 1, 0, 0, 0])] * n,
        "observation.arm_joint": [np.arange(6, dtype=float) * 0.1] * n,
        "timestamp": np.arange(n) / 30.0,
    })
    df.to_parquet(root / "data" / "chunk-000" / "episode_000000.parquet")
    actions, grip, obs, joints, ts, schema = load_episode(root, 0)
    assert schema == "native"
    assert joints.shape == (n, 6)
    np.testing.assert_allclose(joints[0], np.arange(6) * 0.1)
    assert grip[0] == pytest.approx(0.05)


def test_missing_episode_lists_available(tmp_path):
    root = _write_modality_ds(tmp_path / "ds")
    assert available_episodes(root) == [38]
    with pytest.raises(SystemExit, match=r"\[38\]"):
        load_episode(root, 0, "right")


def test_build_targets_relative_reanchors_pose():
    actions7 = np.array([
        [0.0, 0.0, 0.0, 1, 0, 0, 0],
        [0.1, 0.0, 0.0, 1, 0, 0, 0],
    ])
    start = np.array([0.3, 0.0, 0.2, 0.0, 0.0, math.radians(10)])
    t = build_targets(actions7, relative=True, start_pose6=start)
    np.testing.assert_allclose(t[0], start, atol=1e-9)         # zero delta
    np.testing.assert_allclose(t[1][:3], [0.4, 0.0, 0.2], atol=1e-9)
    assert t[1][5] == pytest.approx(math.radians(10))          # yaw preserved

    # A recorded +30 deg yaw delta lands on top of the start yaw.
    yaw30 = [math.cos(math.radians(15)), 0, 0, math.sin(math.radians(15))]
    actions7[1, 3:7] = yaw30
    t = build_targets(actions7, relative=True, start_pose6=start)
    assert t[1][5] == pytest.approx(math.radians(40), abs=1e-6)


def test_map_gripper():
    width, note = map_gripper(np.array([0.0, 0.03, 0.07]), 0.07)
    np.testing.assert_allclose(width, [0.0, 0.03, 0.07])
    assert "pass-through" in note

    width, note = map_gripper(np.array([0.0, 2.5, 5.0]), 0.07)
    np.testing.assert_allclose(width, [0.0, 0.035, 0.07])
    width, _ = map_gripper(np.array([0.0, 2.5, 5.0]), 0.07, raw_max=10.0)
    assert width[-1] == pytest.approx(0.035)


def test_apply_rot_offset_tool_side():
    from piper_teleop.apps.replay import apply_rot_offset

    acts = np.zeros((2, 7))
    acts[:, 3] = 1.0                       # identity quats (wxyz)
    acts[1, :3] = [0.1, 0.0, 0.0]
    out = apply_rot_offset(acts, [0, 0, 90])
    np.testing.assert_allclose(out[0, :3], acts[0, :3])  # positions untouched
    np.testing.assert_allclose(               # identity @ Rz(90) -> yaw-90 quat
        out[0, 3:7], [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)],
        atol=1e-9)

    # A constant tool-side rotation must cancel out in RELATIVE replay.
    start = np.array([0.3, 0.0, 0.2, 0.0, 0.0, 0.5])
    np.testing.assert_allclose(
        build_targets(out, relative=True, start_pose6=start),
        build_targets(acts, relative=True, start_pose6=start), atol=1e-9)


def test_dataset_rot_offset_sidecar(tmp_path):
    from piper_teleop.apps.replay import dataset_rot_offset

    root = _write_modality_ds(tmp_path / "ds")
    assert dataset_rot_offset(root) is None
    (root / "meta" / "piper_replay.json").write_text(
        json.dumps({"rot_offset_deg": [-90, 0, -90]}))
    assert dataset_rot_offset(root) == [-90.0, 0.0, -90.0]


def test_clean_permutation_is_rpy_m90_0_m90():
    from piper_teleop import xr_math

    C = xr_math.rpy_to_matrix(*np.radians([-90.0, 0.0, -90.0]))
    np.testing.assert_allclose(
        C, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-12)


def test_dataset_conventions_offset(tmp_path):
    from piper_teleop.apps.replay import dataset_conventions

    root = _write_modality_ds(tmp_path / "ds")
    assert dataset_conventions(root) == {}
    (root / "meta" / "piper_replay.json").write_text(
        json.dumps({"rot_offset_deg": [-90, 0, -90],
                    "offset_m": [0.30, 0.0, 0.20]}))
    assert dataset_conventions(root)["offset_m"] == [0.30, 0.0, 0.20]
