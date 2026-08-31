import json

import numpy as np

from piper_teleop.apps.replay import apply_rot_offset, load_episode
from piper_teleop.tools.convert_to_piper_frame import convert
from test_replay_loader import _write_modality_ds

CONV = {"rot_offset_deg": [-90.0, 0.0, -90.0], "offset_m": [0.30, 0.0, 0.20]}


def _converted(tmp_path, **kw):
    root = _write_modality_ds(tmp_path / "ds")
    (root / "meta" / "piper_replay.json").write_text(json.dumps(CONV))
    return root, convert(root, tmp_path / "out", **kw)


def test_convert_matches_runtime_conventions(tmp_path):
    root, out = _converted(tmp_path)
    for side in ("right", "left"):
        raw, _, _, _, _, _ = load_episode(root, 38, side)
        got, _, _, _, _, _ = load_episode(out, 38, side)
        exp = apply_rot_offset(raw, CONV["rot_offset_deg"])
        exp[:, :3] += CONV["offset_m"]
        np.testing.assert_allclose(got, exp, atol=1e-12)


def test_convert_gripper_rescaled_to_width(tmp_path):
    # fixture right gripper is raw 0..5; full open must map to max width
    _, out = _converted(tmp_path)
    _, grip, _, _, _, _ = load_episode(out, 38, "right")
    assert np.isclose(grip.max(), 0.07)
    prov = json.load(open(out / "meta" / "frame_conversion.json"))
    assert prov["gripper"]["raw_full_open"]["action.right_gripper"] == 5.0


def test_convert_keep_gripper_raw(tmp_path):
    _, out = _converted(tmp_path, keep_gripper_raw=True)
    _, grip, _, _, _, _ = load_episode(out, 38, "right")
    assert np.isclose(grip.max(), 5.0)


def test_convert_strips_stats_and_sidecar(tmp_path):
    root, out = _converted(tmp_path)
    assert not (out / "meta" / "piper_replay.json").exists()  # no double-apply
    for f in ("stats.json", "relative_stats.json", "delta_stats.json"):
        assert not (out / "meta" / f).exists()
    assert (out / "meta" / "modality.json").exists()
    assert (out / "meta" / "info.json").exists()
    assert (out / "meta" / "frame_conversion.json").exists()


def test_convert_joints_untouched(tmp_path):
    root = _write_modality_ds(tmp_path / "ds", joints=True)
    (root / "meta" / "piper_replay.json").write_text(json.dumps(CONV))
    out = convert(root, tmp_path / "out")
    _, _, _, jr, _, _ = load_episode(root, 38, "right")
    _, _, _, jc, _, _ = load_episode(out, 38, "right")
    np.testing.assert_allclose(jc, jr)
