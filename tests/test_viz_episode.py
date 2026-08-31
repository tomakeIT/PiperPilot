import numpy as np

from piper_teleop.apps.viz_episode import generate, suggest_offset
from piper_teleop.config import load_config
from test_replay_loader import _write_modality_ds

WS_MIN = np.array([0.05, -0.35, 0.03])
WS_MAX = np.array([0.55, 0.35, 0.50])


def test_suggest_offset_moves_inside():
    p = np.array([[-0.031, 0.003, -0.038], [0.135, 0.118, 0.180]])
    off, fits = suggest_offset(p, WS_MIN, WS_MAX)
    assert fits
    assert ((p + off >= WS_MIN) & (p + off <= WS_MAX)).all()


def test_suggest_offset_zero_when_already_inside():
    p = np.array([[0.2, 0.0, 0.2], [0.3, 0.1, 0.3]])
    off, fits = suggest_offset(p, WS_MIN, WS_MAX)
    assert fits
    np.testing.assert_allclose(off, 0.0)


def test_suggest_offset_centers_when_span_exceeds_box():
    p = np.array([[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    off, fits = suggest_offset(p, WS_MIN, WS_MAX)
    assert not fits
    assert off[0] == (WS_MIN[0] + WS_MAX[0]) / 2  # centered on the box


def test_generate_html_modality(tmp_path):
    root = _write_modality_ds(tmp_path / "ds")
    out, report = generate(root, 38, "right", load_config(None), tmp_path / "v.html")
    html = out.read_text()
    assert "__DATA_JSON__" not in html and "__TITLE__" not in html
    assert '"episode":38' in html and '"schema":"modality"' in html
    assert "base plane z=0" in html
    assert "[right]" in report and "workspace" in report


def test_generate_html_both_arms(tmp_path):
    root = _write_modality_ds(tmp_path / "ds")
    out, report = generate(root, 38, "both", load_config(None), tmp_path / "v.html")
    assert '"right":' in out.read_text() and '"left":' in out.read_text()
    assert "[right]" in report and "[left]" in report


def test_replay_log_roundtrip(tmp_path):
    from piper_teleop.apps.replay import save_replay_log
    from piper_teleop.apps.viz_episode import generate_replay_log

    n = 6
    ts = np.arange(n) / 30.0
    targets = np.tile([0.30, 0.0, 0.20, 0.0, 0.3, 0.0], (n, 1))
    targets[:, 0] += np.linspace(0, 0.05, n)
    meas = targets.copy()
    meas[:, 0] -= 0.004                      # constant 4 mm tracking lag
    ev = np.ones(n, dtype=bool)
    ev[2] = False                            # one dropped feedback tick
    log = tmp_path / "ep000001_test.npz"
    save_replay_log(log, ts, targets, np.full(n, 0.03), meas, ev,
                    np.zeros((n, 6)), np.zeros(n, dtype=bool),
                    np.zeros((n, 7)), np.zeros((n, 7)),
                    {"mode": "eef", "backend": "piper_mit", "episode": 1,
                     "root": "ds", "relative": False, "n_clamped": 0,
                     "arm_guards": {"jlim": {"count": 3, "active": False,
                                             "detail": "j5-"}}})
    out, report = generate_replay_log(log, load_config(None))
    html = out.read_text()
    assert '"cmd"' in html and '"meas"' in html
    assert "cmd vs meas" in report
    assert "jlim x3 (j5-)" in report
    assert "invalid EEF feedback rows (held last pose): 1" in report
