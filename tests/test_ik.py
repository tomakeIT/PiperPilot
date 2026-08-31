"""IK round-trip tests against the real pyAgxArm MDH forward kinematics
(offline — driver object created but never connected)."""

import time

import numpy as np
import pytest

from piper_teleop import xr_math
from piper_teleop.config import load_config
from piper_teleop.ik import MDHIK


@pytest.fixture(scope="module")
def fk_and_ik():
    pytest.importorskip("pyAgxArm", reason="optional real-arm SDK is not installed")
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

    cfg = load_config()
    acfg = create_agx_arm_config(robot=ArmModel.PIPER_X, firmeware_version=PiperFW.V189,
                                 interface="socketcan", channel="can0")
    robot = AgxArmFactory.create_arm(acfg)  # no connect(): FK only
    imp = cfg.impedance
    ik = MDHIK(robot.fk,
               lower=np.radians(imp.joint_limits_deg.lower),
               upper=np.radians(imp.joint_limits_deg.upper),
               damping=imp.ik.damping, iters=8, pos_tol=0.002)
    return robot.fk, ik


HOME = np.array([0.0, 0.85, -0.75, 0.0, 0.0, 0.0])


def test_roundtrip_near_home(fk_and_ik):
    fk, ik = fk_and_ik
    rng = np.random.default_rng(7)
    solved = 0
    for _ in range(30):
        q_ref = HOME + rng.uniform(-0.25, 0.25, 6)
        q_ref = np.clip(q_ref, ik.lower, ik.upper)
        pose = fk(list(q_ref))
        # Warm start near-but-not-at the solution (teleop conditions).
        q0 = np.clip(q_ref + rng.uniform(-0.08, 0.08, 6), ik.lower, ik.upper)
        q_sol, ok = ik.solve(pose, q0)
        if not ok:
            continue
        solved += 1
        pose_sol = fk(list(q_sol))
        assert np.linalg.norm(np.array(pose_sol[:3]) - np.array(pose[:3])) < 0.004
    assert solved >= 27, f"only {solved}/30 converged"


def test_tracks_small_deltas_like_teleop(fk_and_ik):
    """Simulate a teleop stream: EEF target drifts a few mm per tick; IK must
    track continuously from warm starts without ever losing convergence."""
    fk, ik = fk_and_ik
    q = HOME.copy()
    pose = np.array(fk(list(q)))
    misses = 0
    for k in range(150):
        pose[0] += 0.0012  # 1.2 mm/tick forward
        pose[2] += 0.0008 * np.sin(k / 12)
        q_new, ok = ik.solve(pose, q)
        if not ok:
            misses += 1
            continue
        assert np.max(np.abs(q_new - q)) < 0.12, "joint jump between ticks"
        q = q_new
    assert misses <= 3


def test_ik_speed(fk_and_ik):
    fk, ik = fk_and_ik
    q = HOME.copy()
    pose = np.array(fk(list(q)))
    t0 = time.perf_counter()
    n = 50
    for k in range(n):
        pose[0] += 0.001
        q, _ = ik.solve(pose, q)
    per_call_ms = (time.perf_counter() - t0) / n * 1000
    # 100 Hz control needs << 10 ms per solve.
    assert per_call_ms < 8.0, f"IK too slow: {per_call_ms:.2f} ms/solve"


def test_unreachable_reports_failure(fk_and_ik):
    fk, ik = fk_and_ik
    pose = [1.5, 0.0, 1.5, 0.0, 0.0, 0.0]  # far outside the ~0.62 m reach
    q_sol, ok = ik.solve(pose, HOME)
    assert not ok
    assert np.all(q_sol >= ik.lower) and np.all(q_sol <= ik.upper)
