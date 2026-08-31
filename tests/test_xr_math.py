import math

import numpy as np
import pytest

from piper_teleop import xr_math as xm


def random_quat(rng):
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)


def test_xr_to_robot_is_proper_rotation():
    m = xm.XR_TO_ROBOT
    assert np.allclose(m @ m.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(m), 1.0)
    # XR forward (-z) -> robot forward (+x); XR up (+y) -> robot up (+z)
    assert np.allclose(m @ np.array([0, 0, -1.0]), [1, 0, 0])
    assert np.allclose(m @ np.array([0, 1.0, 0]), [0, 0, 1])
    assert np.allclose(m @ np.array([1.0, 0, 0]), [0, -1, 0])


def test_quat_matrix_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = random_quat(rng)
        m = xm.quat_xyzw_to_matrix(q)
        assert np.allclose(m @ m.T, np.eye(3), atol=1e-9)
        q2 = xm.matrix_to_quat_xyzw(m)
        # q and -q are the same rotation
        assert np.allclose(q, q2, atol=1e-6) or np.allclose(q, -q2, atol=1e-6)


def test_rpy_matrix_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        roll = rng.uniform(-math.pi, math.pi)
        pitch = rng.uniform(-math.pi / 2 + 0.01, math.pi / 2 - 0.01)
        yaw = rng.uniform(-math.pi, math.pi)
        m = xm.rpy_to_matrix(roll, pitch, yaw)
        r2, p2, y2 = xm.matrix_to_rpy(m)
        assert np.allclose([roll, pitch, yaw], [r2, p2, y2], atol=1e-9)


def test_rpy_convention_matches_rz_ry_rx():
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll): yaw-only rotates x toward y.
    m = xm.rpy_to_matrix(0.0, 0.0, math.pi / 2)
    assert np.allclose(m @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-12)
    m = xm.rpy_to_matrix(0.0, math.pi / 2 - 1e-9, 0.0)
    assert np.allclose(m @ np.array([1.0, 0, 0]), [0, 0, -1], atol=1e-6)


def test_wxyz_xyzw_conversions():
    q = np.array([0.1, 0.2, 0.3, 0.4])
    assert np.allclose(xm.quat_wxyz_to_xyzw(xm.quat_xyzw_to_wxyz(q)), q)


def test_clamp_angle_step():
    r0 = np.eye(3)
    r1 = xm.rpy_to_matrix(0, 0, 1.0)  # 1 rad about z
    r_lim = xm.clamp_angle_step(r0, r1, 0.1)
    dr = r_lim @ r0.T
    angle = math.acos((np.trace(dr) - 1) / 2)
    assert angle == pytest.approx(0.1, abs=1e-6)
    # Within limit -> unchanged
    r_ok = xm.clamp_angle_step(r0, r1, 2.0)
    assert np.allclose(r_ok, r1)


def test_one_euro_converges():
    f = xm.OneEuroFilter(min_cutoff=1.0, beta=0.0)
    t = 0.0
    out = np.zeros(3)
    for _ in range(500):
        t += 0.01
        out = f(np.array([1.0, 2.0, 3.0]), t)
    assert np.allclose(out, [1, 2, 3], atol=1e-3)


def test_quat_filter_hemisphere():
    f = xm.QuatOneEuroFilter(min_cutoff=5.0)
    q = np.array([0, 0, 0, 1.0])
    out1 = f(q, 0.01)
    out2 = f(-q, 0.02)  # same rotation, flipped sign
    assert np.dot(out1, out2) > 0.99
