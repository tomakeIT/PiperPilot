"""Rotation/frame math for XR -> robot mapping (numpy only).

Conventions:
- XR (OpenXR STAGE space): x right, y up, z backward; quaternions [x,y,z,w].
- Robot (Piper base): x forward, y left, z up; pyAgxArm Cartesian pose is
  [x,y,z,roll,pitch,yaw] with R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
- Dataset: quaternions wxyz.
"""

from __future__ import annotations

import math

import numpy as np

# Fixed axis remap XR -> robot: robot_x = -xr_z, robot_y = -xr_x, robot_z = xr_y.
# Rows are the robot axes expressed in XR coordinates; det = +1.
XR_TO_ROBOT = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def rotz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def xr_to_robot_matrix(yaw_offset_rad: float = 0.0) -> np.ndarray:
    """Mapping matrix M such that v_robot = M @ v_xr."""
    return rotz(yaw_offset_rad) @ XR_TO_ROBOT


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def matrix_to_quat_xyzw(m: np.ndarray) -> np.ndarray:
    """Robust rotation matrix -> quaternion [x,y,z,w] (Shepperd's method)."""
    t = np.trace(m)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_xyzw_to_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[3], q[0], q[1], q[2]])


def quat_wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """pyAgxArm convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(m: np.ndarray) -> tuple[float, float, float]:
    """Inverse of rpy_to_matrix; pitch is always in [-pi/2, pi/2]."""
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(m[2, 0]) < 0.9999999:
        roll = math.atan2(m[2, 1], m[2, 2])
        yaw = math.atan2(m[1, 0], m[0, 0])
    else:
        # Gimbal lock: yaw and roll are coupled; put everything into yaw.
        roll = 0.0
        yaw = math.atan2(-m[0, 1], m[1, 1])
    return roll, pitch, yaw


def pose6_to_matrix(pose6) -> tuple[np.ndarray, np.ndarray]:
    """[x,y,z,r,p,y] -> (position, rotation matrix)."""
    p = np.asarray(pose6[:3], dtype=float)
    return p, rpy_to_matrix(pose6[3], pose6[4], pose6[5])


def matrix_to_pose6(p: np.ndarray, r: np.ndarray) -> list[float]:
    roll, pitch, yaw = matrix_to_rpy(r)
    return [float(p[0]), float(p[1]), float(p[2]), roll, pitch, yaw]


def clamp_angle_step(r_from: np.ndarray, r_to: np.ndarray, max_angle: float) -> np.ndarray:
    """Limit rotation from r_from to r_to to at most max_angle (radians)."""
    dr = r_to @ r_from.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(dr) - 1.0) / 2.0)))
    if angle <= max_angle or angle < 1e-9:
        return r_to
    # Rodrigues: scale the rotation angle down.
    axis = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        return r_to
    axis /= norm
    a = max_angle
    k = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )
    dr_lim = np.eye(3) + math.sin(a) * k + (1.0 - math.cos(a)) * (k @ k)
    return dr_lim @ r_from


def rotvec_to_matrix(v: np.ndarray) -> np.ndarray:
    """Rodrigues: rotation vector (axis*angle) -> matrix."""
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    axis = v / angle
    k = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


class OneEuroFilter:
    """One-euro filter for vectors (Casiez et al. 2012)."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x_prev: np.ndarray | None = None
        self._dx_prev: np.ndarray | None = None
        self._t_prev: float | None = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self) -> None:
        self._x_prev = None
        self._dx_prev = None
        self._t_prev = None

    def __call__(self, x: np.ndarray, t: float) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if self._t_prev is None or t <= self._t_prev:
            self._x_prev = x
            self._dx_prev = np.zeros_like(x)
            self._t_prev = t
            return x
        dt = t - self._t_prev
        dx = (x - self._x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(dx_hat))
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        self._t_prev = t
        return x_hat


class QuatOneEuroFilter:
    """One-euro on quaternion components with hemisphere alignment."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        self._f = OneEuroFilter(min_cutoff, beta, d_cutoff)
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        self._f.reset()
        self._prev = None

    def __call__(self, q_xyzw: np.ndarray, t: float) -> np.ndarray:
        q = np.asarray(q_xyzw, dtype=float)
        if self._prev is not None and float(np.dot(q, self._prev)) < 0.0:
            q = -q
        out = self._f(q, t)
        out = out / np.linalg.norm(out)
        self._prev = out
        return out
