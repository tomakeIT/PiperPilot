"""Damped-least-squares IK on top of pyAgxArm's MDH forward kinematics.

Used by the MIT-impedance backend: the firmware does IK only for move_p,
so joint-space (move_mit) control needs the host to convert EEF pose
targets into joint targets. Warm-started from the previous solution and
fed with small per-tick deltas, 2-4 iterations converge comfortably at
100 Hz (numeric Jacobian = 6 extra FK calls per iteration; FK is a
6-link MDH chain, ~0.1 ms in pure Python).
"""

from __future__ import annotations

import math

import numpy as np

from . import xr_math


def _pose_error(target_pos, target_rot, cur_pos, cur_rot) -> np.ndarray:
    """6D error twist: [position error, orientation error as rotation vector]."""
    e_pos = target_pos - cur_pos
    dr = target_rot @ cur_rot.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(dr) - 1.0) / 2.0)))
    if angle < 1e-9:
        e_rot = np.zeros(3)
    else:
        axis = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])
        n = np.linalg.norm(axis)
        e_rot = (axis / n) * angle if n > 1e-12 else np.zeros(3)
    return np.concatenate([e_pos, e_rot])


class MDHIK:
    def __init__(self, fk_fn, lower: np.ndarray, upper: np.ndarray,
                 damping: float = 0.05, iters: int = 4,
                 pos_tol: float = 0.002, rot_tol: float = 0.02):
        """fk_fn: joints(list[6] rad) -> [x,y,z,roll,pitch,yaw] (pyAgxArm robot.fk).
        lower/upper: joint limits (rad)."""
        self.fk_fn = fk_fn
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.damping = float(damping)
        self.iters = int(iters)
        self.pos_tol = float(pos_tol)
        self.rot_tol = float(rot_tol)

    def _fk(self, q: np.ndarray):
        p = self.fk_fn([float(x) for x in q])
        return np.asarray(p[:3]), xr_math.rpy_to_matrix(p[3], p[4], p[5])

    def solve(self, pose6_target, q_init) -> tuple[np.ndarray, bool]:
        """Returns (q, converged). q is always joint-limit clamped; on
        non-convergence q is the best iterate (caller decides what to do)."""
        target_pos = np.asarray(pose6_target[:3], dtype=float)
        target_rot = xr_math.rpy_to_matrix(*pose6_target[3:6])
        q = np.clip(np.asarray(q_init, dtype=float), self.lower, self.upper)
        h = 1e-5
        lam2 = self.damping ** 2

        for _ in range(self.iters):
            cur_pos, cur_rot = self._fk(q)
            err = _pose_error(target_pos, target_rot, cur_pos, cur_rot)
            if (np.linalg.norm(err[:3]) < self.pos_tol
                    and np.linalg.norm(err[3:]) < self.rot_tol):
                return q, True

            jac = np.zeros((6, 6))
            for i in range(6):
                qp = q.copy()
                qp[i] += h
                p_i, r_i = self._fk(qp)
                jac[:3, i] = (p_i - cur_pos) / h
                dr = r_i @ cur_rot.T
                jac[3:, i] = np.array(
                    [dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]
                ) / (2.0 * h)

            jjt = jac @ jac.T + lam2 * np.eye(6)
            dq = jac.T @ np.linalg.solve(jjt, err)
            dq = np.clip(dq, -0.2, 0.2)  # keep iterates tame near singularities
            q = np.clip(q + dq, self.lower, self.upper)

        cur_pos, cur_rot = self._fk(q)
        err = _pose_error(target_pos, target_rot, cur_pos, cur_rot)
        ok = (np.linalg.norm(err[:3]) < self.pos_tol
              and np.linalg.norm(err[3:]) < self.rot_tol)
        return q, ok
