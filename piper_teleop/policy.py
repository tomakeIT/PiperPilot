"""Policy interface for inference on the Piper.

Observation dict (built by apps/infer.py, keys match the dataset schema):
    {
      "task": str,
      "state": {
        "observation.arm_joint": float[6],   # rad
        "observation.arm_eef":   float[7],   # pos(m) + quat wxyz, base frame
        "observation.gripper":   float[1],   # width m
      },
      "images": {"cam_front": HxWx3 uint8 RGB, ...},
    }

Action chunk: float array (H, 8) = [pos3, quat_wxyz4, gripper_width1] per
step, ABSOLUTE, at the dataset rate (30 Hz).

Remote protocol (framework-agnostic; implement server-side next to the
model): POST <url>/predict with JSON
    {"task": str,
     "state": {<key>: [floats]},
     "images": {<cam>: "<base64 jpeg>"},
     "horizon": int}
->  {"actions": [[x,y,z,qw,qx,qy,qz,grip], ...]}     # length <= horizon
"""

from __future__ import annotations

import base64
import json
import urllib.request
from pathlib import Path

import numpy as np


class PolicyBase:
    def reset(self) -> None:
        pass

    def predict(self, obs: dict, horizon: int) -> np.ndarray:
        """Returns (H, 8) absolute actions at the dataset rate."""
        raise NotImplementedError


class HoldPolicy(PolicyBase):
    """Holds the current pose — smoke-tests the full loop without a model."""

    def predict(self, obs: dict, horizon: int) -> np.ndarray:
        eef = np.asarray(obs["state"]["observation.arm_eef"], dtype=float)
        grip = np.asarray(obs["state"]["observation.gripper"], dtype=float)
        step = np.concatenate([eef, grip])
        return np.tile(step, (horizon, 1))


class ReplayPolicy(PolicyBase):
    """Feeds a recorded episode's actions chunk by chunk. Validates the whole
    inference loop (obs building, chunking, execution) against ground truth
    before any real model exists."""

    def __init__(self, root: str, episode: int = 0):
        import pandas as pd

        root_p = Path(root).expanduser()
        pq = (root_p / "data" / f"chunk-{episode // 1000:03d}"
              / f"episode_{episode:06d}.parquet")
        df = pd.read_parquet(pq)
        self.actions = np.hstack([
            np.vstack(df["action.arm_eef"]),
            np.vstack(df["action.gripper"]),
        ])
        self._i = 0

    def reset(self) -> None:
        self._i = 0

    @property
    def done(self) -> bool:
        return self._i >= len(self.actions)

    def predict(self, obs: dict, horizon: int) -> np.ndarray:
        chunk = self.actions[self._i : self._i + horizon]
        self._i += len(chunk)
        if len(chunk) == 0:  # past the end: hold last action
            chunk = self.actions[-1:][:]
        return chunk


class RemotePolicy(PolicyBase):
    """HTTP client for a policy server (see module docstring for protocol)."""

    def __init__(self, url: str, timeout: float = 2.0, jpeg_quality: int = 85):
        self.url = url.rstrip("/") + "/predict"
        self.timeout = timeout
        self.jpeg_quality = jpeg_quality

    def predict(self, obs: dict, horizon: int) -> np.ndarray:
        import cv2

        images = {}
        for name, rgb in obs.get("images", {}).items():
            ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                images[name] = base64.b64encode(jpeg.tobytes()).decode()
        payload = json.dumps({
            "task": obs.get("task", ""),
            "state": {k: list(map(float, v)) for k, v in obs["state"].items()},
            "images": images,
            "horizon": horizon,
        }).encode()
        req = urllib.request.Request(
            self.url, data=payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            out = json.loads(resp.read())
        actions = np.asarray(out["actions"], dtype=float)
        assert actions.ndim == 2 and actions.shape[1] == 8, actions.shape
        return actions


def make_policy(spec: str) -> PolicyBase:
    """spec: 'hold' | 'replay:<root>[:<episode>]' | 'http://host:port'."""
    if spec == "hold":
        return HoldPolicy()
    if spec.startswith("replay:"):
        parts = spec.split(":")
        episode = int(parts[2]) if len(parts) > 2 else 0
        return ReplayPolicy(parts[1], episode)
    if spec.startswith("http"):
        return RemotePolicy(spec)
    raise ValueError(f"unknown policy spec: {spec}")
