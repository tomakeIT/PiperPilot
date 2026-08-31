"""Configuration loading: YAML -> nested attribute access with defaults."""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG_PATH = Path(__file__).parent / "configs" / "default.yaml"


class Cfg:
    """Read-only nested attribute view over a dict (cfg.arm.channel etc.)."""

    def __init__(self, data: dict):
        self._data = data

    def __getattr__(self, key: str) -> Any:
        if key.startswith("_"):
            raise AttributeError(key)
        if key not in self._data:
            raise AttributeError(f"config key missing: {key}")
        val = self._data[key]
        if isinstance(val, dict):
            return Cfg(val)
        return val

    def get(self, key: str, default: Any = None) -> Any:
        val = self._data.get(key, default)
        return Cfg(val) if isinstance(val, dict) else val

    def to_dict(self) -> dict:
        return copy.deepcopy(self._data)

    def __repr__(self) -> str:
        return f"Cfg({self._data!r})"


def _deep_update(base: dict, override: dict) -> dict:
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base


def load_config(path: str | None = None, overrides: dict | None = None) -> Cfg:
    """Load default.yaml, optionally overlay a user YAML and a dict of overrides."""
    with open(DEFAULT_CONFIG_PATH) as f:
        data = yaml.safe_load(f)
    if path:
        with open(os.path.expanduser(path)) as f:
            user = yaml.safe_load(f) or {}
        _deep_update(data, user)
    if overrides:
        _deep_update(data, overrides)
    return Cfg(data)
