"""Shared CLI arguments and teleop-stack builder used by the apps
(collect_data, infer, track_test).

There is no separate teleop entry point: piper-collect IS the teleop app —
recording starts only when you press A / space, and nothing is written to
disk until an episode is saved.
"""

from __future__ import annotations

import argparse

from ..config import load_config
from ..piper_arm import make_arm


def add_common_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--config", default=None, help="YAML config overlay")
    ap.add_argument("--input", choices=["quest", "spacemouse"], default=None,
                    help="teleop input device (default from config: quest)")
    ap.add_argument("--sim", action="store_true", help="fake arm backend (no hardware)")
    ap.add_argument("--impedance", action="store_true",
                    help="MIT impedance control (compliant; the default)")
    ap.add_argument("--position", action="store_true",
                    help="firmware position control (move_p; tightest tracking)")
    ap.add_argument("--viz", action="store_true", help="enable rerun visualization")
    ap.add_argument("--serve", action="store_true", help="rerun web viewer (headless)")
    ap.add_argument("--hand", choices=["left", "right"], default=None,
                    help="Quest control hand")


def build_stack(args):
    """Returns (cfg, quest_or_None, arm, controller, viz)."""
    overrides: dict = {}
    if args.sim:
        overrides.setdefault("arm", {})["backend"] = "fake"
    elif getattr(args, "position", False):
        overrides.setdefault("arm", {})["backend"] = "piper"
    elif getattr(args, "impedance", False):
        overrides.setdefault("arm", {})["backend"] = "piper_mit"
    if args.hand:
        overrides.setdefault("quest", {})["control_hand"] = args.hand
    if args.input:
        overrides["input"] = args.input
    cfg = load_config(args.config, overrides)

    arm = make_arm(cfg)
    home = cfg.arm.get("home_joints")
    if home:
        print(f"[arm] moving to home joints {home}")
        arm.move_joints(home)
    quest = None
    if cfg.input == "spacemouse":
        from ..spacemouse_client import SpaceMouseReader
        from ..spacemouse_controller import SpaceMouseController
        sm = SpaceMouseReader(cfg.spacemouse.device)
        controller = SpaceMouseController(sm, arm, cfg)
    else:
        from ..quest_client import QuestReader
        from ..teleop_controller import TeleopController
        quest = QuestReader(cfg.quest.host, cfg.quest.port)
        quest.start()
        controller = TeleopController(quest, arm, cfg)

    viz = None
    if args.viz or args.serve or cfg.viz.enabled:
        from ..visualize import RerunViz
        viz = RerunViz(spawn=not args.serve, serve=args.serve)
    return cfg, quest, arm, controller, viz
