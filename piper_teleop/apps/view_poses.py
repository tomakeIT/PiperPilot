"""Step-2 tool: read Quest controller poses and visualize them live in rerun.

Usage:
    scripts/quest_connect.sh          # once: install/start APK + adb forward
    piper-view [--config my.yaml] [--serve]
"""

from __future__ import annotations

import argparse
import time

from ..config import load_config
from ..quest_client import QuestReader
from ..visualize import RerunViz
from .. import xr_math


def main() -> None:
    ap = argparse.ArgumentParser(description="Visualize Quest controller poses")
    ap.add_argument("--config", default=None, help="YAML config overlay")
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--serve", action="store_true", help="rerun web viewer (headless host)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    host = args.host or cfg.quest.host
    port = args.port or cfg.quest.port

    viz = RerunViz(spawn=not args.serve, serve=args.serve)
    quest = QuestReader(host, port)
    quest.start()
    print(f"[view] connecting to {host}:{port} … (run scripts/quest_connect.sh first)")

    map_m = xr_math.xr_to_robot_matrix()
    last_report = 0.0
    try:
        while True:
            qs = quest.get()
            if qs is not None and quest.age() < 0.5:
                viz.log_quest(qs)
                hand = qs.hand(cfg.quest.control_hand)
                if hand.valid:
                    pos_r = map_m @ hand.pos
                    rot_r = map_m @ xr_math.quat_xyzw_to_matrix(hand.quat_xyzw) @ map_m.T
                    viz.log_mapped_hand(pos_r, rot_r)
            now = time.monotonic()
            if now - last_report > 2.0:
                last_report = now
                status = (f"{quest.rate_hz():.1f} Hz, session={qs.session_state}"
                          if qs and quest.age() < 0.5 else "waiting for stream…")
                print(f"[view] {status}")
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        quest.stop()


if __name__ == "__main__":
    main()
