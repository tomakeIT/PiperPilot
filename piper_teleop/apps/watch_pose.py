"""Watch measured Piper joint angles and EEF pose in operator-friendly units."""

from __future__ import annotations

import argparse
import time

import numpy as np

from ..config import load_config
from ..piper_arm import make_arm


def _fmt(values: np.ndarray, ndigits: int) -> str:
    return "[" + ", ".join(f"{float(v):.{ndigits}f}" for v in values) + "]"


def _send_drag_teach_ctrl(arm, ctrl: int) -> bool:
    robot = getattr(arm, "robot", None)
    send = getattr(robot, "_send_msg", None)
    if send is None:
        print("[watch] drag teach control is not available for this backend")
        return False

    from pyAgxArm.protocols.can_protocol.msgs.piper.default import ArmMsgMotionCtrl

    send(ArmMsgMotionCtrl(grag_teach_ctrl=ctrl))
    return True


def _status_fields(arm, fallback_status: int) -> str:
    robot = getattr(arm, "robot", None)
    get_status = getattr(robot, "get_arm_status", None)
    if get_status is None:
        return f"arm_status={fallback_status}"
    msg = get_status()
    if msg is None:
        return f"arm_status={fallback_status}"
    st = msg.msg
    return (
        f"arm_status={getattr(st, 'arm_status', fallback_status)} "
        f"ctrl={getattr(st, 'ctrl_mode', '?')} "
        f"teach={getattr(st, 'teach_status', '?')} "
        f"mode={getattr(st, 'mode_feedback', '?')}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Print live Piper joints and EEF pose while the arm is moved"
    )
    ap.add_argument("--config", default=None)
    ap.add_argument("--sim", action="store_true", help="fake arm backend")
    ap.add_argument("--rate", type=float, default=10.0, help="print rate in Hz")
    ap.add_argument("--once", action="store_true", help="print one sample and exit")
    ap.add_argument(
        "--drag-mode",
        action="store_true",
        help="enter SDK drag teaching mode before watching; exit on shutdown",
    )
    args = ap.parse_args()

    backend = "fake" if args.sim else "piper"
    cfg = load_config(args.config, {"arm": {"backend": backend}})
    arm = make_arm(cfg)
    period = 1.0 / max(float(args.rate), 0.1)

    try:
        drag_active = False
        if args.drag_mode:
            drag_active = _send_drag_teach_ctrl(arm, 0x01)
            if drag_active:
                print("[watch] requested drag teaching mode")
        print(
            "[watch] columns: joints_deg | eef_xyz_m | eef_rpy_deg | "
            "eef_quat_wxyz | status"
        )
        while True:
            st = arm.get_state()
            joints = np.degrees(st.joints)
            xyz = st.eef_pose6[:3]
            rpy = np.degrees(st.eef_pose6[3:6])
            quat = st.eef_quat_wxyz
            valid = (
                f"joints={'ok' if st.joints_valid else 'missing'} "
                f"eef={'ok' if st.eef_valid else 'missing'} "
                f"{_status_fields(arm, st.arm_status)}"
            )
            print(
                "\r"
                f"joints_deg={_fmt(joints, 1)}  "
                f"eef_xyz_m={_fmt(xyz, 3)}  "
                f"eef_rpy_deg={_fmt(rpy, 1)}  "
                f"eef_quat_wxyz={_fmt(quat, 4)}  "
                f"{valid:<48}",
                end="",
                flush=True,
            )
            if args.once:
                print()
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print()
    finally:
        if args.drag_mode and "drag_active" in locals() and drag_active:
            _send_drag_teach_ctrl(arm, 0x02)
            print("[watch] ended drag teaching mode")
        arm.stop()


if __name__ == "__main__":
    main()
