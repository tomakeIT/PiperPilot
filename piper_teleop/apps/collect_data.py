"""Step-4 tool: full data collection — teleop + cameras + LeRobot recording.

Usage:
    piper-collect --task "pick up the red block" --root ~/piper_datasets/pick_red
    piper-collect --sim --task test            # end-to-end dry run, no hardware

Controls — Quest (control hand):
    GRIP hold      engage clutch (drive the arm)
    TRIGGER        gripper (pressed = closed)
    A / X          start / stop+save episode   (short buzz / long buzz)
    B / Y          discard current episode     (triple buzz)
    non-control hand X/A held 1 s   discard current episode, or delete the
                                    LAST saved episode of this session
Controls — SpaceMouse (--input spacemouse):
    puck           EEF velocity (rate control)
    left button    gripper toggle
    right button   short press = start/stop+save episode; hold 1 s = home;
                   both buttons = discard
Homing: hold the NON-control hand's Y/B for 1 s (Quest, clutch released),
or press [h]. Keyboard: [space]=start/stop  [d]=discard  [x]=delete last saved
[h]=home  [q]=quit
"""

from __future__ import annotations

import argparse
import faulthandler
import os
import select
import sys
import termios
import threading
import time
import tty
from datetime import datetime

from ..cameras import make_cameras
from ..lerobot_writer import LeRobotWriter
from ..recorder import Recorder
from .teleop_arm import add_common_args, build_stack


class RawKeyboard:
    """Non-blocking single-key reader (POSIX tty)."""

    def __enter__(self):
        self._fd = sys.stdin.fileno()
        try:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            # cbreak leaves XON/XOFF flow control ON: a reflexive Ctrl-S
            # ("save"!) silently freezes terminal output, the status-line
            # print() blocks once the tty buffer fills, and the whole UI
            # loop hangs until Ctrl-Q. Make Ctrl-S/Ctrl-Q ordinary keys.
            attrs = termios.tcgetattr(self._fd)
            attrs[0] &= ~(termios.IXON | termios.IXOFF)
            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
            self._tty = True
        except termios.error:
            self._tty = False  # not a terminal (e.g. piped)
        return self

    def __exit__(self, *exc):
        if self._tty:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def poll(self) -> str | None:
        if not self._tty:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


class LoopWatchdog:
    """Attributes UI-loop freezes: if beat() goes stale for >threshold_s, dump
    every thread's stack — to a log file FIRST (immune to a blocked tty), then
    to stderr. One dump per stall; re-arms after the loop recovers."""

    def __init__(self, threshold_s: float = 2.0):
        self.log_path = f"/tmp/piper_collect_stall_{os.getpid()}.log"
        self._beat = time.monotonic()
        self._threshold = threshold_s
        self._running = True
        self._t = threading.Thread(target=self._watch, name="ui-watchdog", daemon=True)
        self._t.start()

    def beat(self) -> None:
        self._beat = time.monotonic()

    def stop(self) -> None:
        self._running = False

    def _watch(self) -> None:
        while self._running:
            time.sleep(0.5)
            stall = time.monotonic() - self._beat
            if stall <= self._threshold:
                continue
            with open(self.log_path, "a") as f:
                f.write(f"\n=== UI loop stalled {stall:.1f}s at "
                        f"{datetime.now().isoformat(timespec='seconds')} ===\n")
                faulthandler.dump_traceback(file=f, all_threads=True)
            print(f"\n[collect] WARNING: UI loop stalled {stall:.1f}s — thread "
                  f"stacks in {self.log_path}", file=sys.stderr, flush=True)
            while self._running and time.monotonic() - self._beat > self._threshold:
                time.sleep(0.5)  # wait for recovery before re-arming


def main() -> None:
    ap = argparse.ArgumentParser(description="Quest teleop + LeRobot data collection")
    add_common_args(ap)
    ap.add_argument("--task", default=None, help="language instruction for the episodes")
    ap.add_argument("--root", default=None, help="dataset root directory")
    ap.add_argument("--no-cameras", action="store_true")
    args = ap.parse_args()

    cfg, quest, arm, controller, viz = build_stack(args)
    task = args.task or cfg.recording.task
    # Default root gets a timestamp so repeated runs never collide; pass an
    # explicit --root to append to an existing dataset instead.
    root = args.root or (f"{cfg.recording.root}/{task.replace(' ', '_')[:48]}"
                         f"_{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    cameras = [] if args.no_cameras else make_cameras(cfg, fake=args.sim)
    video_specs = [{"name": c.name, "width": c.width, "height": c.height, "fps": c.fps}
                   for c in cameras]

    streamer = None
    vcfg = cfg.quest.get("video")
    if quest is not None and vcfg and vcfg.enabled:
        from ..quest_video import QuestVideoStreamer
        streamer = QuestVideoStreamer(
            cameras, host=cfg.quest.host, port=int(vcfg.port),
            fps=float(vcfg.fps), quality=int(vcfg.quality),
            max_width=int(vcfg.max_width))  # status_fn wired after recorder exists

    writer = LeRobotWriter(
        root=root,
        fps=int(cfg.recording.fps),
        video_specs=video_specs,
        chunks_size=int(cfg.recording.chunks_size),
        robot_type=cfg.recording.robot_type,
        video_cfg=cfg.recording.get("video").to_dict() if cfg.recording.get("video") else {},
        save_raw_quest=bool(cfg.recording.save_raw_quest),
    )
    # Patch the task into a fresh config-derived recorder.
    cfg._data["recording"]["task"] = task  # noqa: SLF001 (CLI-level override)

    # Provenance: bind this dataset to the exact controller/arm/parameters.
    from .. import __version__ as tool_version

    if quest is not None:
        deadline = time.monotonic() + 2.0
        while quest.hello is None and time.monotonic() < deadline:
            time.sleep(0.1)  # hello arrives right after the socket connects

    session_meta = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "start_episode_index": writer.next_episode_index,
        "task": task,
        "piper_teleop_version": tool_version,
        "input_device": cfg.input,
        "quest_app": quest.hello if quest is not None else None,
        "arm": arm.describe(),
        "control": {
            "backend": cfg.arm.backend,
            "teleop": cfg.teleop.to_dict(),
            "impedance": (cfg.impedance.to_dict()
                          if cfg.arm.backend == "piper_mit" else None),
            "spacemouse": (cfg.spacemouse.to_dict()
                           if cfg.input == "spacemouse" else None),
            "gripper": cfg.gripper.to_dict(),
        },
        "cameras": [{"name": c.name, "serial": getattr(c, "serial", None),
                     "width": c.width, "height": c.height, "fps": c.fps}
                    for c in cameras],
        "recording": cfg.recording.to_dict(),
        "conventions": {
            "units": "m / rad / s", "quaternions": "wxyz",
            "frame": "arm base", "actions": "absolute EEF pose + gripper width",
        },
    }
    writer.record_session(session_meta)

    recorder = Recorder(quest, arm, controller, cameras, writer, cfg)
    if streamer is not None:
        streamer.status_fn = recorder.status_image
        streamer.start()

    monitor = None
    mon_cfg = cfg.get("monitor")
    if mon_cfg is not None and mon_cfg.enabled:
        from ..web_monitor import WebMonitor
        monitor = WebMonitor(recorder.status_dict, cameras, host=mon_cfg.host,
                             port=int(mon_cfg.port),
                             datasets_root=str(cfg.recording.root))
        monitor.start()

    controller.start()
    recorder.start()

    fresh = not (writer.root / "meta").exists()
    print(f"[collect] dataset root: {writer.root}"
          + ("  (created when the first episode is saved)" if fresh else ""))
    print(f"[collect] task: {task!r}")
    print("[collect] A/X=start/stop  B/Y=discard  |  other hand: Y/B 1s=home, "
          "X/A 1s=delete last  |  keys: space=start/stop d=discard "
          "x=delete-last h=home c=calibrate-X a=save-anchor A=clear-anchor q=quit")

    watchdog = LoopWatchdog()
    try:
        with RawKeyboard() as kb:
            while True:
                watchdog.beat()
                key = kb.poll()
                if key == " ":
                    recorder.toggle_recording()
                elif key == "d":
                    recorder.discard()
                elif key == "x":
                    recorder.delete_last_or_discard()
                elif key == "h":
                    controller.request_home()
                elif key == "c" and quest is not None:
                    # Two-point X-axis calibration, done in the headset with
                    # the trigger; press c again to cancel. On success the
                    # axis is saved as the persistent anchor.
                    qs = quest.get()
                    if qs is not None and qs.calib_step > 0:
                        quest.send_calibrate_cancel()
                        print("\n[collect] X calibration cancelled")
                    else:
                        quest.send_calibrate_x(cfg.quest.control_hand)
                        print("\n[collect] X calibration: pull TRIGGER at "
                              "point 1, move along robot +X (>=15 cm), pull "
                              "TRIGGER at point 2. Arrow previews the axis; "
                              "menu button or c cancels. Teleop is paused.")
                elif key == "a" and quest is not None:
                    # Pin the current (calibrated) frame to the room: poses
                    # survive headset reboots. Do this AFTER verifying that
                    # forward on the controller is forward on the arm.
                    quest.send_anchor_save()
                    print("\n[collect] anchor save requested — controllers "
                          "buzz on success; status shows [anchor]")
                elif key == "A" and quest is not None:
                    quest.send_anchor_clear()
                    print("\n[collect] anchor cleared — poses back in stage "
                          "frame (yaw may drift across reboots)")
                elif key == "q":
                    break
                if viz is not None:
                    if quest is not None and (qs := quest.get()) is not None:
                        viz.log_quest(qs)
                    viz.log_arm(arm.get_state(), controller.status.target_pose6)
                    if cameras:
                        frames = {c.name: f.rgb for c in cameras
                                  if (f := c.latest()) is not None}
                        viz.log_images(frames)
                print(f"\r[collect] {recorder.status_line():<110}", end="", flush=True)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[collect] shutting down")
        watchdog.stop()
        if monitor is not None:
            monitor.stop()
        if streamer is not None:
            streamer.stop()
        recorder.stop()
        controller.stop()
        for cam in cameras:
            cam.stop()
        arm.stop()
        if quest is not None:
            quest.stop()
        print(f"[collect] dataset: {writer.root} — {len(writer.episodes)} episodes, "
              f"{writer.total_frames} frames")


if __name__ == "__main__":
    main()
