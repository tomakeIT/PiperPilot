# PiperTeleopTools

![PiperTeleopTools](assets/hero.svg)

**Teleoperate an AgileX Piper robot arm with a Meta Quest 3 (or a SpaceMouse), and
record your demonstrations straight into LeRobot-format datasets — ready for
robot-learning training, replay, and policy deployment.**

You move your hand; the arm follows in real time. Press one button and the
episode is saved as a fully-aligned dataset (robot state, actions, and up to
three camera views at 30 fps). When your policy is trained, the same toolkit
runs it back on the robot.

## What's in the box

<div class="grid cards" markdown>

- :material-safety-goggles: **Native Quest 3 app**

    ---

    A C++/OpenXR app (no Unity) streams both controllers' 6-DoF poses at 90 Hz
    over a wired USB link. Passthrough view, floating camera panels, and a
    recording HUD — you never take the headset off.

- :material-mouse: **SpaceMouse alternative**

    ---

    No headset? A 3Dconnexion SpaceMouse drives the arm with rate control —
    same pipeline, one flag.

- :material-robot-industrial: **Compliant by default**

    ---

    An MIT-mode impedance controller makes the arm behave like a spring:
    contact forces are bounded, so it's safe around objects and people.
    A stiff position mode is one flag away when you need millimetre tracking.

- :material-database: **LeRobot v2.0 datasets**

    ---

    Parquet + MP4 with timestamp-aligned streams, plus a provenance record of
    the exact controller, firmware, and gains used — so your policy always
    knows what it was trained on.

- :material-replay: **Replay & inference runtime**

    ---

    Replay any recorded episode on the real arm with an error report, or serve
    a trained policy over HTTP and run it closed-loop.

- :material-shield-check: **Safety envelope**

    ---

    Workspace box, velocity clamps, target-tethering that caps contact force,
    stream watchdogs, and fault-triggered stops — on every code path.

</div>

## See it in five minutes

No robot, no headset, no cameras needed — the whole pipeline runs in
simulation:

```bash
git clone https://github.com/agilexrobotics/pyAgxArm ~/pyAgxArm   # arm SDK
git clone https://github.com/tomakeIT/PiperTeleopTools.git && cd PiperTeleopTools
make env          # one-time: conda env + dependencies
make collect-sim  # fake arm + fake cameras + keyboard controls
```

Press ++space++, wait a few seconds, press ++space++ again — you just recorded
a LeRobot episode. The [Quick start](getting-started/quickstart.md) walks
through it and shows you what landed on disk.

## Where to go next

1. **[Quick start](getting-started/quickstart.md)** — try the pipeline with zero
   hardware.
2. **[Installation](getting-started/installation.md)** — set up the real
   hardware: arm, headset, cameras.
3. **[First run on the real arm](getting-started/first-teleop.md)** — a careful,
   safe first teleoperation session.
4. **[Collecting datasets](guide/data-collection.md)** — the day-to-day
   recording workflow.
5. **[Reference](reference/architecture.md)** — protocols, control theory,
   dataset schema, and every config key, once you need the details.

## Hardware at a glance

| Component | Notes |
|---|---|
| AgileX Piper / Piper-X | 6 DOF + gripper, USB-CAN adapter (gs_usb/candleLight) at 1 Mbps |
| Meta Quest 3 *(optional)* | Developer mode + USB data cable — or use a SpaceMouse |
| 3Dconnexion SpaceMouse *(optional)* | Compact / Wireless, USB |
| Intel RealSense *(optional)* | 1–3 cameras (e.g. D435i) for visual observations |
| Linux host | Ubuntu 22.04 verified; conda; ~2.5 GB Android toolchain only if you build the Quest APK yourself |

Everything is optional except the host — start in simulation and add hardware
piece by piece.

## System overview

![PiperTeleopTools system overview](assets/system-overview.svg)
