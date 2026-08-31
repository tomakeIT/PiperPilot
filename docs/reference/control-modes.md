# Control modes & tuning

PiperPilot drives the real arm through one of two control backends,
selected by `arm.backend` in the config (or by CLI shortcut). Both accept the
same stream of absolute EEF pose targets from the teleop loop — they differ in
*where* inverse kinematics runs and *how* the joints are servoed:

- **`piper_mit`** (the default, CLI `--impedance`) — compliant MIT impedance
  control. The host solves IK and streams per-joint spring-damper commands.
  Contact force is bounded, so the arm yields when it touches something.
- **`piper`** (CLI `--position`) — firmware position control via `move_p`.
  Tightest tracking, but rigid: the position servo pushes through contact.

A third backend, `fake` (CLI `--sim`), simulates the arm for pipeline testing
and is not covered here — see [Architecture](architecture.md).

## Choosing a mode

| | `piper_mit` (default) | `piper` (`--position`) |
|---|---|---|
| Principle | Host DLS-IK → per-joint MIT impedance (`T = kp·(q_des − q) − kd·v + t_ff`) + torque-snapshot gravity feed-forward | `move_p` streams EEF poses; IK runs in the arm firmware |
| Contact behaviour | **Compliant** — force bounded ≈ cartesian stiffness × 0.12 m tether (~20 N with shipped gains) | Rigid (position servo) |
| Measured tracking | 14.1 mm dynamic RMS, 11 mm static hold error (shipped soft gains; higher kp tracks tighter) | **1.34 mm dynamic RMS, ≈0 static error** |
| Estimated latency | ~90 ms | ~10 ms |
| When to use | Everyday data collection, contact-rich tasks, safe RL exploration | High-precision free-space tasks |

Numbers were measured on a Piper-X with firmware S-V1.8-9 using
`piper-track-test` (3 cm circle, 5 s per lap) — see
[Benchmarks](benchmarks.md) for the methodology.

## Position mode (`piper`, `--position`)

The simplest backend: every teleop tick (at `teleop.rate_hz`, 100 Hz by
default) the host calls `move_p([x, y, z, roll, pitch, yaw])` with the
absolute EEF target. The firmware does the rest:

- **IK runs in the firmware.** The host never touches joint space.
- **Target-overwrite semantics** make streaming at 50–200 Hz safe — each new
  target simply replaces the previous one.
- Units are meters/radians in the base frame, with
  `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`. The SDK hard-limits pitch to
  ±90°; the teleop safety envelope stays clear of it with
  `teleop.limits.pitch_abs_max_deg: 88.0`.
- `arm.speed_percent` scales the firmware's motion speed.

This gives the tightest tracking the arm can do (1.34 mm dynamic RMS, ~10 ms
latency, essentially zero static error). The trade-off: on contact the
position servo keeps pushing. Use it for free-space precision work, not for
tasks where the gripper is expected to touch the world.

## Impedance mode (`piper_mit`, default)

In impedance mode the arm behaves like a 6-joint spring-damper pulled toward a
streamed joint target. The command path, in order:

### 1. Host-side DLS-IK

The firmware only does IK for `move_p`, so joint-space (`move_mit`) control
needs the host to convert EEF pose targets into joint targets. The solver is
damped least squares on pyAgxArm's MDH forward-kinematics model:

- Warm-started from the previous joint target and fed small per-tick deltas,
  so the configured 4 iterations converge comfortably at 100 Hz. The numeric
  Jacobian costs 6 extra FK calls per iteration; the 6-link MDH FK is
  ~0.1 ms in pure Python.
- Per-iteration joint updates are clamped to ±0.2 rad to keep iterates tame
  near singularities, and every iterate is clamped to the joint limits.
- If the solver does **not** converge (unreachable or singular target), the
  backend holds the previous joint target instead of chasing a bad solution.

The solver knobs live under `impedance.ik`:

| Key | Default | Meaning |
|---|---|---|
| `ik.iters` | `4` | DLS iterations per tick |
| `ik.damping` | `0.05` | DLS lambda |
| `ik.pos_tol` | `0.002` | position convergence tolerance (m) |

### 2. Per-joint MIT impedance law

Each of the 6 joints receives a `move_mit` frame implementing

```
T = kp * (q_des - q) - kd * qdot + t_ff
```

with per-joint gains from the config:

```yaml
impedance:
  kp: [5, 25, 5, 8, 8, 5]                # N*m/rad (SDK reference: 10)
  kd: [0.3, 1.2, 0.3, 0.45, 0.45, 0.3]   # damping (SDK reference: 0.8)
  t_ff: [0, 0, 0, 0, 0, 0]               # manual feed-forward trim (N*m)
```

### 3. Per-tick joint velocity clamp

Each tick, the new IK solution may move the joint target by at most
`impedance.max_joint_vel / teleop.rate_hz` — with the defaults
(1.5 rad/s at 100 Hz) that is 0.015 rad per joint per tick. Sudden input
jumps therefore turn into bounded-speed motion, never a lunge.

### 4. SDK joint-limit intersection

Your configured `impedance.joint_limits_deg` are automatically intersected
with the SDK's per-model limit preset (with a 2° safety margin), and the
effective limits are printed at startup. This matters because a joint clamped
by the SDK/firmware while the impedance loop keeps pulling on it buzzes and
drifts — the IK must never command what the lower layers would clamp.

!!! note "Piper-X wrist limits"
    On the Piper-X, joints 4 and 5 are ±89° in the SDK preset — tighter than
    you might expect. Whatever you put in `joint_limits_deg`, the
    intersection wins. Check the `[piper-mit] joint limits (deg): ...`
    startup line for the values actually in force.

## What happens at MIT entry

Switching a loaded arm from the firmware position servo to impedance control
is the most delicate moment in the whole stack. The entry sequence in
`PiperArmMIT` exists to make that handover torque-continuous:

1. **Latch the target from measured joints.** The first joint target is a
   copy of the measured joint angles, so the arm is never asked to jump.
2. **Torque snapshot — taken *before* the mode switch.** With `gravity_ff:
   true` (default), the backend samples each motor's holding torque 5 times
   over 100 ms while the position servo is still active. That average is
   exactly the gravity-plus-friction torque needed to hold the entry pose. It
   is applied as feed-forward **unramped from the first MIT frame**: it
   replaces exactly what the position servo was outputting, so the handover
   produces no torque step — no clunk, no droop at the entry pose.
3. **Mode switch, sent once.** Automatic mode re-sending is disabled and
   `set_motion_mode('mit')` is issued a single time before streaming begins.
4. **Soft-start gain ramp.** `kp` and `kd` ramp from ~0 to 100 % over
   `soft_start_s` (0.8 s default). Without the ramp, full spring stiffness
   snapping onto the measured pose produces the audible handover "clunk".
   The gravity snapshot deliberately does *not* ramp (see step 2).
5. **Post-ramp deflection check.** 0.7 s after the ramp completes, a one-shot
   sanity check compares measured joints against the target. A wrong-signed
   torque snapshot would *double* the gravity deflection instead of
   cancelling it — so if any joint has deflected more than 0.2 rad, the
   snapshot is zeroed with a warning and the arm falls back to plain
   spring-damper behaviour.
6. **50 Hz keepalive.** A background thread re-sends the hold frame whenever
   no command has gone out for 40 ms. This lets the gain ramp complete even
   if teleop is idle, and guarantees the firmware always has fresh impedance
   targets — the spring behaviour with the clutch released is well-defined,
   not "whatever the last frame said".
7. **`engage_on_start`.** With the default `true`, the backend enters
   impedance hold right at startup (retrying for up to 3 s while joint
   feedback comes up). Set it to `false` to defer entry to the first clutch
   engage.
8. **Exit handback.** `stop()` (and any firmware-planned move such as homing)
   leaves MIT mode by re-enabling automatic mode handling and issuing a
   `move_j` at the measured joints — a firmware position hold. The arm cannot
   slump when the process exits.

One related subtlety: under gravity an impedance-controlled arm sags slightly
below its commanded target. The clutch therefore latches onto the *held
target* pose (forward kinematics of the joint target), not the sagged
measured pose — otherwise every clutch re-engage would ratchet the arm
downward.

## Bounded contact force

The teleop safety envelope tethers the commanded target to the measured EEF
pose: `teleop.limits.max_cmd_deviation: 0.12` m. In impedance mode this turns
stiffness into a force bound:

> contact force ≈ cartesian stiffness × 0.12 m tether ≈ **~20 N** with the
> shipped gains.

If the gripper presses into a surface (or a human hand), the target can lead
the measured pose by at most 12 cm, so the spring can pull with at most that
bounded force — no matter how far the operator keeps moving. This is why
impedance mode is the default for contact-rich demonstrations and for safe RL
exploration: the worst-case interaction force is a config-determined
constant, not a function of operator error.

## Tuning guide

Gains are tuned on the robot, by symptom:

| Symptom | Knob |
|---|---|
| Arm droops under gravity away from the entry pose | Raise `kp` on the drooping joint(s) |
| Audible hum or buzz | Lower `kp` and `kd` — too-high `kd` amplifies velocity-sensor noise into hum (SDK reference `kd` is 0.8; the shipped values are lower) |
| One joint feels hard to push during the gravity test | Lower that joint's `kp`/`kd` |
| Tracking feels loose or sluggish | Raise `kp` (higher kp tracks tighter); back off as soon as it starts to hum |

### The gravity-test probe

`gravity_test.yaml` (in the repository root) is a near-zero-gain overlay for
feeling each joint by hand and probing the firmware's own gravity behaviour:

```yaml
impedance:
  kp: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
  kd: [1.0, 1.5, 1.2, 0.5, 0.5, 0.3]
  engage_on_start: true
```

```bash
piper-collect --impedance --config gravity_test.yaml
```

With `kp` ~0 there is no spring holding the arm up; `kd` keeps any fall slow
and damped. Watch the arm for ~20 s after `MIT mode active` (keep your hands
ready to catch it):

- **Arm holds its pose** → the firmware *does* gravity-compensate in MIT
  mode.
- **Arm sinks slowly** → no firmware gravity compensation (the droop was
  previously masked by friction); host-side `t_ff` gravity feed-forward would
  be needed.

While it runs, push each joint by hand — this is the easiest way to feel
which joint's damping is off before touching the per-joint `kd` values.

!!! warning "Gains are part of the dataset provenance"
    The control backend and its gains (`backend`, `kp`, `kd`, `t_ff`,
    `gravity_ff`, firmware, fps) are recorded per session in the dataset's
    provenance metadata, because a learned policy is bound to the controller
    it was trained against. Retuning between sessions of one dataset prints a
    prominent warning at collection time. See
    [Dataset format](dataset-format.md).

## Related pages

- [Configuration](configuration.md) — the full `impedance:` and `teleop:` key
  reference, and how `--config` overlays work.
- [Benchmarks](benchmarks.md) — how the tracking numbers were measured.
- [CLI reference](cli.md) — `--position`, `--impedance`, `--sim` and friends.
- [Troubleshooting](../troubleshooting.md) — includes the impedance-hum entry.
