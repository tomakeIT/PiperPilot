# External datasets: frame conventions and conversion

External robot datasets often use different end-effector frames, quaternion
orders, and gripper units. Replaying them without conversion can cause IK
failures, out-of-bounds trajectories, or incorrect gripper motion. This page
describes a public, dataset-agnostic validation and conversion workflow. Always
complete the simulation checks before validating a dataset on real hardware.

## Common differences

- **Position frame:** world, robot base, or a frame relative to the episode's
  initial pose.
- **End-effector orientation:** an absolute flange orientation or a rotation
  relative to a neutral tool pose.
- **Quaternion order:** `wxyz` or `xyzw`.
- **Relative rotation convention:** world-frame and body-frame deltas are not
  interchangeable.
- **Gripper units:** metric width, normalized values, or raw device readings.

## Dataset sidecar

Store dataset-specific transforms in `meta/piper_replay.json` instead of
hard-coding them into the robot runtime:

```json
{
  "rot_offset_deg": [-90.0, 0.0, -90.0],
  "offset_m": [0.30, 0.0, 0.20]
}
```

These values are examples only and must not be reused for a new dataset.
Determine the transform independently from the dataset documentation,
visualization, and offline IK checks.

## Toolchain

- `piper-viz --root <dataset> --episode 0`: inspect the workspace, trajectory,
  and orientation axes.
- `piper-replay --sim --root <dataset>`: apply the sidecar on the simulated
  backend and exercise the safety guards.
- `piper-replay --root <dataset>`: perform a low-speed hardware validation only
  after manual review.
- `piper-convert --root <dataset>`: create a one-time copy in the Piper frame
  and record provenance in `meta/frame_conversion.json`.

The converted dataset does not retain `piper_replay.json`, which prevents the
transform from being applied twice during replay. Recompute statistics from
the converted values rather than carrying over normalization statistics from
the source frame.

## Recommended workflow

1. Read the dataset's coordinate-frame, quaternion, and gripper definitions.
2. Visualize one trajectory and inspect its position range and end-effector
   axes.
3. Compute IK feasibility offline and confirm that every target lies inside the
   configured workspace.
4. Add the sidecar and replay with `--sim`.
5. Use `piper-convert` to create a new copy and recompute its statistics.
6. For hardware validation, reduce the speed, keep the emergency stop within
   reach, and begin with a short, obstacle-free trajectory.

!!! danger "Do not leave frame conversion in the training data loader"
    If normalization statistics are computed before conversion, the checkpoint
    records statistics for the wrong frame. Denormalizing at deployment then
    amplifies the same error. Prefer one conversion at the dataset boundary so
    training, evaluation, and deployment all operate in the robot's native
    frame.
