#!/bin/bash
# Create the "piper_teleop" conda env with all host-side dependencies and
# install pyAgxArm (editable, from $PYAGXARM_PATH) + this repo's package.
set -ex

HERE="$(cd "$(dirname "$0")" && pwd)"
ENV_NAME=${ENV_NAME:-piper_teleop}
PYAGXARM_PATH=${PYAGXARM_PATH:-$HOME/pyAgxArm}
CONDA_BIN=${CONDA_BIN:-$HOME/miniforge3/bin/conda}

if ! "$CONDA_BIN" env list | grep -qE "envs/$ENV_NAME\$|^$ENV_NAME "; then
  "$CONDA_BIN" create -y -n "$ENV_NAME" python=3.11
fi

run_in_env() {
  "$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" "$@"
}

run_in_env python -m pip install -r "$HERE/requirements.txt"

# Arm SDK: prefer the local checkout (editable) so SDK fixes are picked up.
if [ -d "$PYAGXARM_PATH" ]; then
  run_in_env python -m pip install -e "$PYAGXARM_PATH"
else
  echo "WARNING: $PYAGXARM_PATH not found — clone https://github.com/agilexrobotics/pyAgxArm there" >&2
  exit 1
fi

# This repo's package (editable) + CLI entry points.
run_in_env python -m pip install -e "$HERE/.."

# Record the exact resolved versions for reproducibility.
run_in_env python -m pip freeze > "$HERE/lock_$(date +%Y%m%d).txt"

echo "=== PYTHON ENV READY ==="
run_in_env python - <<'EOF'
import numpy, pandas, pyarrow, av, cv2, rerun, can, yaml
import pyAgxArm
print("pyAgxArm", pyAgxArm.__version__)
try:
    import pyrealsense2 as rs
    print("pyrealsense2 OK,", len(rs.context().devices), "device(s) connected")
except Exception as e:
    print("pyrealsense2 import issue:", e)
print("all imports OK")
EOF
echo "Activate with: conda activate piper_teleop"
