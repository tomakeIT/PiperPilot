#!/bin/bash
# SpaceMouse (3Dconnexion) hidraw permissions + read test.
# The python driver reads HID reports directly (hidapi package, already in
# requirements.txt) — no spacenavd needed. If spacenavd is installed and
# grabs the device, stop it: sudo systemctl stop spacenavd.
set -e

ENV_NAME=${ENV_NAME:-piper_teleop}
CONDA_BIN=${CONDA_BIN:-$HOME/miniforge3/bin/conda}

# udev: allow non-root access to 3Dconnexion devices (VID 256f, legacy 046d).
RULE=/etc/udev/rules.d/99-spacemouse.rules
if [ ! -f "$RULE" ]; then
  echo "writing $RULE (sudo)…"
  sudo tee "$RULE" >/dev/null <<'EOF'
# hidraw nodes (hidraw-backend hidapi)
KERNEL=="hidraw*", ATTRS{idVendor}=="256f", MODE="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c626", MODE="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c628", MODE="0666"
# raw usb device nodes (libusb-backend hidapi, e.g. conda-forge build)
SUBSYSTEM=="usb", ATTR{idVendor}=="256f", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="046d", ATTR{idProduct}=="c626", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="046d", ATTR{idProduct}=="c628", MODE="0666"
EOF
  sudo udevadm control --reload-rules
  sudo udevadm trigger --action=add --subsystem-match=usb
  sudo udevadm trigger --subsystem-match=hidraw
  echo "udev rule installed — re-plug the SpaceMouse if reads fail"
fi

echo "=== test read (deflect the puck / press buttons for 5 s) ==="
"$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<'EOF'
import time
from piper_teleop.spacemouse_client import SpaceMouseReader
sm = SpaceMouseReader()
t0 = time.time()
while time.time() - t0 < 5.0:
    st = sm.get()
    t = st.twist
    print(f"\rx={t[0]:+.2f} y={t[1]:+.2f} z={t[2]:+.2f} "
          f"roll={t[3]:+.2f} pitch={t[4]:+.2f} yaw={t[5]:+.2f} "
          f"buttons={st.buttons} rate={sm.rate_hz():5.1f}Hz ", end="")
    time.sleep(0.02)
sm.stop()
print("\nOK")
EOF
