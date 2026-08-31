#!/bin/bash
# Quest (Meta/Oculus, VID 2833) USB debugging permissions. One-time, needs sudo.
set -e

RULE=/etc/udev/rules.d/51-oculus.rules
echo "writing $RULE (sudo)…"
sudo tee "$RULE" >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

ADB=${ADB:-$HOME/android-sdk/platform-tools/adb}
"$ADB" kill-server 2>/dev/null || true
sleep 1
"$ADB" devices -l
echo
echo "If the device shows 'unauthorized', put on the headset and accept the"
echo "'Allow USB debugging' dialog, then re-run: $ADB devices"
