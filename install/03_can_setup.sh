#!/bin/bash
# Bring up the CAN interface for the Piper arm (1 Mbps). Idempotent; needs sudo.
set -e
CH=${1:-can0}

if ip link show "$CH" >/dev/null 2>&1; then
  STATE=$(cat "/sys/class/net/$CH/operstate" 2>/dev/null || echo unknown)
  if ip -details link show "$CH" | grep -q "bitrate 1000000" && [ "$STATE" = "up" ]; then
    echo "$CH already up at 1 Mbps"
    exit 0
  fi
  sudo ip link set "$CH" down || true
  sudo ip link set "$CH" up type can bitrate 1000000
  echo "$CH configured at 1 Mbps"
else
  echo "ERROR: $CH not found. Plug in the USB-CAN adapter (gs_usb/candleLight)." >&2
  echo "Available interfaces:"; ls /sys/class/net/
  exit 1
fi
ip -details link show "$CH" | head -3
