#!/bin/bash
# Bring up the wired Quest link end-to-end:
#   1. find adb + the Quest device (USB)
#   2. (re)install the teleop APK if needed (-i forces reinstall)
#   3. start the app, set up `adb forward` (all data over the USB cable)
#   4. optionally disable the proximity sensor so the headset keeps tracking
#      while resting on a table (-p)
set -e
cd "$(dirname "$0")/.."

FORCE_INSTALL=0
PROX_OFF=0
while getopts "ip" opt; do
  case $opt in
    i) FORCE_INSTALL=1 ;;
    p) PROX_OFF=1 ;;
    *) echo "usage: $0 [-i reinstall apk] [-p disable proximity sensor]"; exit 1 ;;
  esac
done

PKG=com.pipertools.questteleop
APK=quest_app/app/build/outputs/apk/debug/app-debug.apk
PORT=8735

# Prefer the SDK adb (newer than the apt one).
for CAND in "$HOME/android-sdk/platform-tools/adb" "$(command -v adb || true)"; do
  [ -x "$CAND" ] && ADB="$CAND" && break
done
[ -n "${ADB:-}" ] || { echo "ERROR: adb not found"; exit 1; }
echo "adb: $ADB ($($ADB version | head -1))"

DEVICES=$($ADB devices | awk 'NR>1 && $2=="device" {print $1}')
if [ -z "$DEVICES" ]; then
  echo "ERROR: no authorized device. Plug in the Quest via USB and accept the"
  echo "       'Allow USB debugging' dialog inside the headset."
  $ADB devices
  exit 1
fi
echo "device: $DEVICES"

if [ "$FORCE_INSTALL" = 1 ] || ! $ADB shell pm list packages | grep -q "$PKG"; then
  [ -f "$APK" ] || { echo "APK missing — run quest_app/build.sh first"; exit 1; }
  echo "installing $APK …"
  $ADB install -r "$APK"
fi

# Wake the device and launch the app.
$ADB shell input keyevent KEYCODE_WAKEUP || true
$ADB shell am start -n "$PKG/android.app.NativeActivity"

# Wired transport over USB: 8735 = pose/action stream, 8736 = camera video.
$ADB forward tcp:$PORT tcp:$PORT
$ADB forward tcp:8736 tcp:8736
echo "adb forward tcp:$PORT (pose) + tcp:8736 (video) active"

if [ "$PROX_OFF" = 1 ]; then
  $ADB shell am broadcast -a com.oculus.vrpowermanager.prox_close || true
  echo "proximity sensor disabled (re-enable: adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable)"
fi

echo
echo "Quest teleop link ready — test with:  make view   (or piper-view)"
echo "App log:  $ADB logcat -s QuestTeleop"
