#!/bin/bash
# Build the Quest teleop APK. Requires install/01_android_toolchain.sh to have run.
set -e
cd "$(dirname "$0")"

export ANDROID_HOME="${ANDROID_HOME:-$HOME/android-sdk}"
export JAVA_HOME="${JAVA_HOME:-$HOME/miniforge3/envs/jdk17}"
GRADLE="${GRADLE:-$ANDROID_HOME/gradle-8.7/bin/gradle}"

if [ ! -x "$GRADLE" ]; then
    echo "gradle not found at $GRADLE — run install/01_android_toolchain.sh first" >&2
    exit 1
fi

echo "sdk.dir=$ANDROID_HOME" > local.properties

"$GRADLE" --no-daemon assembleDebug "$@"

APK=app/build/outputs/apk/debug/app-debug.apk
echo
echo "APK ready: $(realpath "$APK")"
