#!/bin/bash
# One-time Android toolchain setup for building the Quest APK.
# Installs: JDK 17 (conda env "jdk17"), Android cmdline-tools + SDK 32 +
# build-tools 34 + NDK 26 + CMake 3.22, Gradle 8.7.
# Everything goes under $HOME/android-sdk (no sudo needed). ~2.5 GB download.
set -ex

SDK=${ANDROID_HOME:-$HOME/android-sdk}
CONDA_BIN=${CONDA_BIN:-$HOME/miniforge3/bin/conda}
mkdir -p "$SDK"
cd "$SDK"

# 1) Android cmdline-tools
if [ ! -d "$SDK/cmdline-tools/latest" ]; then
  curl -sSL -o cmdtools.zip https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip
  rm -rf cmdline-tools-tmp && unzip -q cmdtools.zip -d cmdline-tools-tmp
  mkdir -p cmdline-tools
  mv cmdline-tools-tmp/cmdline-tools cmdline-tools/latest
  rm -rf cmdline-tools-tmp cmdtools.zip
fi

# 2) JDK 17 (sdkmanager and AGP 8 both require it; system java 11 is too old)
if [ ! -x "$HOME/miniforge3/envs/jdk17/bin/java" ]; then
  "$CONDA_BIN" create -y -n jdk17 -c conda-forge 'openjdk=17'
fi
export JAVA_HOME="$HOME/miniforge3/envs/jdk17"
export PATH="$JAVA_HOME/bin:$PATH"
java -version

export ANDROID_HOME=$SDK
SDKM="$SDK/cmdline-tools/latest/bin/sdkmanager"

# 3) SDK components
yes | "$SDKM" --licenses >/dev/null || true
"$SDKM" --install \
  "platform-tools" \
  "platforms;android-32" \
  "build-tools;34.0.0" \
  "ndk;26.3.11579264" \
  "cmake;3.22.1"

# 4) Gradle 8.7 standalone
if [ ! -d "$SDK/gradle-8.7" ]; then
  curl -sSL -o gradle.zip https://services.gradle.org/distributions/gradle-8.7-bin.zip \
    || curl -sSL -o gradle.zip https://mirrors.cloud.tencent.com/gradle/gradle-8.7-bin.zip
  unzip -q gradle.zip -d "$SDK" && rm gradle.zip
fi

echo "=== ANDROID TOOLCHAIN READY ==="
"$SDK/gradle-8.7/bin/gradle" --version | head -8
ls "$SDK/ndk" "$SDK/platforms" "$SDK/build-tools"
