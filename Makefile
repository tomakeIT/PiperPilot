SHELL := /bin/bash
# Override when needed, e.g. `make test PY=/path/to/python`.
PY ?= python3
PIP ?= $(PY) -m pip

# Optional config overlay for the collect targets. By default NONE is
# applied — make collect uses piper_teleop/configs/default.yaml only.
# Opt in per run:  make collect CONFIG=first_run.yaml
CONFIG ?=
CONFIG_FLAG = $(if $(CONFIG),--config "$(CONFIG)")

.PHONY: help toolchain env can apk install-apk connect view collect collect-sm collect-sim test docs docs-serve docs-env

help:
	@echo "PiperPilot targets:"
	@echo "  toolchain    install Android SDK/NDK/JDK17/Gradle (one-time, ~2.5GB)"
	@echo "  env          create piper_teleop conda env + deps + pyAgxArm"
	@echo "  can          bring up can0 @ 1Mbps (sudo)"
	@echo "  apk          build the Quest APK"
	@echo "  connect      install/start APK + adb forward (wired link)"
	@echo "  view         live 3D visualization of controller poses"
	@echo "  collect      the ONE app: teleop + recording (press A/space to"
	@echo "               record; nothing saved until you do). Uses default.yaml;"
	@echo "               add an overlay with: make collect CONFIG=first_run.yaml"
	@echo "               variants: collect-sm (SpaceMouse), collect-sim (fake)"
	@echo "  test         run unit tests"

toolchain:
	bash install/01_android_toolchain.sh

env:
	bash install/02_python_env.sh

can:
	bash install/03_can_setup.sh

spacemouse:
	bash install/04_spacemouse_setup.sh

apk:
	bash quest_app/build.sh

install-apk: connect

connect:
	bash scripts/quest_connect.sh -p

home:
	$(PY) -m piper_teleop.apps.home_arm

view:
	$(PY) -m piper_teleop.apps.view_poses

# Single entry: collect IS the teleop app — recording starts only when you
# press A / space, and nothing is written to disk until an episode is saved.
collect:
	$(PY) -m piper_teleop.apps.collect_data $(CONFIG_FLAG) --task "$(TASK)"

collect-sm:
	$(PY) -m piper_teleop.apps.collect_data $(CONFIG_FLAG) --input spacemouse --task "$(TASK)"

collect-sim:
	$(PY) -m piper_teleop.apps.collect_data $(CONFIG_FLAG) --sim --task "sim test task"

test:
	cd $(CURDIR) && env PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PY) -m pytest tests/ -v

docs-env:
	$(PIP) install -r install/requirements-docs.txt

docs:
	cd $(CURDIR) && $(PY) -m mkdocs build --strict

docs-serve:
	cd $(CURDIR) && $(PY) -m mkdocs serve
