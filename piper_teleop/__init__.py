"""Quest 3 teleoperation + data collection for the AgileX Piper arm.

Pipeline: Quest APK (quest_app/) streams controller poses over USB (adb
forward) -> QuestReader -> TeleopController maps clutch deltas to absolute
EEF targets -> PiperArm streams move_p / gripper -> Recorder samples
50 Hz state/action + RealSense frames -> LeRobotWriter emits the
LeRobot v2.0 dataset (with GR00T-style modality metadata).
"""

__version__ = "0.1.0"
