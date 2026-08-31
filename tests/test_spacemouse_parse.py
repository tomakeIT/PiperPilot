"""HID report parsing tests (SpaceMouse Compact layout)."""

import struct

import numpy as np

from piper_teleop.spacemouse_client import AXIS_SCALE, parse_report


def make(rid, *vals):
    return bytes([rid]) + struct.pack(f"<{len(vals)}h", *vals)


def test_translation_report():
    twist = np.zeros(6)
    buttons = [0, 0]
    # raw x=+350 (right), y=+350, z=+350
    assert parse_report(make(1, 350, 350, 350), twist, buttons)
    assert twist[0] == 1.0        # x: +raw
    assert twist[1] == -1.0       # y: -raw (forward positive after flip)
    assert twist[2] == -1.0       # z: -raw (up positive after flip)
    assert np.all(twist[3:] == 0)


def test_rotation_report_order_is_pitch_roll_yaw():
    twist = np.zeros(6)
    buttons = [0, 0]
    # report bytes order: pitch, roll, yaw
    assert parse_report(make(2, 100, 200, 300), twist, buttons)
    assert twist[4] == np.float64(np.clip(100 * -1 / AXIS_SCALE, -1, 1))  # pitch
    assert twist[3] == np.float64(np.clip(200 * -1 / AXIS_SCALE, -1, 1))  # roll
    assert twist[5] == np.float64(np.clip(300 * 1 / AXIS_SCALE, -1, 1))   # yaw
    assert np.all(twist[:3] == 0)


def test_combined_wireless_report():
    twist = np.zeros(6)
    buttons = [0, 0]
    assert parse_report(make(1, 350, 0, 0, 0, 0, -350), twist, buttons)
    assert twist[0] == 1.0
    assert twist[5] == -1.0


def test_button_report():
    twist = np.zeros(6)
    buttons = [0, 0]
    assert parse_report(bytes([3, 0b01]), twist, buttons)
    assert buttons == [1, 0]
    assert parse_report(bytes([3, 0b11]), twist, buttons)
    assert buttons == [1, 1]
    assert parse_report(bytes([3, 0b00]), twist, buttons)
    assert buttons == [0, 0]


def test_clamping_and_garbage():
    twist = np.zeros(6)
    buttons = [0, 0]
    assert parse_report(make(1, 500, -500, 0), twist, buttons)
    assert twist[0] == 1.0 and twist[1] == 1.0
    assert not parse_report(b"", twist, buttons)
    assert not parse_report(bytes([9, 1, 2]), twist, buttons)
