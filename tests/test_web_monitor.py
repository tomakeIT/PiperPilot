import json
import urllib.error
import urllib.request

import numpy as np

from piper_teleop.web_monitor import WebMonitor


class StubRecorder:
    def status_dict(self):
        return {"state": "ready", "episode_index": None, "elapsed_s": 0.0,
                "frames": 0, "eps_saved": 2, "total_frames": 655,
                "engaged": False, "input_hz": 90.0, "task": "t", "root": "/r",
                "error": ""}


class StubFrame:
    def __init__(self):
        self.rgb = np.zeros((8, 8, 3), dtype=np.uint8)


class StubCam:
    name = "cam_front"

    def latest(self):
        return StubFrame()


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, r.headers.get("Content-Type"), r.read()


def test_web_monitor_endpoints():
    mon = WebMonitor(StubRecorder().status_dict, [StubCam()],
                     host="127.0.0.1", port=0)
    assert mon.start()
    base = f"http://127.0.0.1:{mon.port}"
    try:
        code, ctype, body = _get(base + "/status")
        assert code == 200
        assert ctype.startswith("application/json")
        assert json.loads(body)["eps_saved"] == 2

        code, _, body = _get(base + "/")
        assert code == 200
        assert b"cam_front" in body

        code, ctype, body = _get(base + "/cam/cam_front.jpg")
        assert code == 200
        assert ctype == "image/jpeg"
        assert body[:2] == b"\xff\xd8"  # JPEG SOI marker

        try:
            _get(base + "/nope")
            raise AssertionError("expected 404")
        except urllib.error.HTTPError as e:
            assert e.code == 404
    finally:
        mon.stop()


def _make_fake_dataset(root, name, cam="cam_front"):
    """Minimal LeRobot layout: meta/info.json + one 2-frame episode video."""
    import cv2
    ds = root / name
    (ds / "meta").mkdir(parents=True)
    (ds / "meta" / "info.json").write_text("{}")
    vdir = ds / "videos" / "chunk-000" / f"observation.images.{cam}"
    vdir.mkdir(parents=True)
    vw = cv2.VideoWriter(str(vdir / "episode_000000.mp4"),
                         cv2.VideoWriter_fourcc(*"mp4v"), 30, (16, 16))
    if not vw.isOpened():
        return None
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    frame[:8] = 200  # non-trivial content so decode failures are visible
    vw.write(frame)
    vw.write(frame)
    vw.release()
    return ds


def test_web_monitor_camera_calibrate(tmp_path):
    import pytest
    if _make_fake_dataset(tmp_path, "ds_a") is None:
        pytest.skip("cv2 build lacks an mp4 encoder")
    mon = WebMonitor(StubRecorder().status_dict, [StubCam()],
                     host="127.0.0.1", port=0, datasets_root=str(tmp_path))
    assert mon.start()
    base = f"http://127.0.0.1:{mon.port}"
    try:
        code, _, body = _get(base + "/")
        assert code == 200
        assert b"ref-cam_front" in body  # overlay img is in the page

        code, ctype, body = _get(base + "/calib/datasets")
        assert code == 200
        assert json.loads(body)["datasets"] == ["ds_a"]

        code, ctype, body = _get(base + "/calib/ref/cam_front.jpg?dataset=ds_a")
        assert code == 200
        assert ctype == "image/jpeg"
        assert body[:2] == b"\xff\xd8"

        for bad in ["/calib/ref/cam_front.jpg?dataset=nope",
                    "/calib/ref/cam_front.jpg?dataset=..%2Fds_a",
                    "/calib/ref/cam_front.jpg",
                    "/calib/ref/other_cam.jpg?dataset=ds_a"]:
            try:
                _get(base + bad)
                raise AssertionError(f"expected 404 for {bad}")
            except urllib.error.HTTPError as e:
                assert e.code == 404
    finally:
        mon.stop()


def test_web_monitor_calib_disabled_without_root():
    mon = WebMonitor(StubRecorder().status_dict, [], host="127.0.0.1", port=0)
    assert mon.start()
    try:
        code, _, body = _get(f"http://127.0.0.1:{mon.port}/calib/datasets")
        assert code == 200
        assert json.loads(body)["datasets"] == []
    finally:
        mon.stop()


def test_web_monitor_port_conflict():
    a = WebMonitor(StubRecorder().status_dict, [], host="127.0.0.1", port=0)
    assert a.start()
    b = WebMonitor(StubRecorder().status_dict, [], host="127.0.0.1", port=a.port)
    try:
        assert b.start() is False  # refuses politely, never raises
    finally:
        a.stop()
