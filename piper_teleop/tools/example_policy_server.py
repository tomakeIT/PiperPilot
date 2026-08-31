"""Minimal reference policy server implementing the RemotePolicy protocol.

Run next to your model (GPU box):
    python -m piper_teleop.tools.example_policy_server --port 8901

Then on the robot host:
    piper-infer --policy http://<gpu-host>:8901

Replace `predict()` with real model inference (decode the base64 JPEGs,
run the VLA, return an (H, 8) absolute-action chunk: pos3 + quat_wxyz4 +
gripper_width1 at the dataset rate).
"""

from __future__ import annotations

import argparse
import base64  # noqa: F401  (decode images in a real implementation)
import json
from http.server import BaseHTTPRequestHandler, HTTPServer


def predict(payload: dict) -> list[list[float]]:
    """STUB: hold the current pose for the requested horizon."""
    eef = payload["state"]["observation.arm_eef"]
    grip = payload["state"]["observation.gripper"]
    step = list(map(float, eef)) + [float(grip[0])]
    return [step] * int(payload.get("horizon", 30))


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):  # noqa: N802
        if self.path != "/predict":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0))
        payload = json.loads(self.rfile.read(length))
        actions = predict(payload)
        body = json.dumps({"actions": actions}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *fmt_args):
        pass  # quiet


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8901)
    args = ap.parse_args()
    print(f"example policy server on :{args.port} (POST /predict)")
    HTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
