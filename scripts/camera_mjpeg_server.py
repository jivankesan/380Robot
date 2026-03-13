#!/usr/bin/env python3
"""
Minimal MJPEG HTTP server using picamera2.
Runs natively on the Pi (no ROS needed) and streams frames to any HTTP client.

Usage:
    python3 scripts/camera_mjpeg_server.py
    python3 scripts/camera_mjpeg_server.py --port 8081 --width 640 --height 480 --fps 58
"""

import argparse
import io
import logging
import socketserver
import threading
import time
from http import server

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("camera_mjpeg_server")


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    output: StreamingOutput = None

    def log_message(self, fmt, *args):
        pass  # suppress per-request logs

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
            )
            self.end_headers()
            try:
                while True:
                    with StreamingHandler.output.condition:
                        StreamingHandler.output.condition.wait()
                        frame = StreamingHandler.output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass
        elif self.path == "/health":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"ok")
        else:
            self.send_error(404)


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=58)
    args = parser.parse_args()

    picam = Picamera2()
    config = picam.create_video_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls={"FrameRate": args.fps},
    )
    picam.configure(config)

    output = StreamingOutput()
    StreamingHandler.output = output

    picam.start_recording(MJPEGEncoder(), FileOutput(output))
    log.info(f"Camera started at {args.width}x{args.height} @ {args.fps}fps")

    address = ("0.0.0.0", args.port)
    httpd = StreamingServer(address, StreamingHandler)
    log.info(f"MJPEG stream at http://localhost:{args.port}/stream")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        picam.stop_recording()
        log.info("Camera stopped")


if __name__ == "__main__":
    main()
