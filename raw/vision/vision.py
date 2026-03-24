#!/usr/bin/env python3
"""
vision.py – threaded line detector + blue circle detector.

Thread layout:
  capture_thread  – reads frames from camera into a shared FrameBuffer.
                    VideoCapture is not thread-safe so it lives here alone.
  line_thread     – grabs latest frame, runs red-line detection at LINE_RATE_HZ.
  object_thread   – grabs latest frame, runs blue-circle detection at OBJECT_RATE_HZ.

Both detector threads call OpenCV / NumPy which release the GIL during their
C-extension work, so they genuinely run in parallel on separate cores.

Results are sent to the C++ process via a Unix domain SOCK_DGRAM socket.

Protocol (one ASCII datagram per message):
  LINE,<valid 0/1>,<lateral_error_m>,<heading_error_rad>,<curvature_1pm>
  DET,<cx>,<cy>,<w>,<h>,<score>       -- blue circle found (normalised coords)
  NODET                                -- no circle this frame
  LOCKED                               -- one-shot: full circle confirmed in frame

Usage:
  python3 vision.py [camera_source]
  camera_source: int (0 = /dev/video0) or MJPEG URL. Default: 0.
"""

import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────

SOCKET_PATH = "/tmp/robot_vision.sock"

# Line detector
LINE_ROI_Y_START         = 0.1
LINE_ROI_Y_END           = 0.6
LINE_ROI_X_START         = 0.10
LINE_ROI_X_END           = 0.90
LINE_H_MIN,  LINE_H_MAX  = 0,   10    # lower red (H wraps at 0)
LINE_H2_MIN, LINE_H2_MAX = 160, 180   # upper red
LINE_S_MIN,  LINE_S_MAX  = 100, 255
LINE_V_MIN,  LINE_V_MAX  = 100, 255
LINE_MIN_AREA_PX         = 500
LINE_CENTER_X_OFFSET     = 0          # px, compensates for off-centre camera mount
LINE_PIXELS_TO_M         = 0.001      # px → metres for lateral error

# Object detector (blue circle)
OBJ_H_MIN, OBJ_H_MAX    = 105, 125  # royal blue – tighter to avoid cyan/sky confusion
OBJ_S_MIN, OBJ_S_MAX    = 120, 255  # allow slight shadow variation
OBJ_V_MIN, OBJ_V_MAX    = 40,  255
OBJ_MIN_AREA_PX          = 500

# Pi camera resolution / framerate (used when camera_src is an integer)
CAM_W   = 640
CAM_H   = 480
CAM_FPS = 60

# Rates
LINE_RATE_HZ   = 60.0   # match camera framerate – faster corrections
OBJECT_RATE_HZ = 15.0

# ── Shared frame buffer ────────────────────────────────────────────────────────

class FrameBuffer:
    """Holds the most recent camera frame. Thread-safe."""
    def __init__(self):
        self._lock  = threading.Lock()
        self._frame = None          # latest BGR frame (numpy array)
        self._ready = threading.Event()

    def put(self, frame):
        with self._lock:
            self._frame = frame
        self._ready.set()

    def get(self):
        """Block until at least one frame is available, then return a copy."""
        self._ready.wait()
        with self._lock:
            return self._frame.copy()

# ── Socket helper ─────────────────────────────────────────────────────────────

class VisionSocket:
    """Thin wrapper around a SOCK_DGRAM Unix socket. Thread-safe (sendto is atomic)."""
    def __init__(self):
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def send(self, msg: str):
        try:
            self._sock.sendto(msg.encode(), SOCKET_PATH)
        except Exception:
            pass  # C++ side may not be ready yet; just drop

# ── Capture thread ────────────────────────────────────────────────────────────
# Two implementations:
#   capture_thread_picam – rpicam-vid pipe (Pi camera, never blocks on open)
#   capture_thread_cv    – OpenCV VideoCapture (MJPEG URLs / generic sources)

def capture_thread_picam(buf: FrameBuffer, stop: threading.Event):
    frame_bytes = CAM_W * CAM_H * 3 // 2  # YUV420 = W*H*1.5 bytes per frame
    cmd = [
        'rpicam-vid',
        '--width',     str(CAM_W),
        '--height',    str(CAM_H),
        '--framerate', str(CAM_FPS),
        '--codec',     'yuv420',
        '--nopreview',
        '-t', '0',   # run indefinitely
        '-o', '-',   # stdout
    ]
    print(f"[capture] rpicam-vid {CAM_W}x{CAM_H}@{CAM_FPS}fps → YUV420 pipe")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    try:
        while not stop.is_set():
            raw = proc.stdout.read(frame_bytes)
            if len(raw) < frame_bytes:
                print("[capture] rpicam-vid pipe closed")
                break
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((CAM_H * 3 // 2, CAM_W))
            buf.put(cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420))
    finally:
        proc.terminate()
        proc.wait()


def capture_thread_cv(cap: cv2.VideoCapture, buf: FrameBuffer, stop: threading.Event):
    while not stop.is_set():
        ret, frame = cap.read()
        if ret:
            buf.put(frame)
        else:
            print("[capture] WARN: frame read failed")
            time.sleep(0.02)

# ── Line detection ────────────────────────────────────────────────────────────

def _run_line_detection(frame) -> str:
    """Pure function: frame → protocol string. Releases GIL in OpenCV calls."""
    h, w = frame.shape[:2]

    y0 = int(h * LINE_ROI_Y_START);  y1 = int(h * LINE_ROI_Y_END)
    x0 = int(w * LINE_ROI_X_START);  x1 = int(w * LINE_ROI_X_END)
    roi = frame[y0:y1, x0:x1]
    roi_h, roi_w = roi.shape[:2]

    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lo1  = np.array([LINE_H_MIN,  LINE_S_MIN, LINE_V_MIN])
    hi1  = np.array([LINE_H_MAX,  LINE_S_MAX, LINE_V_MAX])
    lo2  = np.array([LINE_H2_MIN, LINE_S_MIN, LINE_V_MIN])
    hi2  = np.array([LINE_H2_MAX, LINE_S_MAX, LINE_V_MAX])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    M    = cv2.moments(mask)
    area = M['m00']

    if area < LINE_MIN_AREA_PX:
        return "LINE,0,0.0,0.0,0.0"

    cx              = M['m10'] / area
    image_cx        = roi_w / 2.0 + LINE_CENTER_X_OFFSET
    lateral_error_m = (cx - image_cx) * LINE_PIXELS_TO_M

    heading_rad = 0.0
    mid_y = roi_h // 2
    M_top = cv2.moments(mask[:mid_y, :])
    M_bot = cv2.moments(mask[mid_y:, :])
    if M_top['m00'] > 0 and M_bot['m00'] > 0:
        cx_top      = M_top['m10'] / M_top['m00']
        cx_bot      = M_bot['m10'] / M_bot['m00']
        heading_rad = float(np.arctan2(cx_top - cx_bot, float(mid_y)))

    return f"LINE,1,{lateral_error_m:.5f},{heading_rad:.5f},0.0"


def line_thread(buf: FrameBuffer, sock: VisionSocket, stop: threading.Event):
    period = 1.0 / LINE_RATE_HZ
    while not stop.is_set():
        t0    = time.monotonic()
        frame = buf.get()
        msg   = _run_line_detection(frame)
        sock.send(msg)
        elapsed = time.monotonic() - t0
        sleep_s = period - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

# ── Object detection ──────────────────────────────────────────────────────────

def _run_object_detection(frame):
    """Returns DET message string if any blue blob found, else NODET."""
    h, w = frame.shape[:2]

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([OBJ_H_MIN, OBJ_S_MIN, OBJ_V_MIN]),
        np.array([OBJ_H_MAX, OBJ_S_MAX, OBJ_V_MAX]),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best      = None
    best_area = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < OBJ_MIN_AREA_PX:
            continue
        if area > best_area:
            best_area = area
            (cx_px, cy_px), rad = cv2.minEnclosingCircle(cnt)
            best = (cx_px, cy_px, rad)

    if best is None:
        return "NODET"

    cx_px, cy_px, radius = best
    return (f"DET,{cx_px/w:.4f},{cy_px/h:.4f},"
            f"{2*radius/w:.4f},{2*radius/h:.4f},1.00")


def object_thread(buf: FrameBuffer, sock: VisionSocket, stop: threading.Event):
    period     = 1.0 / OBJECT_RATE_HZ
    last_log_t = 0.0

    while not stop.is_set():
        t0    = time.monotonic()
        frame = buf.get()
        msg   = _run_object_detection(frame)   # GIL released inside OpenCV
        sock.send(msg)

        now = time.monotonic()
        if msg != "NODET" and now - last_log_t >= 1.0:
            last_log_t = now
            print(f"[object] blue detected: {msg}")
        elif msg == "NODET" and now - last_log_t >= 3.0:
            last_log_t = now
            print("[object] no blue detected")

        elapsed = time.monotonic() - t0
        sleep_s = period - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    camera_src = sys.argv[1] if len(sys.argv) > 1 else "0"
    try:
        camera_src = int(camera_src)
    except ValueError:
        pass

    buf  = FrameBuffer()
    sock = VisionSocket()
    stop = threading.Event()

    # Pi camera (integer index) → rpicam-vid pipe (no blocking open, no GStreamer deps)
    # URL / MJPEG stream        → OpenCV VideoCapture
    if isinstance(camera_src, int):
        cap = None
        cap_t = threading.Thread(target=capture_thread_picam, args=(buf, stop),
                                 daemon=True, name="capture")
    else:
        print(f"[vision] opening stream: {camera_src}")
        cap = cv2.VideoCapture(camera_src)
        if not cap.isOpened():
            print(f"[vision] ERROR: could not open {camera_src}")
            sys.exit(1)
        cap_t = threading.Thread(target=capture_thread_cv, args=(cap, buf, stop),
                                 daemon=True, name="capture")

    threads = [
        cap_t,
        threading.Thread(target=line_thread,   args=(buf, sock, stop), daemon=True, name="line"),
        threading.Thread(target=object_thread, args=(buf, sock, stop), daemon=True, name="object"),
    ]

    for t in threads:
        t.start()
    print("[vision] capture / line / object threads running")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
        if cap is not None:
            cap.release()
        print("[vision] stopped")


if __name__ == "__main__":
    main()
