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

import math
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
OBJ_H_MIN, OBJ_H_MAX    = 100, 130
OBJ_S_MIN, OBJ_S_MAX    = 150, 255
OBJ_V_MIN, OBJ_V_MAX    = 50,  255
OBJ_MIN_AREA_PX          = 500
OBJ_CIRCULARITY_THRESH   = 0.75
OBJ_FRAME_MARGIN_PX      = 5

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
    """Returns (msg, locked) where locked=True triggers a one-shot LOCKED send."""
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

    best_circular      = None
    best_circ_val      = 0.0
    best_fallback      = None
    best_fallback_area = 0.0

    for cnt in contours:
        area  = cv2.contourArea(cnt)
        if area < OBJ_MIN_AREA_PX:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circularity          = 4.0 * math.pi * area / (perim * perim)
        (cx_px, cy_px), rad  = cv2.minEnclosingCircle(cnt)

        if circularity >= OBJ_CIRCULARITY_THRESH:
            if circularity > best_circ_val:
                best_circ_val = circularity
                best_circular = (cx_px, cy_px, rad, True)
        else:
            if area > best_fallback_area:
                best_fallback_area = area
                best_fallback      = (cx_px, cy_px, rad, False)

    result = best_circular if best_circular is not None else best_fallback
    if result is None:
        return "NODET", False

    cx_px, cy_px, radius, is_circular = result
    score  = 1.0 if is_circular else 0.5
    msg    = (f"DET,{cx_px/w:.4f},{cy_px/h:.4f},"
              f"{2*radius/w:.4f},{2*radius/h:.4f},{score:.2f}")

    m        = OBJ_FRAME_MARGIN_PX
    fully_in = (
        cx_px - radius >= m and cx_px + radius <= w - m and
        cy_px - radius >= m and cy_px + radius <= h - m
    )
    locked = is_circular and fully_in
    return msg, locked


def object_thread(buf: FrameBuffer, sock: VisionSocket, stop: threading.Event):
    period             = 1.0 / OBJECT_RATE_HZ
    target_locked_sent = False

    while not stop.is_set():
        t0    = time.monotonic()
        frame = buf.get()
        msg, locked = _run_object_detection(frame)   # GIL released inside OpenCV
        sock.send(msg)

        if locked and not target_locked_sent:
            print("[object] full blue circle confirmed – sending LOCKED")
            target_locked_sent = True
            sock.send("LOCKED")

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
