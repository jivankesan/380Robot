#!/usr/bin/env python3
"""
Live camera viewer + line-follow debug (no ROS).

Shows:
  - Camera feed with ROI, mask overlay, centroid, bounding box
  - Estimated line errors (lateral + heading)
  - Controller output (v, omega) and turn direction (LEFT/RIGHT/STRAIGHT)

Uses the same parameters from:
  ros2_ws/src/robot_bringup/config/vision.yaml
  ros2_ws/src/robot_bringup/config/control.yaml

Controls (windowed mode only):
  q / ESC : quit
  m       : toggle mask window
  p       : pause/unpause

Headless mode (--headless):
  Saves debug frames to /tmp/line_debug.jpg every second.
  Pull with: scp robot@<ip>:/tmp/line_debug.jpg .
  Detection results are printed to stdout continuously.
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

try:
    import yaml  # type: ignore
except Exception:
    yaml = None

try:
    from picamera2 import Picamera2  # type: ignore
    HAS_PICAMERA2 = True
except Exception:
    HAS_PICAMERA2 = False


@dataclass
class VisionParams:
    roi_y_start: float = 0.57
    roi_y_end: float = 0.85
    roi_x_start: float = 0.0
    roi_x_end: float = 1.0
    line_h_min: int = 0
    line_h_max: int = 10
    line_s_min: int = 100
    line_s_max: int = 255
    line_v_min: int = 100
    line_v_max: int = 255
    line_h2_min: int = 160
    line_h2_max: int = 180
    min_mask_area_px: int = 500
    center_x_offset_px: int = 0
    pixels_to_m_scale: float = 0.001


@dataclass
class ControlParams:
    control_rate_hz: float = 50.0
    kp_lateral: float = 4.0
    kd_lateral: float = 0.5
    kp_heading: float = 3.0
    kd_heading: float = 0.2
    base_speed_mps: float = 0.35
    max_lin_vel_mps: float = 0.45
    min_lin_vel_mps: float = 0.05
    max_ang_vel_rps: float = 1.6
    lost_line_timeout_s: float = 2.0
    reacquire_lin_vel_mps: float = 0.06
    reacquire_ang_vel_rps: float = 1.4
    reacquire_heading_deadband_rad: float = 0.05
    reacquire_lateral_deadband_m: float = 0.01
    turn_enter_heading_rad: float = 0.28
    turn_exit_heading_rad: float = 0.2
    turn_exit_confirm_s: float = 0.12
    turn_max_time_s: float = 1.5
    turn_lin_vel_mps: float = 0.10
    turn_omega_rps: float = 1.2


def _load_ros_params(yaml_path: str, node_key: str) -> Dict[str, object]:
    if yaml is None:
        return {}
    if not os.path.exists(yaml_path):
        return {}
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    node = data.get(node_key, {})
    return node.get("ros__parameters", {}) or {}


def load_params(vision_yaml: str, control_yaml: str) -> Tuple[VisionParams, ControlParams]:
    vp = VisionParams()
    cp = ControlParams()

    v_overrides = _load_ros_params(vision_yaml, "line_detector_node")
    c_overrides = _load_ros_params(control_yaml, "line_follow_controller_node")

    for k, v in v_overrides.items():
        if hasattr(vp, k):
            setattr(vp, k, v)
    for k, v in c_overrides.items():
        if hasattr(cp, k):
            setattr(cp, k, v)

    return vp, cp


@dataclass
class LineObservation:
    valid: bool
    lateral_error_m: float
    heading_error_rad: float
    curvature_1pm: float


@dataclass
class DetectionDebug:
    mask: np.ndarray
    roi_rect: Tuple[int, int, int, int]  # x1, y1, x2, y2
    centroid: Optional[Tuple[int, int]]
    bbox: Optional[Tuple[int, int, int, int]]
    area: float


def detect_line(frame: np.ndarray, p: VisionParams) -> Tuple[LineObservation, DetectionDebug]:
    h, w = frame.shape[:2]
    y1 = int(h * p.roi_y_start)
    y2 = int(h * p.roi_y_end)
    x1 = int(w * p.roi_x_start)
    x2 = int(w * p.roi_x_end)
    roi = frame[y1:y2, x1:x2]
    roi_h, roi_w = roi.shape[:2]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower1 = np.array([p.line_h_min, p.line_s_min, p.line_v_min])
    upper1 = np.array([p.line_h_max, p.line_s_max, p.line_v_max])
    lower2 = np.array([p.line_h2_min, p.line_s_min, p.line_v_min])
    upper2 = np.array([p.line_h2_max, p.line_s_max, p.line_v_max])

    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower1, upper1),
        cv2.inRange(hsv, lower2, upper2),
    )

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    M = cv2.moments(mask)
    area = M["m00"]

    obs = LineObservation(False, 0.0, 0.0, 0.0)
    centroid = None
    bbox = None

    if area >= p.min_mask_area_px:
        cx = M["m10"] / area
        cy = M["m01"] / area
        centroid = (int(cx) + x1, int(cy) + y1)

        image_center_x = roi_w / 2.0 + p.center_x_offset_px
        lateral_error_px = cx - image_center_x
        lateral_error_m = lateral_error_px * p.pixels_to_m_scale

        mid_y = roi_h // 2
        top_mask = mask[:mid_y, :]
        bot_mask = mask[mid_y:, :]
        M_top = cv2.moments(top_mask)
        M_bot = cv2.moments(bot_mask)
        heading = 0.0
        if M_top["m00"] > 0 and M_bot["m00"] > 0:
            cx_top = M_top["m10"] / M_top["m00"]
            cx_bot = M_bot["m10"] / M_bot["m00"]
            heading = float(np.arctan2(cx_top - cx_bot, float(mid_y)))

        obs = LineObservation(True, float(lateral_error_m), float(heading), 0.0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, bw, bh = cv2.boundingRect(c)
            bbox = (x + x1, y + y1, bw, bh)

    dbg = DetectionDebug(mask=mask, roi_rect=(x1, y1, x2, y2),
                         centroid=centroid, bbox=bbox, area=area)
    return obs, dbg


class LineController:
    def __init__(self, p: ControlParams):
        self.p = p
        self.last_lateral_error = 0.0
        self.last_heading_error = 0.0
        self.last_time = None  # type: Optional[float]

        self.line_valid = False
        self.last_valid_time = time.monotonic()
        self.last_valid_obs = LineObservation(False, 0.0, 0.0, 0.0)
        self.last_turn_dir = 0
        self.ever_valid = False
        self.in_reacquire = False
        self.reacquire_dir = 0
        self.last_heading_sign = 0
        self.last_lateral_sign = 0

        self.in_turn_mode = False
        self.turn_dir = 0
        self.turn_start_time = time.monotonic()
        self.turn_exit_candidate = False
        self.turn_exit_candidate_start = time.monotonic()

    def update(self, obs: LineObservation) -> Tuple[float, float, str, str]:
        now = time.monotonic()

        self.line_valid = obs.valid
        if self.line_valid:
            self.last_valid_time = now
            self.last_valid_obs = obs
            self.ever_valid = True
            self.in_reacquire = False

            if abs(obs.heading_error_rad) > self.p.reacquire_heading_deadband_rad:
                self.last_heading_sign = 1 if obs.heading_error_rad > 0 else -1
            if abs(obs.lateral_error_m) > self.p.reacquire_lateral_deadband_m:
                self.last_lateral_sign = 1 if obs.lateral_error_m > 0 else -1

        time_since_valid = now - self.last_valid_time

        if not self.line_valid:
            if self.in_turn_mode:
                time_in_turn = now - self.turn_start_time
                if time_in_turn <= self.p.turn_max_time_s:
                    omega = float(self.turn_dir) * self.p.turn_omega_rps
                    omega = float(np.clip(omega, -self.p.max_ang_vel_rps, self.p.max_ang_vel_rps))
                    v = float(np.clip(self.p.turn_lin_vel_mps, 0.0, self.p.max_lin_vel_mps))
                    return v, omega, "TURN", "turn_mode"
                self.in_turn_mode = False

            if (not self.ever_valid) or (time_since_valid > self.p.lost_line_timeout_s):
                return 0.0, 0.0, "STOP", "line_lost"

            if not self.in_reacquire:
                if self.last_heading_sign != 0:
                    self.reacquire_dir = self.last_heading_sign
                elif self.last_lateral_sign != 0:
                    self.reacquire_dir = self.last_lateral_sign
                elif self.last_turn_dir != 0:
                    self.reacquire_dir = self.last_turn_dir
                else:
                    self.reacquire_dir = 1
                self.in_reacquire = True

            omega = float(self.reacquire_dir) * self.p.reacquire_ang_vel_rps
            omega = float(np.clip(omega, -self.p.max_ang_vel_rps, self.p.max_ang_vel_rps))
            v = float(np.clip(self.p.reacquire_lin_vel_mps, 0.0, self.p.max_lin_vel_mps))
            return v, omega, "REACQUIRE", "reacquire"

        # Line valid: compute control
        lateral = obs.lateral_error_m
        heading = obs.heading_error_rad

        if not self.in_turn_mode and abs(heading) > self.p.turn_enter_heading_rad:
            self.in_turn_mode = True
            self.turn_dir = 1 if heading >= 0.0 else -1
            self.turn_start_time = now
            self.turn_exit_candidate = False

        if self.in_turn_mode:
            time_in_turn = now - self.turn_start_time
            if time_in_turn > self.p.turn_max_time_s:
                self.in_turn_mode = False
            elif abs(heading) < self.p.turn_exit_heading_rad:
                if not self.turn_exit_candidate:
                    self.turn_exit_candidate = True
                    self.turn_exit_candidate_start = now
                else:
                    if (now - self.turn_exit_candidate_start) > self.p.turn_exit_confirm_s:
                        self.in_turn_mode = False
            else:
                self.turn_exit_candidate = False

        if self.last_time is None:
            dt = 1.0 / self.p.control_rate_hz
        else:
            dt = max(1e-3, now - self.last_time)

        d_lateral = (lateral - self.last_lateral_error) / dt
        d_heading = (heading - self.last_heading_error) / dt

        omega = (
            self.p.kp_lateral * lateral
            + self.p.kd_lateral * d_lateral
            + self.p.kp_heading * heading
            + self.p.kd_heading * d_heading
        )

        if self.in_turn_mode:
            if omega * float(self.turn_dir) < 0.0:
                omega = float(self.turn_dir) * abs(omega)
            if abs(omega) < self.p.turn_omega_rps:
                omega = float(self.turn_dir) * self.p.turn_omega_rps

        omega = float(np.clip(omega, -self.p.max_ang_vel_rps, self.p.max_ang_vel_rps))
        v = float(np.clip(self.p.base_speed_mps, self.p.min_lin_vel_mps, self.p.max_lin_vel_mps))
        if self.in_turn_mode:
            v = min(v, self.p.turn_lin_vel_mps)

        self.last_lateral_error = lateral
        self.last_heading_error = heading
        self.last_time = now
        if abs(omega) > 1e-3:
            self.last_turn_dir = 1 if omega > 0 else -1

        action = "LEFT" if omega > 0.05 else ("RIGHT" if omega < -0.05 else "STRAIGHT")
        mode = "turn_mode" if self.in_turn_mode else "track"
        return v, omega, action, mode


def draw_overlay(frame: np.ndarray, obs: LineObservation, dbg: DetectionDebug,
                 v: float, omega: float, action: str, mode: str) -> np.ndarray:
    out = frame.copy()
    x1, y1, x2, y2 = dbg.roi_rect

    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if obs.valid:
        if dbg.centroid:
            cv2.circle(out, dbg.centroid, 8, (255, 0, 0), -1)
        if dbg.bbox:
            x, y, bw, bh = dbg.bbox
            cv2.rectangle(out, (x, y), (x + bw, y + bh), (255, 0, 255), 2)

        # reference center line
        roi_w = x2 - x1
        ref_x = int(roi_w / 2.0) + x1
        cv2.line(out, (ref_x, y1), (ref_x, y2), (255, 255, 0), 2)

    status = "VALID" if obs.valid else "LOST"
    cv2.putText(out, f"Line: {status}  Area: {int(dbg.area)}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(out, f"Lat: {obs.lateral_error_m:.3f}m  Hdg: {np.degrees(obs.heading_error_rad):.1f}deg",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(out, f"CMD: v={v:.2f} m/s  omega={omega:.2f} r/s  {action}  [{mode}]",
                (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--video", type=str, default="", help="Video file instead of camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--vision-config", type=str,
                        default="ros2_ws/src/robot_bringup/config/vision.yaml")
    parser.add_argument("--control-config", type=str,
                        default="ros2_ws/src/robot_bringup/config/control.yaml")
    parser.add_argument("--no-mask", action="store_true", help="Disable mask window")
    parser.add_argument("--picamera", action="store_true",
                        help="Use Raspberry Pi Camera via picamera2 instead of OpenCV")
    parser.add_argument("--headless", action="store_true",
                        help="No display: print results to stdout and save debug frame to /tmp/line_debug.jpg")
    parser.add_argument("--save-interval", type=float, default=1.0,
                        help="Seconds between saved frames in headless mode (default: 1.0)")
    args = parser.parse_args()

    vp, cp = load_params(args.vision_config, args.control_config)
    vp.roi_x_start = 0.0
    vp.roi_x_end = 1.0
    controller = LineController(cp)

    # --- Camera setup ---
    picam = None
    cap = None

    if args.picamera:
        if not HAS_PICAMERA2:
            print("picamera2 not installed. Run: pip install picamera2")
            return 1
        picam = Picamera2()
        config = picam.create_preview_configuration(
            main={"size": (args.width, args.height), "format": "BGR888"}
        )
        picam.configure(config)
        picam.start()
        print(f"Pi Camera started at {args.width}x{args.height}")
    elif args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not cap.isOpened():
            print("Failed to open camera/video.")
            return 1

    show_mask = not args.no_mask
    paused = False
    last_save_time = time.monotonic()
    frame_count = 0

    try:
        while True:
            if not paused:
                # Grab frame
                if picam is not None:
                    frame = picam.capture_array()
                else:
                    ok, frame = cap.read()
                    if not ok:
                        break

                obs, dbg = detect_line(frame, vp)
                v, omega, action, mode = controller.update(obs)
                overlay = draw_overlay(frame, obs, dbg, v, omega, action, mode)
                frame_count += 1

                if args.headless:
                    # Print stats every frame
                    status = "VALID" if obs.valid else "LOST"
                    print(
                        f"[{frame_count:05d}] Line:{status}  "
                        f"Lat:{obs.lateral_error_m:+.3f}m  "
                        f"Hdg:{np.degrees(obs.heading_error_rad):+.1f}deg  "
                        f"v:{v:.2f}m/s  omega:{omega:+.2f}r/s  {action}  [{mode}]"
                    )
                    # Save debug image periodically
                    now = time.monotonic()
                    if now - last_save_time >= args.save_interval:
                        cv2.imwrite("/tmp/line_debug.jpg", overlay)
                        cv2.imwrite("/tmp/line_mask.jpg", dbg.mask)
                        print(f"  -> saved /tmp/line_debug.jpg and /tmp/line_mask.jpg")
                        last_save_time = now
                else:
                    cv2.imshow("line_follow_view", overlay)
                    if show_mask:
                        cv2.imshow("line_mask", dbg.mask)

            if args.headless:
                # No waitKey needed — small sleep to avoid 100% CPU
                time.sleep(0.01)
            else:
                if paused:
                    key = cv2.waitKey(30) & 0xFF
                    if key in (ord("p"), ord(" ")):
                        paused = False
                    elif key in (ord("q"), 27):
                        break
                    continue

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("m"):
                    show_mask = not show_mask
                    if not show_mask:
                        cv2.destroyWindow("line_mask")
                if key in (ord("p"), ord(" ")):
                    paused = True

    except KeyboardInterrupt:
        pass
    finally:
        if picam is not None:
            picam.stop()
        if cap is not None:
            cap.release()
        if not args.headless:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
