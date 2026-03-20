#!/usr/bin/env python3
"""Object detector node – detects a blue circle target.

Publishes to /vision/detections whenever a blue circle is visible.
When the circle is FULLY inside the frame, publishes True to
/vision/target_locked and goes dormant until the FSM signals
RETURN_FOLLOW_LINE (post-pickup), at which point detection resumes.

Hard-stop wiring: FSM should subscribe to /vision/target_locked and
immediately transition to PICKUP (bypassing the stability counter)
when it receives True.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from robot_interfaces.msg import Detection2D, Detections2D


class ObjectDetectorNode(Node):
    """Detects a blue circle in the full camera frame."""

    def __init__(self):
        super().__init__('object_detector_node')

        # Blue HSV range (tunable via params)
        self.declare_parameter('h_min', 100)
        self.declare_parameter('h_max', 130)
        self.declare_parameter('s_min', 150)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)
        # Minimum contour area in pixels to reject noise
        self.declare_parameter('min_area_px', 500)
        # 4π·area/perimeter² – 1.0 is a perfect circle
        self.declare_parameter('circularity_threshold', 0.75)
        # Pixels the circle edge must be inside the frame border
        # to count as "fully visible"
        self.declare_parameter('frame_margin_px', 5)
        self.declare_parameter('detection_rate_hz', 15.0)

        self.h_min = self.get_parameter('h_min').value
        self.h_max = self.get_parameter('h_max').value
        self.s_min = self.get_parameter('s_min').value
        self.s_max = self.get_parameter('s_max').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.min_area_px = self.get_parameter('min_area_px').value
        self.circularity_thresh = self.get_parameter('circularity_threshold').value
        self.frame_margin = self.get_parameter('frame_margin_px').value
        detection_rate = self.get_parameter('detection_rate_hz').value

        self.bridge = CvBridge()

        # Goes dormant when FSM enters PICKUP (no point detecting while claw
        # is closing). Resets when FSM enters RETURN_FOLLOW_LINE.
        self.target_acquired = False

        # One-shot flag so target_locked is only published once per approach,
        # not on every frame after the full circle is first confirmed.
        self.target_locked_sent = False

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sensor_qos)

        # Normal detection stream – FSM uses this for state transitions
        self.det_pub = self.create_publisher(Detections2D, '/vision/detections', 10)

        # Hard-stop signal – FSM should subscribe and immediately enter PICKUP
        self.locked_pub = self.create_publisher(Bool, '/vision/target_locked', 10)

        # Watch FSM state so we can resume after the pickup-and-return leg
        self.fsm_sub = self.create_subscription(
            String, '/control/fsm_state', self._fsm_state_cb, 10)

        self.last_proc_time = self.get_clock().now()
        self.detection_period = 1.0 / detection_rate

        self.get_logger().info(
            f'Blue circle detector ready '
            f'(H={self.h_min}-{self.h_max}, '
            f'circularity>={self.circularity_thresh})'
        )

    # ------------------------------------------------------------------
    def _fsm_state_cb(self, msg: String) -> None:
        if msg.data == 'PICKUP' and not self.target_acquired:
            # Approach is done — go dormant while claw closes
            self.target_acquired = True
            self.get_logger().info('Detection dormant during pickup')
        elif msg.data == 'RETURN_FOLLOW_LINE' and self.target_acquired:
            # Back on the line — resume detecting and allow target_locked again
            self.target_acquired = False
            self.target_locked_sent = False
            self.get_logger().info('Detection re-enabled for return leg')

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        if self.target_acquired:
            return

        now = self.get_clock().now()
        if (now - self.last_proc_time).nanoseconds / 1e9 < self.detection_period:
            return
        self.last_proc_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        height, width = frame.shape[:2]

        det_msg = Detections2D()
        det_msg.stamp = msg.header.stamp
        det_msg.frame_id = msg.header.frame_id

        circle = self._detect_blue_circle(frame)

        if circle is not None:
            cx_px, cy_px, radius = circle

            det = Detection2D()
            det.class_name = 'blue_circle'
            det.score = 1.0
            det.cx = float(cx_px / width)
            det.cy = float(cy_px / height)
            det.w = float(2.0 * radius / width)
            det.h = float(2.0 * radius / height)
            det_msg.detections = [det]

            m = self.frame_margin
            fully_in_frame = (
                cx_px - radius >= m and
                cx_px + radius <= width - m and
                cy_px - radius >= m and
                cy_px + radius <= height - m
            )

            if fully_in_frame and not self.target_locked_sent:
                self.get_logger().info(
                    f'Full blue circle confirmed '
                    f'(cx={cx_px:.0f} cy={cy_px:.0f} r={radius:.0f}px) — locking'
                )
                self.target_locked_sent = True
                self.locked_pub.publish(Bool(data=True))

        self.det_pub.publish(det_msg)

    # ------------------------------------------------------------------
    def _detect_blue_circle(self, image: np.ndarray):
        """Return (cx, cy, radius) of the most circular blue blob, or None."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(
            hsv,
            np.array([self.h_min, self.s_min, self.v_min]),
            np.array([self.h_max, self.s_max, self.v_max]),
        )

        # Elliptical kernel cleans up blobs more naturally than a square
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_circularity = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area_px:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.circularity_thresh:
                continue

            if circularity > best_circularity:
                best_circularity = circularity
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                best = (cx, cy, radius)

        return best


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
