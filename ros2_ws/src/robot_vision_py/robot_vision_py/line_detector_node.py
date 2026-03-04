#!/usr/bin/env python3
"""Line detector node for 380Robot.

Detects a red line on the floor using camera input and publishes
lateral/heading error for the controller.

Detection pipeline (matches validated standalone script):
  HSV threshold (dual-range for red wrap-around) -> Canny edges -> HoughLinesP
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from robot_interfaces.msg import LineObservation


class LineDetectorNode(Node):
    """ROS 2 node for red line detection."""

    def __init__(self):
        super().__init__('line_detector_node')

        # Declare parameters
        self.declare_parameter('roi_y_start', 0.5)
        self.declare_parameter('roi_y_end', 1.0)
        self.declare_parameter('roi_x_start', 0.1)
        self.declare_parameter('roi_x_end', 0.9)

        # HSV range 1 (lower red: H=0-10)
        self.declare_parameter('line_h_min', 0)
        self.declare_parameter('line_h_max', 10)
        self.declare_parameter('line_s_min', 100)
        self.declare_parameter('line_s_max', 255)
        self.declare_parameter('line_v_min', 100)
        self.declare_parameter('line_v_max', 255)
        # HSV range 2 (upper red: H=160-180)
        self.declare_parameter('line_h2_min', 160)
        self.declare_parameter('line_h2_max', 180)

        # Canny parameters
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)

        # HoughLinesP parameters
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_line_length', 50)
        self.declare_parameter('hough_max_line_gap', 10)

        # Scale: pixels to meters for lateral error
        self.declare_parameter('pixels_to_m_scale', 0.001)
        self.declare_parameter('publish_debug_image', True)

        # Get parameters
        self.roi_y_start = self.get_parameter('roi_y_start').value
        self.roi_y_end = self.get_parameter('roi_y_end').value
        self.roi_x_start = self.get_parameter('roi_x_start').value
        self.roi_x_end = self.get_parameter('roi_x_end').value
        self.line_h_min = self.get_parameter('line_h_min').value
        self.line_h_max = self.get_parameter('line_h_max').value
        self.line_s_min = self.get_parameter('line_s_min').value
        self.line_s_max = self.get_parameter('line_s_max').value
        self.line_v_min = self.get_parameter('line_v_min').value
        self.line_v_max = self.get_parameter('line_v_max').value
        self.line_h2_min = self.get_parameter('line_h2_min').value
        self.line_h2_max = self.get_parameter('line_h2_max').value
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.hough_min_line_length = self.get_parameter('hough_min_line_length').value
        self.hough_max_line_gap = self.get_parameter('hough_max_line_gap').value
        self.pixels_to_m_scale = self.get_parameter('pixels_to_m_scale').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # CV Bridge
        self.bridge = CvBridge()

        # QoS for camera
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos
        )

        # Publishers
        self.line_pub = self.create_publisher(
            LineObservation,
            '/vision/line_observation',
            10
        )

        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image,
                '/vision/debug_image',
                10
            )

        self.get_logger().info('Line detector node initialized (Canny+Hough pipeline)')

    def image_callback(self, msg: Image):
        """Process incoming image and detect red line."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = cv_image.shape[:2]

        # Extract ROI (lower half of image by default)
        y_start = int(height * self.roi_y_start)
        y_end = int(height * self.roi_y_end)
        x_start = int(width * self.roi_x_start)
        x_end = int(width * self.roi_x_end)
        roi = cv_image[y_start:y_end, x_start:x_end]
        roi_width = x_end - x_start

        # HSV masking - dual range for red wrap-around
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower1 = np.array([self.line_h_min, self.line_s_min, self.line_v_min])
        upper1 = np.array([self.line_h_max, self.line_s_max, self.line_v_max])
        lower2 = np.array([self.line_h2_min, self.line_s_min, self.line_v_min])
        upper2 = np.array([self.line_h2_max, self.line_s_max, self.line_v_max])
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower1, upper1),
            cv2.inRange(hsv, lower2, upper2)
        )

        # Isolate red regions, convert to gray, run Canny
        red_only = cv2.bitwise_and(roi, roi, mask=mask)
        gray = cv2.cvtColor(red_only, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)

        # Hough line detection
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

        # Build observation message
        obs = LineObservation()
        obs.stamp = msg.header.stamp
        obs.valid = False
        obs.lateral_error_m = 0.0
        obs.heading_error_rad = 0.0
        obs.curvature_1pm = 0.0

        if lines is not None:
            # Lateral error: average midpoint x of all lines vs ROI centre
            mid_xs = [(l[0][0] + l[0][2]) / 2.0 for l in lines]
            avg_mid_x = float(np.mean(mid_xs))
            lateral_error_px = avg_mid_x - (roi_width / 2.0)
            obs.lateral_error_m = lateral_error_px * self.pixels_to_m_scale

            # Heading error: average angle of lines from vertical
            angles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = float(x2 - x1)
                dy = float(y2 - y1)
                if abs(dy) > 1e-3:
                    angles.append(np.arctan2(dx, abs(dy)))
            if angles:
                obs.heading_error_rad = float(np.mean(angles))

            obs.valid = True

            # Debug visualization
            if self.publish_debug:
                debug_img = cv_image.copy()
                # ROI boundary
                cv2.rectangle(
                    debug_img,
                    (x_start, y_start), (x_end, y_end),
                    (0, 255, 0), 2
                )
                # Detected Hough lines (shifted back to full-image coords)
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(
                        debug_img,
                        (x1 + x_start, y1 + y_start),
                        (x2 + x_start, y2 + y_start),
                        (0, 255, 0), 2
                    )
                # Average midpoint
                avg_x_full = int(avg_mid_x) + x_start
                avg_y_full = (y_start + y_end) // 2
                cv2.circle(debug_img, (avg_x_full, avg_y_full), 10, (255, 0, 0), -1)
                # Centre reference line
                cv2.line(
                    debug_img,
                    (width // 2, y_start), (width // 2, y_end),
                    (255, 255, 0), 2
                )
                cv2.putText(
                    debug_img,
                    f'Lat: {obs.lateral_error_m:.3f}m',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                cv2.putText(
                    debug_img,
                    f'Hdg: {np.degrees(obs.heading_error_rad):.1f}deg',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)

        self.line_pub.publish(obs)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
