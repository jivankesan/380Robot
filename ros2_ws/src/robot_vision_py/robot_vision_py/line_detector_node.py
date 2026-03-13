#!/usr/bin/env python3
"""Line detector node for 380Robot.

Detects a red line on the floor using camera input and publishes
lateral/heading error for the controller.

Detection pipeline:
  HSV threshold (dual-range for red wrap-around) -> morphological cleanup
  -> centroid for lateral error -> two-zone centroid for heading error
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
        self.declare_parameter('roi_y_start', 0.62)
        self.declare_parameter('roi_y_end', 0.85)
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

        # Minimum mask area to consider a valid detection (px^2)
        self.declare_parameter('min_mask_area_px', 500)

        # Camera mounting offset: shift the image centre used for lateral error
        # (positive = camera is mounted left of robot centre, shifts reference right)
        self.declare_parameter('center_x_offset_px', 0)

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
        self.min_mask_area = self.get_parameter('min_mask_area_px').value
        self.center_x_offset = self.get_parameter('center_x_offset_px').value
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

        self.get_logger().info('Line detector node initialized (centroid pipeline)')

    def image_callback(self, msg: Image):
        """Process incoming image and detect red line."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = cv_image.shape[:2]

        # Extract ROI
        y_start = int(height * self.roi_y_start)
        y_end = int(height * self.roi_y_end)
        x_start = int(width * self.roi_x_start)
        x_end = int(width * self.roi_x_end)
        roi = cv_image[y_start:y_end, x_start:x_end]
        roi_h, roi_w = roi.shape[:2]

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

        # Morphological cleanup: close gaps in the line, remove speckles
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Build observation message
        obs = LineObservation()
        obs.stamp = msg.header.stamp
        obs.valid = False
        obs.lateral_error_m = 0.0
        obs.heading_error_rad = 0.0
        obs.curvature_1pm = 0.0

        M = cv2.moments(mask)
        total_area = M['m00']

        if total_area >= self.min_mask_area:
            # --- Lateral error: centroid x vs adjusted image centre ---
            cx = M['m10'] / total_area
            # center_x_offset compensates for camera not being centred on robot
            image_center_x = roi_w / 2.0 + self.center_x_offset
            lateral_error_px = cx - image_center_x
            obs.lateral_error_m = lateral_error_px * self.pixels_to_m_scale

            # --- Heading error: compare centroids in top vs bottom half of ROI ---
            mid_y = roi_h // 2
            top_mask = mask[:mid_y, :]
            bot_mask = mask[mid_y:, :]
            M_top = cv2.moments(top_mask)
            M_bot = cv2.moments(bot_mask)

            if M_top['m00'] > 0 and M_bot['m00'] > 0:
                cx_top = M_top['m10'] / M_top['m00']
                cx_bot = M_bot['m10'] / M_bot['m00']
                # Positive heading = line tilts right (robot needs to turn left)
                obs.heading_error_rad = float(np.arctan2(cx_top - cx_bot, float(mid_y)))

            obs.valid = True

            # Debug visualization
            if self.publish_debug:
                debug_img = cv_image.copy()
                # Show mask as overlay
                mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                mask_colored[:, :, 0] = 0  # zero out B and G, keep R-ish
                mask_colored[:, :, 1] = 0
                roi_region = debug_img[y_start:y_end, x_start:x_end]
                cv2.addWeighted(roi_region, 0.6, mask_colored, 0.4, 0, roi_region)
                # ROI boundary
                cv2.rectangle(
                    debug_img,
                    (x_start, y_start), (x_end, y_end),
                    (0, 255, 0), 2
                )
                # Centroid dot
                cx_full = int(cx) + x_start
                cy_full = int(M['m01'] / total_area) + y_start
                cv2.circle(debug_img, (cx_full, cy_full), 10, (255, 0, 0), -1)
                # Adjusted centre reference line
                ref_x = int(image_center_x) + x_start
                cv2.line(debug_img, (ref_x, y_start), (ref_x, y_end), (255, 255, 0), 2)
                cv2.putText(
                    debug_img,
                    f'Lat: {obs.lateral_error_m:.3f}m  Hdg: {np.degrees(obs.heading_error_rad):.1f}deg',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
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
