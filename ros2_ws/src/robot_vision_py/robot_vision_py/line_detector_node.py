#!/usr/bin/env python3
"""Line detector node for 380Robot.

Detects a line on the floor using camera input and publishes
lateral/heading error for the controller.
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
    """ROS 2 node for line detection."""

    def __init__(self):
        super().__init__('line_detector_node')

        # Declare parameters
        self.declare_parameter('roi_y_start', 0.5)
        self.declare_parameter('roi_y_end', 1.0)
        self.declare_parameter('roi_x_start', 0.1)
        self.declare_parameter('roi_x_end', 0.9)
        self.declare_parameter('line_h_min', 0)
        self.declare_parameter('line_h_max', 180)
        self.declare_parameter('line_s_min', 0)
        self.declare_parameter('line_s_max', 50)
        self.declare_parameter('line_v_min', 0)
        self.declare_parameter('line_v_max', 80)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('min_contour_area', 500)
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
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
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

        self.get_logger().info('Line detector node initialized')

    def image_callback(self, msg: Image):
        """Process incoming image and detect line."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Get image dimensions
        height, width = cv_image.shape[:2]

        # Extract ROI
        y_start = int(height * self.roi_y_start)
        y_end = int(height * self.roi_y_end)
        x_start = int(width * self.roi_x_start)
        x_end = int(width * self.roi_x_end)
        roi = cv_image[y_start:y_end, x_start:x_end]

        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Threshold for line (dark line on light floor)
        lower = np.array([self.line_h_min, self.line_s_min, self.line_v_min])
        upper = np.array([self.line_h_max, self.line_s_max, self.line_v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # Morphological operations
        kernel = np.ones(
            (self.morph_kernel_size, self.morph_kernel_size),
            np.uint8
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Create line observation message
        obs = LineObservation()
        obs.stamp = msg.header.stamp
        obs.valid = False
        obs.lateral_error_m = 0.0
        obs.heading_error_rad = 0.0
        obs.curvature_1pm = 0.0

        # Process largest contour
        if contours:
            # Filter by area and get largest
            valid_contours = [
                c for c in contours
                if cv2.contourArea(c) >= self.min_contour_area
            ]

            if valid_contours:
                largest = max(valid_contours, key=cv2.contourArea)

                # Compute centroid
                moments = cv2.moments(largest)
                if moments['m00'] > 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])

                    # ROI dimensions
                    roi_width = x_end - x_start
                    roi_height = y_end - y_start

                    # Lateral error: offset from center (positive = right)
                    center_x = roi_width / 2
                    lateral_error_px = cx - center_x
                    obs.lateral_error_m = float(
                        lateral_error_px * self.pixels_to_m_scale
                    )

                    # Fit line to contour for heading
                    if len(largest) >= 5:
                        [vx, vy, x, y] = cv2.fitLine(
                            largest,
                            cv2.DIST_L2, 0, 0.01, 0.01
                        )
                        # Heading error: angle from vertical
                        # vy should be ~1 for vertical line, angle = atan2(vx, vy)
                        obs.heading_error_rad = float(np.arctan2(vx[0], vy[0]))

                        # Estimate curvature using polynomial fit
                        points = largest.reshape(-1, 2)
                        if len(points) >= 10:
                            # Sort by y for fitting
                            points = points[points[:, 1].argsort()]
                            x_pts = points[:, 0]
                            y_pts = points[:, 1]

                            # Fit second-order polynomial
                            try:
                                coeffs = np.polyfit(y_pts, x_pts, 2)
                                # Curvature at center: 2*a for ax^2 + bx + c
                                obs.curvature_1pm = float(2 * coeffs[0])
                            except np.linalg.LinAlgError:
                                obs.curvature_1pm = 0.0

                    obs.valid = True

                    # Debug visualization
                    if self.publish_debug:
                        debug_img = cv_image.copy()
                        # Draw ROI
                        cv2.rectangle(
                            debug_img,
                            (x_start, y_start),
                            (x_end, y_end),
                            (0, 255, 0), 2
                        )
                        # Draw contour
                        shifted_contour = largest + np.array([x_start, y_start])
                        cv2.drawContours(
                            debug_img, [shifted_contour], -1, (0, 0, 255), 2
                        )
                        # Draw centroid
                        cv2.circle(
                            debug_img,
                            (cx + x_start, cy + y_start),
                            10, (255, 0, 0), -1
                        )
                        # Draw center line
                        cv2.line(
                            debug_img,
                            (width // 2, y_start),
                            (width // 2, y_end),
                            (255, 255, 0), 2
                        )
                        # Add text
                        cv2.putText(
                            debug_img,
                            f'Lat: {obs.lateral_error_m:.3f}m',
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                        )
                        cv2.putText(
                            debug_img,
                            f'Hdg: {np.degrees(obs.heading_error_rad):.1f}deg',
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                        )

                        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
                        debug_msg.header = msg.header
                        self.debug_pub.publish(debug_msg)

        # Publish observation
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
