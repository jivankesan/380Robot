#!/usr/bin/env python3
"""Object detector node for 380Robot.

Detects target objects (blue circle target) in camera images
using either heuristic color + shape detection or YOLO.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from robot_interfaces.msg import Detection2D, Detections2D


class ObjectDetectorNode(Node):
    """ROS 2 node for object detection."""

    def __init__(self):
        super().__init__('object_detector_node')

        # Declare parameters
        self.declare_parameter('detector_type', 'heuristic')
        self.declare_parameter('target_class', 'blue_circle')
        self.declare_parameter('confidence_threshold', 0.5)
        # Blue circle HSV parameters
        self.declare_parameter('heuristic_h_min', 90)
        self.declare_parameter('heuristic_h_max', 130)
        self.declare_parameter('heuristic_s_min', 100)
        self.declare_parameter('heuristic_s_max', 255)
        self.declare_parameter('heuristic_v_min', 50)
        self.declare_parameter('heuristic_v_max', 255)
        self.declare_parameter('heuristic_min_area', 2000)
        self.declare_parameter('heuristic_max_area', 200000)
        self.declare_parameter('heuristic_circularity_min', 0.4)
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('detection_rate_hz', 10.0)

        # Get parameters
        self.detector_type = self.get_parameter('detector_type').value
        self.target_class = self.get_parameter('target_class').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.h_min = self.get_parameter('heuristic_h_min').value
        self.h_max = self.get_parameter('heuristic_h_max').value
        self.s_min = self.get_parameter('heuristic_s_min').value
        self.s_max = self.get_parameter('heuristic_s_max').value
        self.v_min = self.get_parameter('heuristic_v_min').value
        self.v_max = self.get_parameter('heuristic_v_max').value
        self.min_area = self.get_parameter('heuristic_min_area').value
        self.max_area = self.get_parameter('heuristic_max_area').value
        self.circularity_min = self.get_parameter('heuristic_circularity_min').value
        self.model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        detection_rate = self.get_parameter('detection_rate_hz').value

        # CV Bridge
        self.bridge = CvBridge()

        # YOLO model (if using)
        self.model = None
        if self.detector_type == 'yolo' and self.model_path:
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Loaded YOLO model from {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.detector_type = 'heuristic'

        # QoS for camera
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos
        )

        # Publisher
        self.det_pub = self.create_publisher(
            Detections2D,
            '/vision/detections',
            10
        )

        # Rate limiting
        self.last_detection_time = self.get_clock().now()
        self.detection_period = 1.0 / detection_rate

        self.get_logger().info(
            f'Object detector initialized (type: {self.detector_type})'
        )

    def image_callback(self, msg: Image):
        """Process incoming image and detect objects."""
        # Rate limiting
        now = self.get_clock().now()
        elapsed = (now - self.last_detection_time).nanoseconds / 1e9
        if elapsed < self.detection_period:
            return
        self.last_detection_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Create detections message
        det_msg = Detections2D()
        det_msg.stamp = msg.header.stamp
        det_msg.frame_id = msg.header.frame_id

        if self.detector_type == 'yolo' and self.model is not None:
            detections = self._detect_yolo(cv_image)
        else:
            detections = self._detect_heuristic(cv_image)

        det_msg.detections = detections
        self.det_pub.publish(det_msg)

    def _detect_heuristic(self, image: np.ndarray) -> list:
        """Detect blue circle using HSV color + circularity filter."""
        detections = []
        height, width = image.shape[:2]

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold for blue
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # Morphological cleanup with ellipse kernel for circular shapes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        best_det = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue

            # Circularity filter: 4*pi*area / perimeter^2 (1.0 = perfect circle)
            perimeter = cv2.arcLength(contour, True)
            if perimeter < 1.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.circularity_min:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            det = Detection2D()
            det.class_name = self.target_class
            # Confidence: blend circularity and area coverage
            det.score = float(min(1.0, circularity * 0.6 + min(1.0, area / 30000.0) * 0.4))
            det.cx = float((x + w / 2) / width)
            det.cy = float((y + h / 2) / height)
            det.w = float(w / width)
            det.h = float(h / height)

            # Keep the largest (most prominent) blue circle
            if det.score >= self.conf_threshold and area > best_area:
                best_area = area
                best_det = det

        if best_det is not None:
            detections.append(best_det)

        return detections

    def _detect_yolo(self, image: np.ndarray) -> list:
        """Detect objects using YOLO model."""
        detections = []
        height, width = image.shape[:2]

        # Run inference
        results = self.model(image, verbose=False)

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                if conf < self.conf_threshold:
                    continue

                cls_id = int(boxes.cls[i])
                class_name = self.model.names[cls_id]

                # Get box coordinates
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()

                det = Detection2D()
                det.class_name = class_name
                det.score = conf
                det.cx = float((x1 + x2) / 2 / width)
                det.cy = float((y1 + y2) / 2 / height)
                det.w = float((x2 - x1) / width)
                det.h = float((y2 - y1) / height)

                detections.append(det)

        return detections


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
