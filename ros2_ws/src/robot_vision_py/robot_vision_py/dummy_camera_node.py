#!/usr/bin/env python3
"""Dummy camera node for development without hardware.

Publishes images from a video file, image file, or generates test patterns.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DummyCameraNode(Node):
    """ROS 2 node for simulated camera input."""

    def __init__(self):
        super().__init__('dummy_camera_node')

        # Declare parameters
        self.declare_parameter('video_file', '')
        self.declare_parameter('image_file', '')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        # Get parameters
        self.video_file = self.get_parameter('video_file').value
        self.image_file = self.get_parameter('image_file').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.loop = self.get_parameter('loop').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # CV Bridge
        self.bridge = CvBridge()

        # Video capture or static image
        self.cap = None
        self.static_image = None
        self.frame_count = 0

        if self.video_file:
            self.cap = cv2.VideoCapture(self.video_file)
            if not self.cap.isOpened():
                self.get_logger().error(
                    f'Failed to open video file: {self.video_file}'
                )
                self.cap = None
            else:
                self.get_logger().info(f'Playing video: {self.video_file}')
        elif self.image_file:
            self.static_image = cv2.imread(self.image_file)
            if self.static_image is None:
                self.get_logger().error(
                    f'Failed to load image: {self.image_file}'
                )
            else:
                self.get_logger().info(f'Using static image: {self.image_file}')
        else:
            self.get_logger().info('Generating test pattern images')

        # Publisher
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)

        # Timer for publishing
        period = 1.0 / self.frame_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Dummy camera node initialized at {self.frame_rate} Hz'
        )

    def timer_callback(self):
        """Publish next frame."""
        frame = None

        if self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                if not ret:
                    return
        elif self.static_image is not None:
            frame = self.static_image.copy()
        else:
            # Generate test pattern with line
            frame = self._generate_test_pattern()

        if frame is None:
            return

        # Resize if needed
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        # Convert and publish
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.image_pub.publish(msg)

        self.frame_count += 1

    def _generate_test_pattern(self) -> np.ndarray:
        """Generate a test pattern image with a line."""
        # White background
        img = np.ones((self.height, self.width, 3), dtype=np.uint8) * 220

        # Draw a curved black line
        center_x = self.width // 2
        amplitude = 50
        frequency = 0.01
        phase = self.frame_count * 0.1

        points = []
        for y in range(int(self.height * 0.3), self.height):
            x = int(
                center_x +
                amplitude * np.sin(frequency * y + phase) +
                20 * np.sin(frequency * 3 * y)
            )
            points.append((x, y))

        # Draw line
        for i in range(len(points) - 1):
            cv2.line(img, points[i], points[i + 1], (30, 30, 30), 20)

        # Add a colored target object occasionally
        if (self.frame_count // 100) % 3 == 0:
            obj_x = self.width // 2 + int(30 * np.sin(self.frame_count * 0.05))
            obj_y = int(self.height * 0.3)
            cv2.rectangle(
                img,
                (obj_x - 20, obj_y - 30),
                (obj_x + 20, obj_y + 30),
                (0, 200, 255),  # Yellow-orange for LEGO person
                -1
            )

        return img

    def destroy_node(self):
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
