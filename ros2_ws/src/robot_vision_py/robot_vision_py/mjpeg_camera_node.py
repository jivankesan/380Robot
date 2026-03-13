#!/usr/bin/env python3
"""
MJPEG camera node — reads an MJPEG HTTP stream and publishes as ROS Image.

Bridges the native Pi Camera MJPEG server (scripts/camera_mjpeg_server.py)
into ROS 2 as /camera/image_raw, so the rest of the pipeline works unchanged.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class MjpegCameraNode(Node):
    def __init__(self):
        super().__init__("mjpeg_camera_node")

        self.declare_parameter("stream_url", "http://localhost:8081/stream")
        self.declare_parameter("frame_id", "camera_link")

        self.stream_url = self.get_parameter("stream_url").value
        self.frame_id = self.get_parameter("frame_id").value

        self.bridge = CvBridge()

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Image, "/camera/image_raw", sensor_qos)

        self.cap = None
        self.timer = self.create_timer(0.001, self.grab_frame)
        self.get_logger().info(f"Connecting to MJPEG stream: {self.stream_url}")

    def _open_stream(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.stream_url)
        if not self.cap.isOpened():
            self.get_logger().warn("Could not open MJPEG stream, retrying...")
            self.cap = None
            return False
        self.get_logger().info("MJPEG stream opened")
        return True

    def grab_frame(self):
        if self.cap is None:
            self._open_stream()
            return

        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Stream read failed, reconnecting...")
            self.cap = None
            return

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MjpegCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
