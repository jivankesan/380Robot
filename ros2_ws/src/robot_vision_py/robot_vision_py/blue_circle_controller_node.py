#!/usr/bin/env python3
"""Blue circle navigation controller for 380Robot.

When the FSM is in APPROACH_TARGET state, this node uses the detected
blue circle position to navigate slowly toward its center.

Control law:
  - Angular: proportional to horizontal offset (cx - 0.5)
  - Linear: constant slow speed; zero when centered and close

The line follow controller is disabled (via /control/enable=false) while
this controller is active, giving it sole control of /control/cmd_vel.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from robot_interfaces.msg import Detections2D


class BlueCircleControllerNode(Node):
    """Proportional controller to navigate toward the blue circle center."""

    def __init__(self):
        super().__init__('blue_circle_controller_node')

        # Parameters
        self.declare_parameter('target_class', 'blue_circle')
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('linear_speed_mps', 0.12)
        self.declare_parameter('max_angular_rps', 1.2)
        self.declare_parameter('center_tolerance_x', 0.12)
        self.declare_parameter('detection_timeout_s', 0.5)
        self.declare_parameter('rate_hz', 20.0)

        self.target_class = self.get_parameter('target_class').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.linear_speed = self.get_parameter('linear_speed_mps').value
        self.max_angular = self.get_parameter('max_angular_rps').value
        self.center_tol_x = self.get_parameter('center_tolerance_x').value
        self.det_timeout = self.get_parameter('detection_timeout_s').value
        rate_hz = self.get_parameter('rate_hz').value

        # State
        self._active = False          # True when FSM is in APPROACH_TARGET
        self._last_det_time = None    # Time of last valid detection
        self._latest_cx = 0.5        # Normalized horizontal center of circle
        self._latest_w = 0.0         # Normalized width of circle

        # Subscribers
        self.det_sub = self.create_subscription(
            Detections2D,
            '/vision/detections',
            self._detection_callback,
            10
        )
        self.fsm_sub = self.create_subscription(
            String,
            '/control/fsm_state',
            self._fsm_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/control/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info('Blue circle controller initialized')

    def _fsm_callback(self, msg: String):
        was_active = self._active
        self._active = (msg.data == 'APPROACH_TARGET')
        if self._active and not was_active:
            self.get_logger().info('Blue circle controller ACTIVE — line follow is disabled')
        elif not self._active and was_active:
            self.get_logger().info('Blue circle controller INACTIVE')

    def _detection_callback(self, msg: Detections2D):
        for det in msg.detections:
            if det.class_name == self.target_class:
                self._latest_cx = det.cx
                self._latest_w = det.w
                self._last_det_time = self.get_clock().now()
                return

    def _control_loop(self):
        if not self._active:
            return

        cmd = Twist()

        # Check detection freshness
        if self._last_det_time is None:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self._last_det_time).nanoseconds / 1e9
        if age > self.det_timeout:
            # Lost circle — publish stop
            self.cmd_pub.publish(cmd)
            return

        # Proportional angular control to center on the circle horizontally
        error_x = self._latest_cx - 0.5  # positive = circle is to the right
        cmd.angular.z = float(-self.kp_angular * error_x)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        # Slow constant forward speed
        cmd.linear.x = float(self.linear_speed)

        self.get_logger().info(
            f'Steering to circle: cx={self._latest_cx:.2f}  '
            f'error={error_x:+.2f}  ang={cmd.angular.z:+.2f}  '
            f'fwd={cmd.linear.x:.2f}',
            throttle_duration_sec=0.3
        )
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BlueCircleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
