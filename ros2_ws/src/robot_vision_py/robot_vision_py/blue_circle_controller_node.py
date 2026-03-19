#!/usr/bin/env python3
"""Visual approach controller for 380Robot.

Activates during APPROACH_TARGET (steers toward blue_circle) and
APPROACH_DROP (steers toward green_box).  The FSM decides when to stop
based on cy threshold / size threshold; this node just keeps the robot
centered on the target and moving forward while active.

Control law:
  angular_z = -kp_angular * (cx - 0.5)   # positive error = target is right
  linear.x  = linear_speed_mps           # constant forward creep
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from robot_interfaces.msg import Detections2D


class VisualApproachControllerNode(Node):
    """Proportional controller to navigate toward a detected target centroid."""

    def __init__(self):
        super().__init__('blue_circle_controller_node')

        # Parameters
        self.declare_parameter('target_class', 'blue_circle')
        self.declare_parameter('drop_class', 'green_box')
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('linear_speed_mps', 0.06)
        self.declare_parameter('linear_speed_near_mps', 0.03)  # creep speed when close
        self.declare_parameter('near_w_threshold', 0.40)       # switch to creep above this w
        self.declare_parameter('max_angular_rps', 1.2)
        self.declare_parameter('center_tolerance_x', 0.12)
        self.declare_parameter('detection_timeout_s', 0.5)
        self.declare_parameter('rate_hz', 20.0)

        self.target_class  = self.get_parameter('target_class').value
        self.drop_class    = self.get_parameter('drop_class').value
        self.kp_angular    = self.get_parameter('kp_angular').value
        self.linear_speed  = self.get_parameter('linear_speed_mps').value
        self.linear_near   = self.get_parameter('linear_speed_near_mps').value
        self.near_w        = self.get_parameter('near_w_threshold').value
        self.max_angular   = self.get_parameter('max_angular_rps').value
        self.center_tol_x  = self.get_parameter('center_tolerance_x').value
        self.det_timeout   = self.get_parameter('detection_timeout_s').value
        rate_hz            = self.get_parameter('rate_hz').value

        # State
        self._active         = False
        self._current_class  = self.target_class  # changes with FSM state
        self._last_det_time  = None
        self._latest_cx      = 0.5
        self._latest_cy      = 0.5
        self._latest_w       = 0.0

        # Subscribers
        self.det_sub = self.create_subscription(
            Detections2D, '/vision/detections', self._detection_callback, 10)
        self.fsm_sub = self.create_subscription(
            String, '/control/fsm_state', self._fsm_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/control/cmd_vel', 10)

        # Control timer
        self.timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info('Visual approach controller initialized')

    def _fsm_callback(self, msg: String):
        was_active = self._active
        state = msg.data

        if state == 'APPROACH_TARGET':
            self._active        = True
            self._current_class = self.target_class
        elif state == 'APPROACH_DROP':
            self._active        = True
            self._current_class = self.drop_class
        else:
            self._active = False

        if self._active and not was_active:
            self.get_logger().info(
                f'Visual approach ACTIVE — tracking [{self._current_class}]')
        elif not self._active and was_active:
            self.get_logger().info('Visual approach INACTIVE')

    def _detection_callback(self, msg: Detections2D):
        for det in msg.detections:
            if det.class_name == self._current_class:
                self._latest_cx     = det.cx
                self._latest_cy     = det.cy
                self._latest_w      = det.w
                self._last_det_time = self.get_clock().now()
                return

    def _control_loop(self):
        if not self._active:
            return

        cmd = Twist()

        if self._last_det_time is None:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self._last_det_time).nanoseconds / 1e9
        if age > self.det_timeout:
            self.cmd_pub.publish(cmd)  # stop if detection stale
            return

        # Proportional angular to center target horizontally
        error_x       = self._latest_cx - 0.5  # positive = target is to the right
        cmd.angular.z = float(-self.kp_angular * error_x)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        # Two-speed forward: creep when close (large w) to minimise coast overshoot
        fwd = self.linear_near if self._latest_w >= self.near_w else self.linear_speed
        cmd.linear.x = float(fwd)

        self.get_logger().info(
            f'[{self._current_class}] cx={self._latest_cx:.2f} w={self._latest_w:.2f} '
            f'err={error_x:+.2f} ang={cmd.angular.z:+.2f} fwd={cmd.linear.x:.2f}',
            throttle_duration_sec=0.3
        )
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VisualApproachControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
