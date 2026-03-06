#!/usr/bin/env python3
"""Autonomous demo node for 380Robot assessment.

Sequence:
  1. Move forward  (FORWARD_DURATION seconds)
  2. Stop          (PAUSE_DURATION seconds)
  3. Move backward (BACKWARD_DURATION seconds)
  4. Stop          (PAUSE_DURATION seconds)
  5. Open claw     (CLAW_HOLD seconds)
  6. Close claw
  7. Shutdown

Run after bringup_real is already up:
  python3 /workspaces/380Robot/scripts/demo.py
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_interfaces.msg import ClawCommand

# ── Tune these for your physical setup ──────────────────────────────────────
FORWARD_SPEED   =  0.3   # m/s
BACKWARD_SPEED  = -0.3   # m/s
FORWARD_DURATION  = 2.0  # seconds
BACKWARD_DURATION = 2.0  # seconds
PAUSE_DURATION    = 1.0  # seconds between moves
CLAW_HOLD         = 2.0  # seconds to hold claw open
# ────────────────────────────────────────────────────────────────────────────

CMD_TOPIC  = '/control/cmd_vel_limited'
CLAW_TOPIC = '/claw/cmd'


class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.cmd_pub  = self.create_publisher(Twist,       CMD_TOPIC,  10)
        self.claw_pub = self.create_publisher(ClawCommand, CLAW_TOPIC, 10)

    def drive(self, linear: float, duration: float):
        """Publish a constant velocity for `duration` seconds, then stop."""
        twist = Twist()
        twist.linear.x = linear

        end = time.time() + duration
        rate = self.create_rate(20)  # 20 Hz publish loop
        self.get_logger().info(
            f'Driving: linear={linear:+.2f} m/s for {duration:.1f}s'
        )
        while time.time() < end:
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def claw(self, open_: bool):
        msg = ClawCommand()
        if open_:
            msg.mode = ClawCommand.MODE_OPEN
            msg.position = 0.0
            self.get_logger().info('Claw: OPEN')
        else:
            msg.mode = ClawCommand.MODE_CLOSE
            msg.position = 1.0
            self.get_logger().info('Claw: CLOSE')
        self.claw_pub.publish(msg)

    def run(self):
        # Give the system a moment to settle
        time.sleep(1.0)

        # 1. Forward
        self.drive(FORWARD_SPEED, FORWARD_DURATION)
        time.sleep(PAUSE_DURATION)

        # 2. Backward
        self.drive(BACKWARD_SPEED, BACKWARD_DURATION)
        time.sleep(PAUSE_DURATION)

        # 3. Claw open → hold → close
        self.claw(open_=True)
        time.sleep(CLAW_HOLD)
        self.claw(open_=False)
        time.sleep(1.0)

        self.get_logger().info('Demo complete.')


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Safety stop
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
