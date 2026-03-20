#!/usr/bin/env python3
"""
Quick test: drive forward 0.5 s, close claw, rotate claw, stop.
Run inside the dev container:
  python3 /workspaces/380Robot/scripts/test_pickup.py
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robot_interfaces.msg import ClawCommand


class TestPickup(Node):
    def __init__(self):
        super().__init__('test_pickup')
        self.cmd_pub    = self.create_publisher(Twist,        '/control/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool,         '/control/enable',  10)
        self.claw_pub   = self.create_publisher(ClawCommand,  '/claw/cmd',        10)

    def enable(self, on: bool):
        msg = Bool(); msg.data = on
        self.enable_pub.publish(msg)

    def drive(self, linear: float, angular: float = 0.0):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def claw_gripper(self, position: float):
        msg = ClawCommand(); msg.mode = 2; msg.position = position
        self.claw_pub.publish(msg)

    def claw_rotation(self, position: float):
        msg = ClawCommand(); msg.mode = 1; msg.position = position
        self.claw_pub.publish(msg)

    def sleep(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        print("Waiting 1 s for nodes to connect...")
        self.sleep(1.0)

        print("Driving forward 0.5 s...")
        self.enable(True)
        self.drive(0.20)
        self.sleep(0.5)

        print("Stopping...")
        self.drive(0.0)
        self.enable(False)
        self.sleep(0.4)

        print("Closing gripper...")
        self.claw_gripper(1.0)
        self.sleep(1.0)

        print("Rotating claw to carry position...")
        self.claw_rotation(1.0)
        self.sleep(0.5)

        print("Done.")


def main():
    rclpy.init()
    node = TestPickup()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
