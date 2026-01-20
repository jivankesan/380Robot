#!/usr/bin/env python3
"""Teleop node for keyboard control of 380Robot.

Simple keyboard interface for manual robot control.
"""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_interfaces.msg import ClawCommand


HELP_MSG = """
380Robot Keyboard Teleop
------------------------
Movement:
   w
a  s  d

w/s : forward/backward
a/d : turn left/right
space : stop

Claw:
o : open claw
c : close claw

Speed:
+/= : increase speed
-   : decrease speed

q : quit
"""


class TeleopNode(Node):
    """ROS 2 node for keyboard teleop."""

    def __init__(self):
        super().__init__('teleop_node')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)

        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value

        # Current velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/control/cmd_vel_limited',
            10
        )
        self.claw_pub = self.create_publisher(
            ClawCommand,
            '/claw/cmd',
            10
        )

        # Store terminal settings
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Teleop node initialized')
        print(HELP_MSG)
        print(f'Current speed: linear={self.linear_speed:.2f}, '
              f'angular={self.angular_speed:.2f}')

    def get_key(self):
        """Get a single keypress."""
        if not sys.stdin.isatty():
            return None

        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """Main loop for keyboard input."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key is None:
                    break

                twist = Twist()

                if key == 'w':
                    self.linear_vel = self.linear_speed
                    self.angular_vel = 0.0
                elif key == 's':
                    self.linear_vel = -self.linear_speed
                    self.angular_vel = 0.0
                elif key == 'a':
                    self.linear_vel = 0.0
                    self.angular_vel = self.angular_speed
                elif key == 'd':
                    self.linear_vel = 0.0
                    self.angular_vel = -self.angular_speed
                elif key == ' ':
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                elif key == 'o':
                    claw_cmd = ClawCommand()
                    claw_cmd.mode = ClawCommand.MODE_OPEN
                    claw_cmd.position = 0.0
                    self.claw_pub.publish(claw_cmd)
                    print('Claw: OPEN')
                    continue
                elif key == 'c':
                    claw_cmd = ClawCommand()
                    claw_cmd.mode = ClawCommand.MODE_CLOSE
                    claw_cmd.position = 1.0
                    self.claw_pub.publish(claw_cmd)
                    print('Claw: CLOSE')
                    continue
                elif key in ['+', '=']:
                    self.linear_speed += self.linear_step
                    self.angular_speed += self.angular_step
                    print(f'Speed: linear={self.linear_speed:.2f}, '
                          f'angular={self.angular_speed:.2f}')
                    continue
                elif key == '-':
                    self.linear_speed = max(0.05, self.linear_speed - self.linear_step)
                    self.angular_speed = max(0.1, self.angular_speed - self.angular_step)
                    print(f'Speed: linear={self.linear_speed:.2f}, '
                          f'angular={self.angular_speed:.2f}')
                    continue
                elif key == 'q':
                    # Stop before quitting
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    break
                else:
                    continue

                twist.linear.x = self.linear_vel
                twist.angular.z = self.angular_vel
                self.cmd_pub.publish(twist)

                print(f'\rVel: linear={self.linear_vel:.2f}, '
                      f'angular={self.angular_vel:.2f}   ', end='')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Restore terminal settings
            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Stop robot
            twist = Twist()
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
