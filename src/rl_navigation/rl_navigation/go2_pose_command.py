#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import termios
import tty
import select


class TeleopPoseCommand(Node):
    def __init__(self):
        super().__init__('teleop_pose_command')
        self.pose_publisher = self.create_publisher(
            Float32MultiArray,
            'cmd_pose',
            10)
        self.cmd_pose = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Teleop pose command started. Enter pose commands from keyboard.")
        
        # Set up terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        # Check for input at regular intervals
        self.timer = self.create_timer(0.1, self.check_input)

    def check_input(self):
        if self.is_input_available():
            self.get_pose_command()
        
        # Publish the pose command
        msg = Float32MultiArray(data=self.cmd_pose)
        self.pose_publisher.publish(msg)

    def is_input_available(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def get_pose_command(self):
        try:
            self.cmd_pose[0] = float(input("Enter x position: "))
            self.cmd_pose[1] = float(input("Enter y position: "))
            self.cmd_pose[2] = float(input("Enter z position: "))
            self.cmd_pose[3] = float(input("Enter heading (in radians): "))
        except ValueError:
            self.get_logger().info("Invalid input. Please enter numeric values.")
        except KeyboardInterrupt:
            self.get_logger().info("Exiting...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPoseCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)


if __name__ == '__main__':
    main()
