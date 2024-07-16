#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.speed = 1.0  # linear speed (m/s)
        self.turn = 1.0  # angular speed (rad/s)
        self.get_logger().info("Keyboard teleop started. Use WASD for linear motion, and O/P for spinning.")
        self.settings = termios.tcgetattr(sys.stdin)
        self.active_keys = set()
        self.run()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update_twist(self):
        self.twist = Twist()
        if 'w' in self.active_keys:
            self.twist.linear.x = self.speed
        if 's' in self.active_keys:
            self.twist.linear.x = -self.speed
        if 'a' in self.active_keys:
            self.twist.linear.y = self.speed
        if 'd' in self.active_keys:
            self.twist.linear.y = -self.speed
        if 'o' in self.active_keys:
            self.twist.angular.z = self.turn
        if 'p' in self.active_keys:
            self.twist.angular.z = -self.turn
        self.publisher.publish(self.twist)

    def run(self):
        try:
            while True:
                key = self.getKey()

                if key == '\x03':  # Ctrl+C
                    self.twist = Twist()
                    self.publisher.publish(self.twist)
                    break

                if key:
                    self.active_keys.add(key)
                else:
                    self.active_keys.clear()

                self.update_twist()

        except Exception as e:
            self.get_logger().error(str(e))

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
