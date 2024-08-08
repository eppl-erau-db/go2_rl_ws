#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_sdk2py import MotionSwitcherClient  # Adjust the import according to the actual package structure
import logging

class MotionSwitcherNode(Node):

    def __init__(self):
        super().__init__('motion_switcher_node')
        self.logger = self.get_logger()
        self.communicator = None  # Initialize your communicator here
        self.motion_switcher_client = MotionSwitcherClient(self.communicator, logger=self.logger)
        self.motion_switcher_client.Init()

        # Example usage of the client
        self.check_and_switch_motion_mode("ai")

    def check_and_switch_motion_mode(self, new_mode: str):
        try:
            # Read the current mode
            current_mode_code = self.motion_switcher_client.GetMode()
            self.get_logger().info(f'Current Mode response code: {current_mode_code}')
            
            # Check if the current mode is already the desired mode
            if current_mode_code == new_mode:
                self.get_logger().info(f'Mode is already set to {new_mode}. No change needed.')
                return
            
            # Set the new mode
            code = self.motion_switcher_client.SetMode(new_mode)
            self.get_logger().info(f'SetMode response code: {code}')
        except Exception as e:
            self.get_logger().error(f'Failed to switch mode: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotionSwitcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
