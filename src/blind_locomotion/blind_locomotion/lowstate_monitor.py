#!/usr/bin/env python3
"""
LowState Monitor - displays actual joint positions from the robot.
Use this to see what angles the robot is currently at (e.g., when laying down)
so you can tune target positions in constants.hpp.
"""
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState


class LowStateMonitor(Node):
    def __init__(self):
        super().__init__('lowstate_monitor')
        
        # Parameters
        self.declare_parameter('update_rate', 2.0)  # Hz
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        self.last_print_time = self.get_clock().now()
        self.print_interval = 1.0 / update_rate
        self.msg_count = 0
        
        # Subscriber
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.lowstate_callback,
            10
        )
        
        self.get_logger().info(f'LowState Monitor started (rate: {update_rate} Hz)')
        print('\n' + '='*70)
        print('LowState Monitor - Showing ACTUAL joint positions from robot')
        print('Use these values to tune SitPos/StandPos in constants.hpp')
        print('='*70)
        
    def lowstate_callback(self, msg):
        """Process and display joint positions."""
        self.msg_count += 1
        
        # Rate limit printing
        now = self.get_clock().now()
        elapsed = (now - self.last_print_time).nanoseconds / 1e9
        if elapsed < self.print_interval:
            return
        self.last_print_time = now
        
        # Extract joint positions
        q = [msg.motor_state[i].q for i in range(12)]
        
        # Print in clear format
        print(f'\n[LowState #{self.msg_count}] Joint positions (radians):')
        print(f'  FR: hip={q[0]:>7.3f}  thigh={q[1]:>7.3f}  calf={q[2]:>7.3f}')
        print(f'  FL: hip={q[3]:>7.3f}  thigh={q[4]:>7.3f}  calf={q[5]:>7.3f}')
        print(f'  RR: hip={q[6]:>7.3f}  thigh={q[7]:>7.3f}  calf={q[8]:>7.3f}')
        print(f'  RL: hip={q[9]:>7.3f}  thigh={q[10]:>7.3f}  calf={q[11]:>7.3f}')
        
        # Print copy-paste format for constants.hpp
        print(f'\n  // Copy-paste for constants.hpp SitPos:')
        print(f'  {q[0]:>7.3f}, {q[1]:>6.3f}, {q[2]:>6.3f},   // FR')
        print(f'  {q[3]:>7.3f}, {q[4]:>6.3f}, {q[5]:>6.3f},   // FL')
        print(f'  {q[6]:>7.3f}, {q[7]:>6.3f}, {q[8]:>6.3f},   // RR')
        print(f'  {q[9]:>7.3f}, {q[10]:>6.3f}, {q[11]:>6.3f},   // RL')


def main(args=None):
    rclpy.init(args=args)
    node = LowStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('\nLowState monitor shutdown.')


if __name__ == '__main__':
    main()
