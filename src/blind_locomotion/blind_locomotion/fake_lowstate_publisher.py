#!/usr/bin/env python3
"""
Fake LowState publisher for sim-only testing.
Simulates robot state feedback without a real robot.

Starts in "laying down" position (all joints near zero/flat).
The state can be updated to track commanded positions for realistic testing.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from unitree_go.msg import LowState, IMUState, MotorState


class FakeLowstatePublisher(Node):
    def __init__(self):
        super().__init__('fake_lowstate_publisher')
        
        # Parameters
        self.declare_parameter('publish_frequency', 500.0)  # Match real robot
        self.declare_parameter('verbose', False)
        self.declare_parameter('track_commands', True)  # Track /lowcmd for position updates
        self.declare_parameter('position_rate', 0.1)  # How fast to move toward command (0-1)
        
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.track_commands = self.get_parameter('track_commands').get_parameter_value().bool_value
        self.position_rate = self.get_parameter('position_rate').get_parameter_value().double_value
        
        # Laying down position (flat on ground - all joints relatively straight)
        # This simulates the Go2 after sport mode is disabled
        self.laying_down_pos = np.array([
            0.0, 0.8, -1.5,   # FR: hip, thigh, calf (legs slightly bent outward)
            0.0, 0.8, -1.5,   # FL
            0.0, 1.0, -1.5,   # RR  
            0.0, 1.0, -1.5,   # RL
        ])
        
        # Standing position for reference
        self.standing_pos = np.array([
            0.0, 1.1, -1.8,   # FR
            0.0, 1.1, -1.8,   # FL
            0.0, 1.1, -1.8,   # RR
            0.0, 1.1, -1.8,   # RL
        ])
        
        # Sitting position for reference
        self.sitting_pos = np.array([
            -0.1, 1.1, -2.0,   # FR
            -0.1, 1.1, -2.0,   # FL
            -0.1, 1.1, -2.6,   # RR
            -0.1, 1.1, -2.6,   # RL
        ])
        
        # Current simulated state - start laying down
        self.current_pos = self.laying_down_pos.copy()
        self.current_vel = np.zeros(12)
        self.target_pos = self.laying_down_pos.copy()
        
        # IMU state (robot laying flat)
        self.quaternion = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion (w, x, y, z)
        self.gyroscope = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 9.81]  # Gravity in z
        
        # Publisher
        self.publisher = self.create_publisher(LowState, '/lowstate', 10)
        
        # Subscriber to track commands (optional)
        if self.track_commands:
            from unitree_go.msg import LowCmd
            self.cmd_sub = self.create_subscription(
                LowCmd,
                '/lowcmd',
                self.lowcmd_callback,
                10
            )
            self.get_logger().info('Tracking /lowcmd for position updates')
        
        # Timer
        self.timer = self.create_timer(1.0 / freq, self.publish_state)
        
        self.get_logger().info(f'Fake lowstate publisher started at {freq} Hz')
        self.get_logger().info(f'Starting position: LAYING DOWN')
        self.get_logger().info(f'Joint positions: {self.current_pos.tolist()}')
        
    def lowcmd_callback(self, msg):
        """Update target positions based on received commands."""
        for i in range(12):
            if msg.motor_cmd[i].mode == 0x01:  # Position control mode
                self.target_pos[i] = msg.motor_cmd[i].q
                
    def publish_state(self):
        """Publish simulated low state."""
        # Gradually move toward target position (simulate motor response)
        if self.track_commands:
            diff = self.target_pos - self.current_pos
            self.current_vel = diff * self.position_rate * 50  # Approximate velocity
            self.current_pos += diff * self.position_rate
        
        # Build message
        msg = LowState()
        
        # IMU state
        msg.imu_state = IMUState()
        msg.imu_state.quaternion = self.quaternion
        msg.imu_state.gyroscope = self.gyroscope
        msg.imu_state.accelerometer = self.accelerometer
        
        # Motor states
        for i in range(20):  # Go2 has 20 motor slots (12 leg + extras)
            motor = MotorState()
            if i < 12:
                motor.q = float(self.current_pos[i])
                motor.dq = float(self.current_vel[i])
                motor.tau_est = 0.0
                motor.mode = 0x01
            else:
                motor.q = 0.0
                motor.dq = 0.0
                motor.tau_est = 0.0
                motor.mode = 0x00
            msg.motor_state[i] = motor
        
        self.publisher.publish(msg)
        
        if self.verbose:
            self.get_logger().debug(f'Published state: pos={self.current_pos[:3]}...')
            
    def get_pose_name(self):
        """Determine which pose we're closest to."""
        dist_laying = np.linalg.norm(self.current_pos - self.laying_down_pos)
        dist_standing = np.linalg.norm(self.current_pos - self.standing_pos)
        dist_sitting = np.linalg.norm(self.current_pos - self.sitting_pos)
        
        min_dist = min(dist_laying, dist_standing, dist_sitting)
        if min_dist == dist_laying:
            return "LAYING DOWN"
        elif min_dist == dist_standing:
            return "STANDING"
        else:
            return "SITTING"


def main(args=None):
    rclpy.init(args=args)
    node = FakeLowstatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
