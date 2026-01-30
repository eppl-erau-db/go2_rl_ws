#!/usr/bin/env python3
"""
LowCmd decoder - human-readable display of motor commands.
Shows what commands are being sent to the robot in a clear format.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from unitree_go.msg import LowCmd


# Joint names for Go2
JOINT_NAMES = [
    'FR_hip', 'FR_thigh', 'FR_calf',
    'FL_hip', 'FL_thigh', 'FL_calf',
    'RR_hip', 'RR_thigh', 'RR_calf',
    'RL_hip', 'RL_thigh', 'RL_calf',
]

# Mode names
MODE_NAMES = {
    0x00: 'DISABLED',
    0x01: 'POSITION',
    0x0A: 'TORQUE',
}

# Standing position for reference
STAND_POS = np.array([
    0.0, 1.1, -1.8,   # FR
    0.0, 1.1, -1.8,   # FL
    0.0, 1.1, -1.8,   # RR
    0.0, 1.1, -1.8,   # RL
])

# Sitting position for reference
SIT_POS = np.array([
    -0.1, 1.1, -2.0,   # FR
    -0.1, 1.1, -2.0,   # FL
    -0.1, 1.1, -2.6,   # RR
    -0.1, 1.1, -2.6,   # RL
])


class LowCmdDecoder(Node):
    def __init__(self):
        super().__init__('lowcmd_decoder')
        
        # Parameters
        self.declare_parameter('verbose', True)
        self.declare_parameter('show_all_joints', False)  # Show all 12 joints or just summary
        self.declare_parameter('update_rate', 2.0)  # How often to print (Hz)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.show_all_joints = self.get_parameter('show_all_joints').get_parameter_value().bool_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # State
        self.last_cmd = None
        self.cmd_count = 0
        self.last_print_time = self.get_clock().now()
        self.print_interval = 1.0 / update_rate
        
        # Subscriber
        self.subscription = self.create_subscription(
            LowCmd,
            '/lowcmd',
            self.lowcmd_callback,
            10
        )
        
        self.get_logger().info('LowCmd decoder started')
        self.get_logger().info(f'Update rate: {update_rate} Hz, Show all joints: {self.show_all_joints}')
        print('\n' + '='*70)
        print('LowCmd Decoder - Waiting for commands on /lowcmd...')
        print('='*70)
        
    def lowcmd_callback(self, msg):
        """Process and display received command."""
        self.last_cmd = msg
        self.cmd_count += 1
        
        # Rate limit printing
        now = self.get_clock().now()
        elapsed = (now - self.last_print_time).nanoseconds / 1e9
        if elapsed < self.print_interval:
            return
        self.last_print_time = now
        
        self.print_command(msg)
        
    def print_command(self, msg):
        """Print human-readable command summary."""
        # Extract positions and gains
        positions = np.array([msg.motor_cmd[i].q for i in range(12)])
        kps = np.array([msg.motor_cmd[i].kp for i in range(12)])
        kds = np.array([msg.motor_cmd[i].kd for i in range(12)])
        modes = [msg.motor_cmd[i].mode for i in range(12)]
        
        # Determine command type based on gains and positions
        cmd_type = self.identify_command_type(positions, kps, modes)
        
        # Calculate distances to known poses
        dist_stand = np.linalg.norm(positions - STAND_POS)
        dist_sit = np.linalg.norm(positions - SIT_POS)
        
        # Clear line and print
        print('\n' + '-'*70)
        print(f'Command #{self.cmd_count} | Type: {cmd_type}')
        print(f'Distance to STAND: {dist_stand:.3f} rad | Distance to SIT: {dist_sit:.3f} rad')
        
        if self.show_all_joints:
            print('\nJoint Details:')
            print(f'{"Joint":<12} {"Mode":<10} {"Position":>10} {"Kp":>8} {"Kd":>8}')
            print('-'*50)
            for i in range(12):
                mode_name = MODE_NAMES.get(modes[i], f'0x{modes[i]:02X}')
                print(f'{JOINT_NAMES[i]:<12} {mode_name:<10} {positions[i]:>10.3f} {kps[i]:>8.1f} {kds[i]:>8.2f}')
        else:
            # Summary view
            print(f'\nPositions (deg): FR[{np.degrees(positions[0:3])}]')
            print(f'                 FL[{np.degrees(positions[3:6])}]')
            print(f'                 RR[{np.degrees(positions[6:9])}]')
            print(f'                 RL[{np.degrees(positions[9:12])}]')
            print(f'Gains: Kp={kps[0]:.1f}, Kd={kds[0]:.2f} (using first joint)')
            
        # Mode summary
        mode_counts = {}
        for m in modes:
            mode_name = MODE_NAMES.get(m, f'0x{m:02X}')
            mode_counts[mode_name] = mode_counts.get(mode_name, 0) + 1
        print(f'Motor modes: {mode_counts}')
        
    def identify_command_type(self, positions, kps, modes):
        """Identify what type of command this is."""
        # Check if all motors disabled
        if all(m == 0x00 for m in modes):
            return 'KILLED (motors disabled)'
        
        # Check gains to identify mode
        kp_avg = np.mean(kps)
        
        if kp_avg < 1:
            return 'IDLE (zero gains)'
        elif kp_avg < 10:
            return 'DAMPING (low gains)'
        elif kp_avg < 25:
            return 'WALKING (RL control)'
        elif kp_avg < 40:
            return 'SITTING (medium gains)'
        else:
            return 'STANDING (high gains)'


def main(args=None):
    rclpy.init(args=args)
    node = LowCmdDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('\nLowCmd decoder shutdown.')


if __name__ == '__main__':
    main()
