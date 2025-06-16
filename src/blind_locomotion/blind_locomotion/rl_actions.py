#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from unitree_go.msg import LowState, IMUState
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
import onnxruntime as ort


class RLActionsNode(Node):
    def __init__(self):
        super().__init__("rl_actions_publisher")

        # Parameters
        self.declare_parameter("policy_name", "flat_policy_v5")  # Default model name
        policy_name = self.get_parameter("policy_name").get_parameter_value().string_value

        # Publisher
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'actions',
            10)
        
        # Subscribers
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate', 
            self.lowstate_callback,
            10)
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Finding and loading the ONNX model - can be changed to desired model
        share_dir = get_package_share_directory('blind_locomotion')
        model_path = os.path.join(share_dir, 'models', f'{policy_name}.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(
            self.timer_period,
            self.generate_actions)
        
        # Isaac Lab Constants
        self.scale_factor = 0.25
        self.motor_qs_def = [ 0.0,-0.0, 0.0,-0.0,
                              1.1, 1.1, 1.1, 1.1,
                             -1.8,-1.8,-1.8,-1.8]
        
        # Initialize Empty Vectors
        self.init_raw_action = np.zeros(12)
        self.raw_actions = None
        self.processed_actions = None
        self.joint_pos_init = None
        self.cmd_vel = np.zeros(3)
        self.obs = np.zeros(45)  
        self.quat = np.zeros(4)
        self.gravity_proj = np.zeros(3)
        self.motor_qs = np.zeros(12)
        self.motor_dqs = np.zeros(12)
        
        
    def lowstate_callback(self, msg):
        '''BASE_ANGULAR_VELOCITY'''
        # Map base_ang_vel to obs
        self.obs[0:3] = np.array(msg.imu_state.gyroscope[0:3])

        '''PROJECTED_GRAVITY_VECTOR'''
        # Quaternion format: (0-w, 1-x, 2-y, 3-z)
        self.quat = np.array(msg.imu_state.quaternion[0:4])

        # Calculate projected gravity vector
        self.gravity_proj = self.projected_gravity_vector(self.quat)

        # Map projected gravity vector to obs
        self.obs[3:6] = self.gravity_proj

        '''JOINT_POSITIONS'''
        # Mapped joint positions
        self.motor_qs = [
            msg.motor_state[3].q,    # 0  -> FL_hip_joint   to FL_hip   -> 3
            msg.motor_state[0].q,    # 1  -> FR_hip_joint   to FR_hip   -> 0
            msg.motor_state[9].q,    # 2  -> RL_hip_joint   to RL_hip   -> 9
            msg.motor_state[6].q,    # 3  -> RR_hip_joint   to RR_hip   -> 6
            msg.motor_state[4].q,    # 4  -> FL_thigh_joint to FL_thigh -> 4
            msg.motor_state[1].q,    # 5  -> FR_thigh_joint to FR_thigh -> 1
            msg.motor_state[10].q,   # 6  -> RL_thigh_joint to RL_thigh -> 10
            msg.motor_state[7].q,    # 7  -> RR_thigh_joint to RR_thigh -> 7
            msg.motor_state[5].q,    # 8  -> FL_calf_joint  to FL_calf  -> 5
            msg.motor_state[2].q,    # 9  -> FR_calf_joint  to FR_calf  -> 2
            msg.motor_state[11].q,   # 10 -> RL_calf_joint  to RL_calf  -> 11
            msg.motor_state[8].q,    # 11 -> RR_calf_joint  to RR_calf  -> 8
        ]

        # Subtracting defaults
        self.motor_qs = self.motor_qs - self.motor_qs_def

        # Map joint positions vector to obs
        self.obs[9:21] = self.motor_qs

        '''JOINT_VELOCITIES'''
        # Mapped joint velocities
        self.motor_dqs = [
            msg.motor_state[3].dq,   # 0  -> FL_hip_joint   to FL_hip   -> 3
            msg.motor_state[0].dq,   # 1  -> FR_hip_joint   to FR_hip   -> 0
            msg.motor_state[9].dq,   # 2  -> RL_hip_joint   to RL_hip   -> 9
            msg.motor_state[6].dq,   # 3  -> RR_hip_joint   to RR_hip   -> 6
            msg.motor_state[4].dq,   # 4  -> FL_thigh_joint to FL_thigh -> 4
            msg.motor_state[1].dq,   # 5  -> FR_thigh_joint to FR_thigh -> 1
            msg.motor_state[10].dq,  # 6  -> RL_thigh_joint to RL_thigh -> 10
            msg.motor_state[7].dq,   # 7  -> RR_thigh_joint to RR_thigh -> 7
            msg.motor_state[5].dq,   # 8  -> FL_calf_joint  to FL_calf  -> 5
            msg.motor_state[2].dq,   # 9  -> FR_calf_joint  to FR_calf  -> 2
            msg.motor_state[11].dq,  # 10 -> RL_calf_joint  to RL_calf  -> 11
            msg.motor_state[8].dq,   # 11 -> RR_calf_joint  to RR_calf  -> 8
        ]

        # Map joint velocities vector to obs
        self.obs[21:33] = self.motor_qs
    
    def cmd_vel_callback(self, msg):
        # Converting twist message to array for concatenation
        self.cmd_vel[0:3] = [msg.linear.x, msg.linear.y, msg.angular.z]

        # Check for valid inputs
        if self.cmd_vel is None or all(abs(v) < 0.1 for v in self.cmd_vel):
            self.cmd_vel = np.array([0.0, 0.0, 0.0])
        
        '''COMMANDED_LINEAR_VELOCITY'''
        # Map commanded linear velocity vector to obs
        self.obs[6:9] = self.cmd_vel[0:3]

    def generate_actions(self):
        '''LAST_ACTION'''
        # Adding last action or first raw action for obs
        if self.raw_actions is not None:
            self.obs[33:45] = self.raw_actions
        else:
            self.obs[33:45] = self.init_raw_action

        # Make obs into np array & reshape for the batch size
        self.obs = self.obs.astype(np.float32).reshape(1, -1)

        # Run inference
        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: self.obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.raw_actions = ort_outs[0].flatten()
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.raw_actions = np.zeros_like(self.init_raw_action)

        # Accounting for offset and scale (Isaac Lab)
        self.processed_actions = (self.raw_actions * self.scale_factor + self.motor_qs_def).tolist()
        self.processed_actions_ordered = [
            self.processed_actions[1],  # 1  -> FR_hip_joint   to FR_hip   -> 0
            self.processed_actions[5],  # 5  -> FR_thigh_joint to FR_thigh -> 1
            self.processed_actions[9],  # 9  -> FR_calf_joint  to FR_calf  -> 2
            self.processed_actions[0],  # 0  -> FL_hip_joint   to FL_hip   -> 3
            self.processed_actions[4],  # 4  -> FL_thigh_joint to FL_thigh -> 4
            self.processed_actions[8],  # 8  -> FL_calf_joint  to FL_calf  -> 5
            self.processed_actions[3],  # 3  -> RR_hip_joint   to RR_hip   -> 6
            self.processed_actions[7],  # 7  -> RR_thigh_joint to RR_thigh -> 7
            self.processed_actions[11], # 11 -> RR_calf_joint  to RR_calf  -> 8
            self.processed_actions[2],  # 2  -> RL_hip_joint   to RL_hip   -> 9
            self.processed_actions[6],  # 6  -> RL_thigh_joint to RL_thigh -> 10
            self.processed_actions[10]  # 10 -> RL_calf_joint  to RL_calf  -> 11
        ]

        # Motor limits
        self.motor_limits_ordered = [
            [-0.837, 0.837],  # Front Hip
            [-3.490, 1.570],  # Front Thigh
            [-2.720, 0.837],  # Front Calf
            [-0.837, 0.837],  # Front Hip
            [-3.490, 1.570],  # Front Thigh
            [-2.720, 0.837],  # Front Calf
            [-0.837, 0.837],  # Rear Hip
            [-4.530, 1.570],  # Rear Thigh
            [-2.720, 0.837],  # Rear Calf
            [-0.837, 0.837],  # Rear Hip
            [-4.530, 1.570],  # Rear Thigh
            [-2.720, 0.837]   # Rear Calf
        ]

        # Clipping actions by a scale of the motor limits:
        clipped_actions_ordered = [0]*12
        for i, action in enumerate(self.processed_actions_ordered):
            min_limit, max_limit = self.motor_limits_ordered[i]

            # Applying scale factor
            min_limit = min_limit * 0.95
            max_limit = max_limit * 0.95
            clipped_actions_ordered[i] = max(min(action, max_limit), min_limit)

        # Publishing action messages
        action_msg = Float32MultiArray()
        action_msg.data = clipped_actions_ordered
        self.publisher.publish(action_msg)

    def projected_gravity_vector(self, imu_quaternion):
        '''
        CHECK FOR CONVENTION ERROR
        
        TODO: READ DOCS TO UNDERSTAND THE RESULTING ROTATION.
        '''

        # Use rotation from quaternion to find proj g
        rotation = R.from_quat(imu_quaternion)
        gravity_vec_w = np.array([0.0, 0.0, -1.0])  # Gravity vector in world
        gravity_proj = -1 * rotation.apply(gravity_vec_w)
        return gravity_proj
    
        # Could also be this?
        rotation = R.from_quat(imu_quaternion)
        gravity_vec_w = np.array([0.0, 0.0, -1.0])  # Gravity vector in world frame
        gravity_proj = rotation.inv().apply(gravity_vec_w)  # Convert to body frame
        return gravity_proj
    
    def load_onnx_model(self, model_path):
        try:
            self.ort_session = ort.InferenceSession(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")   


def main(args=None):
    rclpy.init(args=args)
    node = RLActionsNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
