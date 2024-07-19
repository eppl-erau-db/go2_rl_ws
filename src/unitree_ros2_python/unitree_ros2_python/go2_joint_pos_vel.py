#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unitree_go.msg import LowState


class JointPosVelNode(Node):
    def __init__(self):
        super().__init__("state_publisher")
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'joint_pos_vel',
            10
        )
        self.subscription = self.create_subscription(
            LowState,
            'lowstate',
            self.lowstate_callback,
            10
        )

    def lowstate_callback(self, msg):
        '''
        TODO: May have to normalize for concatenation with obs term.
        '''
        motor_qs = [
            msg.motor_state[3].q,   # 0  -> FL_hip_joint   to FL_hip   -> 3
            msg.motor_state[0].q,   # 1  -> FR_hip_joint   to FR_hip   -> 0
            msg.motor_state[9].q,   # 2  -> RL_hip_joint   to RL_hip   -> 9
            msg.motor_state[6].q,   # 3  -> RR_hip_joint   to RR_hip   -> 6
            msg.motor_state[4].q,   # 4  -> FL_thigh_joint to FL_thigh -> 4
            msg.motor_state[1].q,   # 5  -> FR_thigh_joint to FR_thigh -> 1
            msg.motor_state[10].q,  # 6  -> RL_thigh_joint to RL_thigh -> 10
            msg.motor_state[7].q,   # 7  -> RR_thigh_joint to RR_thigh -> 7
            msg.motor_state[5].q,   # 8  -> FL_calf_joint  to FL_calf  -> 5
            msg.motor_state[2].q,   # 9  -> FR_calf_joint  to FR_calf  -> 2
            msg.motor_state[11].q,  # 10 -> RL_calf_joint  to RL_calf  -> 11
            msg.motor_state[8].q,   # 11 -> RR_calf_joint  to RR_calf  -> 8
        ]

        motor_dqs = [
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
       
        # Adjusting to training defaults
        motor_qs_defaults = [
            0.0,
            -0.0,
            0.0,
            -0.0,
            1.1,
            1.1,
            1.1,
            1.1,
            -1.8,
            -1.8,
            -1.8,
            -1.8,
        ]

        # Subtracting defaults
        joint_pos = [q - default for q,
                     default in zip(motor_qs, motor_qs_defaults)]

        # Publishing the array of joint positions
        joint_pos_vel_msg = Float32MultiArray()
        joint_pos_vel_msg.data = joint_pos + motor_dqs
        self.publisher.publish(joint_pos_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointPosVelNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
