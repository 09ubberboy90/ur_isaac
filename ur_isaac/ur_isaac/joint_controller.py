import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np

class JointController(Node):

    def __init__(self):
        super().__init__('baxter_joint_controller')
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.motor_pub = self.create_publisher(JointState, 't42_motor_control', 10)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states_sim',
            self.listener_callback,
            10)

        self.joint_states = {}
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.gripper_names = {
            "left": "swivel_1_to_finger_1_1",
            "right": "swivel_2_to_finger_2_1",
        }

    def listener_callback(self, msg:JointState):
        for name, pose in zip(msg.name, msg.position):
            self.joint_states[name] = pose
        
        out_msg = Float64MultiArray()
         
        for el in self.joint_names:
            out_msg.data.append(self.joint_states[el])
        
        self.publisher.publish(out_msg)
        outgoing = JointState()
        for key, el in self.gripper_names.items():
            outgoing.name.append(key)
            ## +0.88 is to handle the offset between the 0 position in the simulator and the 0 position in the real world
            outgoing.position.append((self.joint_states[el]+0.88)*180/np.pi)
        self.motor_pub.publish(outgoing)


def main(args=None):
    rclpy.init(args=args)

    node = JointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
