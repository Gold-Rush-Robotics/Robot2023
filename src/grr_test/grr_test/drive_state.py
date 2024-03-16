import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
import numpy as np

class DriveState(Node):
    def __init__(self):
        super().__init__('drive_state')
        self.joint_state_pub = self.create_publisher(JointState, 'drive_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['front_left_mecanum_joint', 'front_right_mecanum_joint', 'rear_left_mecanum_joint', 'rear_right_mecanum_joint']
        msg.position = [0.0, 0.0, 0.0, 0.0]
        msg.velocity = [np.random.normal(3.14, .1),np.random.normal(3.24, .1),np.random.normal(3.14, .1),np.random.normal(3.14, .1)]
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    drive_state = DriveState()
    rclpy.spin(drive_state)
    drive_state.destroy_node()
    rclpy.shutdown()

