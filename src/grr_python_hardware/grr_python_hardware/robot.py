from grr_roboclaw.roboclaw import Roboclaw
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import yaml
from numpy import pi
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

from joints import SubState, Joint, Motor, Servo


class Robot(Node):
    def __init__(self):
        super().__init__("Robot")
        self.roboclaw = Roboclaw("/dev/ttyS0", 38400)
        self.roboclaw.Open()
        self.I2C = busio.I2C(SCL, SDA)
        self.PCAs = {}
        self.joints: dict[str, Joint] = {}
        self.state_publisher = self.create_publisher(JointState, "/grr/joint_states", 10)
        self.state_timer = self.create_timer(1/30, self.state_return_callback)
        self.refresh_hw_map = self.create_subscription(Bool, "/grr/refresh_hw_map", self.load_hw_map, 10)
        self.follow_trajectories = self.create_subscription(JointTrajectory, "/grr/command", self.joint_state_follow_subscriber, 10)
        
        
        
    def load_hw_map(self, data: Bool) -> None:
        if not data.data: return
        file_dir = get_package_share_directory('grr_python_hardware')
        with open(f"{file_dir}/hardware_map.yaml", "r") as f:
            data = yaml.load(f.read(), yaml.Loader)
            
        for joint in data["joints"]:
            
            if joint['name'] in self.joints.keys():
                self.get_logger().error("tried to register a joint that already exists")
                continue
            
            if joint["type"] == "motor":
                self.joints[joint['name']] = Motor(joint["name"], joint['params']['attachment'], joint['params']['rc_addy'], joint['params']['m1'], joint['params']['ticks_per_rev'], self.roboclaw)       
            elif joint["type"] == "servo":
                pca_addy = joint['params']['PCA_address']
                self.PCAs[pca_addy] = self.PCAs.get(pca_addy, PCA9685(self.I2C, address=pca_addy))
                print(self.PCAs)
                self.joints[joint['name']] = Servo(joint['name'], joint['params']['attachment'], joint['params']['servo_port'], joint['params']['maximum_value'], joint['params']['minimum_value'], self.PCAs[pca_addy])
                
        for k, v in self.joints.items():
            print(k, v)

    def joint_state_follow_subscriber(self, data: JointTrajectory) -> None:
        for i, name in enumerate(data.joint_names):
            res = self.joints.get(name, Joint(name)).follow_state(data.points[i], i)
            if not res:
                self.get_logger().error(f"Attempted to call an unregistered Joint: {name}")
                
    def state_return_callback(self) -> None:
        state = self.build_state_representation()
        self.state_publisher.publish(state)
        return state
        
                
    def build_state_representation(self) -> JointState:
        joint_state = JointState()
        for joint in self.joints.values():
            sub_state = joint.get_state()
            joint_state.name.append(sub_state.name)
            joint_state.position.append(sub_state.position)
            joint_state.velocity.append(sub_state.velocity)
            joint_state.effort.append(sub_state.effort)
            
        return joint_state
        

def main():
    rclpy.init()
    robot = Robot()
    robot.load_hw_map(Bool(data=True))
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()