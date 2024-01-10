from grr_roboclaw.roboclaw import Roboclaw
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import yaml
import os

@dataclass
class SubState:
    name: str
    position: float
    velocity: float
    effort: float

class Joint:    
    def get_state(self) -> SubState:
        pass
    def follow_state(self, state: JointTrajectoryPoint) -> bool:
        return False

class Robot(Node):
    def __init__(self):
        super().__init__("Robot")
        self.roboclaw = Roboclaw("/dev/ttyS0", 38400)
        self.roboclaw.Open()
        self.joints: dict[str, Joint] = {}
        
    def load_hw_map(self):
        file_dir = get_package_share_directory('grr_python_hardware')

        print(file_dir)
        with open(f"{file_dir}/hardware_map.yaml", "r") as f:
            data = yaml.load(f.read(), yaml.Loader)
            
            print(data)
        
    def joint_state_follow_subscriber(self, data: JointTrajectory) -> None:
        for i, name in enumerate(data.joint_names):
            res = self.joints.get(name, Joint()).follow_state(data.points[i])
            if not res:
                self.get_logger().error(f"Attempted to call an unregistered Joint: {name}")
                
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
    robot.load_hw_map()
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()