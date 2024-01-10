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
import os


@dataclass
class SubState:
    name: str
    position: float
    velocity: float
    effort: float

class Joint:
    def __init__(self, name: str) -> None:
        self.name = name
    def get_state(self) -> SubState:
        return SubState("", 0, 0, 0)
    def follow_state(self, state: JointTrajectoryPoint, index: int) -> bool:
        return False
    def __str__(self) -> str:
        return f"Joint of type {__name__}"
    
class Motor(Joint):
    def __init__(self, name: str, attachment: int, rc_addy: int, m1: bool, ticks_per_rev: int, roboclaw: Roboclaw) -> None:
        super().__init__(name)
        self.attachment = attachment
        self.rc_addy = rc_addy
        self.m1 = m1
        self.roboclaw = roboclaw
        self.ticks_per_rev = ticks_per_rev
    
    def get_state(self) -> SubState:
        if self.ticks_per_rev != 0:
            position = self.roboclaw.ReadEncM1(self.rc_addy)[1] if self.m1 else self.roboclaw.ReadEncM2(self.rc_addy)[1]
            position = (position / self.ticks_per_rev) * 2 * pi
            velocity = self.roboclaw.ReadSpeedM1(self.rc_addy)[1] if self.m1 else self.roboclaw.ReadSpeedM2(self.rc_addy)[1]
        else:
            position = 0
            velocity = 0
        effort = self.roboclaw.ReadCurrents(self.rc_addy)[1 if self.m1 else 2]
        effort /= self.roboclaw.ReadM1MaxCurrent(self.rc_addy)[1] if self.m1 else self.roboclaw.ReadM2MaxCurrent(self.rc_addy)[1] 
        return SubState(self.name, position, velocity, effort)

    def __str__(self) -> str:
        return super().__str__() + f" named: {self.name} @ {self.rc_addy} {'m1' if self.m1 else 'm2'} on attachment {self.attachment} w/ {self.ticks_per_rev} t/r"
    
    def follow_state(self, state: JointTrajectoryPoint, index: int) -> bool:
        if self.ticks_per_rev != 0:
            buffer = ""
            self.roboclaw.SpeedAccelDistanceM1(self.rc_addy, state.accelerations[index], state.velocities[index], state.positions[index], buffer)
        else:
                            
            self.roboclaw.SpeedM1(self.rc_addy, state.effort[index]) if self.m1 else self.roboclaw.SpeedM2(self.rc_addy, state.effort[index])
            
        
        return True

class Robot(Node):
    def __init__(self):
        super().__init__("Robot")
        self.roboclaw = Roboclaw("/dev/ttyS0", 38400)
        self.roboclaw.Open()
        self.joints: dict[str, Joint] = {}
        self.state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.state_timer = self.create_timer(1/30, self.state_return_callback)
        self.refresh_hw_map = self.create_subscription(Bool, "/grr/refresh_hw_map", self.load_hw_map, 10)
        self.follow_trajectories = self.create_subscription(JointTrajectory, "/grr/command", self.joint_state_follow_subscriber, 10)
        
    def load_hw_map(self, data: Bool) -> None:
        if not data.data: return
        file_dir = get_package_share_directory('grr_python_hardware')
        with open(f"{file_dir}/hardware_map.yaml", "r") as f:
            data = yaml.load(f.read(), yaml.Loader)
            
        for joint in data["joints"]:
            if joint["type"] == "motor":
                self.joints[joint['name']] = Motor(joint["name"], joint['params']['attachment'], joint['params']['rc_addy'], joint['params']['m1'], joint['params']['ticks_per_rev'], self.roboclaw)       
        
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