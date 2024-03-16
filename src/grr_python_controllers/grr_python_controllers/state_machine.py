#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

#msg imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point
from trajectory_msgs.msg import JointTrajectory 
from std_msgs.msg import Bool, Float32

from collections import defaultdict

from state_actions.wait_for_start import WaitForStart
from state_actions.drive_to_pose import DriveToPose
from state_actions.action import Action


class StateMachine(Node):
    def __init__(self):
        super().__init__('State_Machine')
        
        self.action_tree = WaitForStart()
        self.action_tree.setNext(
            DriveToPose(Pose(position=Point(y=0.1)))
        ).setNext(
            Action()
        )

        self.state = 0

        #===================================
        # Listeners:
        #===================================
        # Goal Listener
        self.goal_arrival_sub = self.create_subscription(Bool, "arrived_at_goal", lambda x: self.bool_callback(x, "goal_arrived"), 10)

        self.start_light_sub = self.create_subscription(Bool,'/grr/start_light', lambda x: self.bool_callback(x, "start_light"), 10)
        
        self.gui_start_sub = self.create_subscription(Bool, '/gui/start', lambda x: self.bool_callback(x, "gui_start"), 10)

        #==========================================
        # Publishers
        #==========================================
        self.goal_pub = self.create_publisher(Pose, "/drivetrain/goal", 10)

        # I define a goal that can be modified throughout the program and sent when new goals are needed
        self.goal = Pose()
        
        self.bools = defaultdict(lambda: False)
        
        self.runner_timer = self.create_timer(1/30.0, self.runner)
        
        self.current_action_name = ""
        
    def runner(self):
        self.action_tree = self.action_tree.run(self)
        if self.action_tree.name != self.current_action_name:
            self.get_logger().info(f"Starting: {self.action_tree.name}")
            self.current_action_name = self.action_tree.name
        
    def bool_callback(self, msg:Bool, name:str):
        self.bools[name] = msg.data


def main():
    rclpy.init()
    Renwick = StateMachine()
    rclpy.spin(Renwick)



if __name__ == '__main__':
    main()
