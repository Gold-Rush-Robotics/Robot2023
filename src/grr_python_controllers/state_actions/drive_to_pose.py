#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class DriveToPose(Action):
    def __init__(self, pose:Pose):
        super().__init__()
        
        self.goal_pose = pose

    def run(self, node):
        node.goal_pub.publish(self.goal_pose)
        if node.bools["goal_arrived"]:
            return self.nextAction
        return self
