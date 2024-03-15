#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose

import action


# This action is meant to take care of placing the April tags in the beginning

class step_one(action):
    def __init__(self):
        super().__init__()

    

    def run(self, Machine: Node):
        while Machine.light != "GREEN":
            x = 1 + 1

        Machine.goal.position.y = 5
        Machine.goal_Publisher.publish(Machine.goal)

        
