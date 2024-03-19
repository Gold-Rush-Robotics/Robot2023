#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose, Vector3, Point
from std_msgs.msg import Bool

from state_actions.action import Action

import numpy as np

# This action is meant to take care of placing the April tags in the beginning

class DriveToGap(Action):
    def __init__(self, name:str=None):
        super().__init__()
        
        if name:
            self.name = name
            
        self.state = 0
        
        self.normal_readings = []        

    def run(self, node):
        reading = node.magnent_data.z
        node.drivetrain_enable.publish(Bool(data=False))
        
        node.get_logger().info(f"State: {self.state}, reading: {reading}")
        
        if node.tof_data["down"] >= 0.06:
            node.raw_cmd.publish(Twist())
            newPose = Pose(position=Point(x=1.14, y=node.position.position.y))
            node.new_pose.publish(newPose)
            node.goal_pub.publish(newPose)
            node.drivetrain_enable.publish(Bool(data=True))
            return self.nextAction
        
        if self.state >= 2:
            node.raw_cmd.publish(Twist(linear=Vector3(x=-0.05)))
        else:
            node.raw_cmd.publish(Twist(linear=Vector3(x=-0.4)))
            
        if self.state == 0:
            if reading >= 95.0:
                self.state = 1
        if self.state == 1:
            print("HELLO")
            if reading <= 94.0:
                print("this shouldnt print")
                self.state = 2
    
        return self
