#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose, Vector3, Point
from std_msgs.msg import Bool

from state_actions.action import Action
from state_actions.corner_reset import CornerReset

import numpy as np
class DriveToGap(Action):
    def __init__(self, name:str=None):
        super().__init__()
        
        if name:
            self.name = name
            
        self.state = 0
        
        # state 0 == Flat
        # State 1 == hill going up
        # state 2 == Flat on hill
        # state 3 == over gap
        
        self.normal_readings = []     
        self.timesThroughState = 0
        
    def reset_on_edge(self, node): # >= 0.06
        node.raw_cmd.publish(Twist())
        newPose = Pose(position=Point(x=1.18, y=node.position.position.y))
        self.setNext(CornerReset(newPose))

    def run(self, node):
        reading = node.tof_data["down"]
        node.drivetrain_enable.publish(Bool(data=False))
        
        node.get_logger().info(f"State: {self.state}, reading: {reading}")
        
        speed = -0.05
        
        match(self.state):
            case 0: #flat pre hill
                speed = -0.45
                if reading <= 0.008:
                    self.state = 1
            case 1:
                speed = -0.45
                if reading >= 0.018:
                    print("OVER THE EDGE")
                    self.state = 2
            case 2: # flat top of hill
                speed = -0.1 - ((30 - self.timesThroughState)) * 0.01
                self.timesThroughState += 1
                if self.timesThroughState >= 30:
                    self.state = 3
            case 3:
                if reading >= 0.015:
                    speed = 0.0
                    self.reset_on_edge(node)
                    return self.nextAction
        
        
        
        node.raw_cmd.publish(Twist(linear=Vector3(x=speed)))
    
        return self
