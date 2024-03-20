#!/usr/bin/python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import Range, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory 
from std_msgs.msg import Bool, Float32

from typing import Callable

class Action:
    nextAction : Action
    name: str
    def __init__(self, function: Callable = None) -> None:
        self.name = self.__class__.__name__
        self.nextAction = self
        self.function = function

    def setNext(self, nextAction: Action):
        nextAction.nextAction = self.nextAction
        self.nextAction = nextAction
        return self.nextAction
    
    def setLast(self, lastAction: Action):
        if(self.nextAction) and (self.nextAction != self):
            return self.nextAction.setLast(lastAction)
        else:
            self.nextAction = lastAction
            return self.nextAction
        
    def run(self, node:Node):
        if self.function:
            self.function()
            return self.nextAction
        return self