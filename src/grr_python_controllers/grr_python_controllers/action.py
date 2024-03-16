#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from __future__ import annotations



#msg imports
from sensor_msgs.msg import Range, JointStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory 
from std_msgs.msg import Bool, Float32


class Action:
    nextAction : Action
    name: str

    def __init__(self) -> None:
        name = self.__qualname__
        nextAction = Action()

    def setNext(self, nextAction: Action):
        nextAction.nextAction = self.nextAction
        self.nextAction = nextAction
        return self.nextAction
    
    def setLast(self, lastAction: Action):
        if(self.nextAction):
            return self.nextAction.setLast(lastAction)
        else:
            self.nextAction = lastAction
            return self.nextAction
        
    def run(self, node:Node):
        return self