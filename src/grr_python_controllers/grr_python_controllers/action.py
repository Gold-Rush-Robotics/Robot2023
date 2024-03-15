#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

#msg imports
from sensor_msgs.msg import Range, JointStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory 
from std_msgs.msg import Bool, Float32


class action:
    nextAction : action

    def __init__(self) -> None:
        pass

    def setNext(self, nextAction: action):
        nextAction.nextAction = self.nextAction
        self.nextAction = nextAction
        return self.nextAction
    
    def setLast(self, lastAction: action):
        if(self.nextAction):
            return self.nextAction.setLast(lastAction)
        else:
            self.nextAction = lastAction
            return self.nextAction
        
    def run(self):
        return self.nextAction