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


class Action:
    nextAction : Action

    def __init__(self) -> None:
        pass

    def setNext(self, nextAction: Action):
        nextAction.nextAction = self.nextAction
        self.nextAction = nextAction
        return self.nextAction