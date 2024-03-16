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


class StateMachine(Node):
    def __init__(self, dt):
        super().__init__('State_Machine')

        self.state = 0

        #===================================
        # Listeners:
        #===================================
        # Goal Listener
        self.goalSubscribers = self.create_subscription(Bool, "arrived_at_goal", self.check_goal, dt)

        # Green Light Listener
        self.greenSubscriber = self.create_subscription(Range,'sensor_msgs/msg/Range', self.green_light_callback, dt)


        
        
        #==========================================
        # Publishers
        #==========================================
        self.goal_Publisher = self.create_publisher(Pose, "goal", self.getGoal, dt)


        # I define a goal that can be modified throughout the program and sent when new goals are needed
        self.goal = Pose()
        self.goal.position.x = 0.0
        self.goal.position.y = 0.0
        self.currentPose.orientation.w = 0.0
        self.currentPose.orientation.x = 0.0
        self.currentPose.orientation.y = 0.0
        self.currentPose.orientation.z = 0.0



    # Checks to see if the light is green or not 
    def green_light_callback(self, msg):
        self.light = msg


def main():

    rclpy.init()
    Renwick = StateMachine(10)
    rclpy.spin(StateMachine)



if __name__ == '__main__':
    main()
