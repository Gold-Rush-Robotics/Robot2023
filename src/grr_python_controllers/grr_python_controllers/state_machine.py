#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

#msg imports
from sensor_msgs.msg import Range, JointStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory 

class StateMachine(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.state = 0

        #===================================
        # Listeners:
        #===================================
        # Green Light Listener
        self.greenSubscriber = self.create_subscription(Range,'sensor_msgs/msg/Range', self.green_light_callback, 10)
        self.greenSubscriber  # prevent unused variable warning

        # Joint States Listener
        self.greenSubscriber = self.create_subscription(JointStates,'sensor_msgs/msg/JointStates', self.jointStates_callback, 10)
        self.greenSubscriber  # prevent unused variable warning

        # Nav mag listener
        self.greenSubscriber = self.create_subscription(Odometry,'nav_msgs/Odometry', self.odometry_callback, 10)
        self.greenSubscriber  # prevent unused variable warning


        #==========================================
        # Publishers
        #==========================================
        
        self.jointState_publisher = self.create_publisher()
        
        self.pose_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.April_publisher = self.create_publisher()

        self.jointTrajectory_publisher = self.create_publisher()









    #===========================
    #Call back functions
    #===========================

    def green_light_callback(self, msg):
        if(msg == "GREEN"):
            self.state = 1
            return True
        
        else: 
            return False

    def jointStates_callback(self, msg):
        pass

    def odometry_callback(self, msg):
        pass


    def nextState(self):
        self.state = self.state + 1






def main():
    # Initialize State manchine

    stateMachine = StateMachine()


    #State Zero : Green Light

    stateMachine.nextState()

    #State One : April Tags
    # Step1: Place 
    StateMachine.April_publisher().publish(pass)
    # Step2: Push 
    msg = Twist()
    msg.linear.x = 
    StateMachine.pose_publisher().publish(msg)
    stateMachine.nextState()

    #State Two: Big Block Grab
    # Open Hand
    StateMachine.jointTrajectory_publisher().publish(pass)
    # Lower Arm
    StateMachine.jointTrajectory_publisher().publish(pass)
    # Close Hand
    StateMachine.jointTrajectory_publisher().publish(pass)
    # Raise Arm
    StateMachine.jointTrajectory_publisher().publish(pass)
    stateMachine.nextState()


    #State Three: Small Block Grab
    StateMachine.pose_publisher().publish(pass)
    stateMachine.nextState()

    #State Four: Go to the drop off zone
    StateMachine.pose_publisher().publish(pass)
    stateMachine.nextState()





    
    

    #State Twelve : Finish State / Clean up :)
    #TODO clean up





if __name__ == "__main__":
    main()

