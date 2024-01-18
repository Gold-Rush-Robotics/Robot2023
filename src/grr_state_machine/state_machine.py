#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

#msg imports
from sensor_msgs import Range

class StateMachine():
    def __init__(self):
        startLight = False
        GreenLight = GreenLightSubscriber()

class GreenLightSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Range,'sensor_msgs/Range', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if(msg == "GREEN"):
            return True
        
        else: 
            return False















if __name__ == "__main__":
    
    # Initialize State manchine

    stateMachine = StateMachine()
    currentState = 0


    #State Zero
    while (StateMachine.startLight == False):
        stateMachine.startLight = stateMachine.GreenLightSubscriber


    



