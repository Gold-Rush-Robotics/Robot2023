#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist


class WayPoint(Node):
    def __init__(self):
        super().__init__('way_point_test')

        self.pose_publisher = self.create_publisher(Twist, 'cmd_vel', 10)



def main(args=None):
    rclpy.init(args=args)

    msg = Twist()

    msg.linear.x = 5.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0



    waypointer = WayPoint()

    rclpy.spin(waypointer)



if __name__ == "__main__":
    main()