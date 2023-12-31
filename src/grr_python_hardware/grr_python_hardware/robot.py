from roboclaw_python_library.roboclaw_python.roboclaw_3 import Roboclaw
import rclpy
from rclpy.node import Node


class Robot(Node):
    def __init__(self):
        super().__init__("Robot")
        

