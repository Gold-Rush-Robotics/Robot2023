from roboclaw_python_library.roboclaw_python.roboclaw_3 import Roboclaw
import rclpy
from rclpy.node import Node

class Robot(Node):
    def __init__(self):
        super().__init__("Robot")
        self.roboclaw = Roboclaw("/dev/ttyS0", 38400)
        self.roboclaw.Open()

        
def main():
    rclpy.init()
    robot = Robot()
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()