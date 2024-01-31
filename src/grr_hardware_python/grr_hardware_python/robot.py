import rclpy

from board import SCL, SDA
import busio

from rclpy.node import Node
from std_msgs.msg import Bool
import adafruit_tcs34725


class Robot(Node):
    def __init__(self):
        super().__init__("grr_robot")
        self.start_light_timer = self.create_timer(.1, self.start_light_checker)
        self.start_light_publisher = self.create_publisher(Bool, "/grr/start_light", 10)
        self.declare_parameter('LED_Threshold', 255)
        self.i2c = busio.I2C(SCL, SDA)
        self.sensor = adafruit_tcs34725.TCS34725(self.i2c)
    
    def start_light_checker(self):
        threshold = self.get_parameter('LED_Threshold').get_parameter_value().integer_value
        color_rgb = self.sensor.color_rgb_bytes
        self.get_logger().info(f"Color: {color_rgb}")
        msg = Bool()
        msg.data = color_rgb[2] >= threshold
        self.start_light_publisher.publish(msg)

def main():
    rclpy.init()
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()