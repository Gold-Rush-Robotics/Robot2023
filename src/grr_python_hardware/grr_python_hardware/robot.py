from typing import List
import rclpy
from rclpy.node import Node
from busio import I2C

from adafruit_pca9685 import PCA9685

class Robot(Node):
    def __init__(self) -> None:
        super().__init__("robot", namespace="/grr/")
        
        self.links = ["link_name"]
        auto_gen_params = []
        for link in self.links:
            auto_gen_params.append((f'servo_mapping.{link}.port', 0))
            auto_gen_params.append((f'servo_mapping.{link}.maximum', 0xFFFF))
            auto_gen_params.append((f'servo_mapping.{link}.minimum', 0x0001))
        
        self.declare_parameters(
            namespace='/grr/',
            parameters=[
                ('pca_address', 0x40),
                ['pca_freq', 50]
            ].extend(auto_gen_params)
        )
        self.servo_mapping = {}
        self.PCA_ADDY = 0
        self.PCA_FREQ = 0
        self.load_parameters()
    
    def load_parameters(self):
        self.PCA_ADDY = self.get_parameter('/grr/pca_address').get_parameter_value().integer_value
        self.PCA_FREQ = self.get_parameter('/grr/pca_freq').get_parameter_value().integer_value
        for link in self.links:
            self.servo_mapping[link]['port'] = self.get_parameter(f'/grr/servo_mapping.{link}.port').get_parameter_value().integer_value
            self.servo_mapping[link]['maximum'] = self.get_parameter(f'/grr/servo_mapping.{link}.maximum').get_parameter_value().integer_value
            self.servo_mapping[link]['minimum'] = self.get_parameter(f'/grr/servo_mapping.{link}.minimum').get_parameter_value().integer_value

    def connect_to_pca(self, i2c:I2C):
        PCA9685(i2c, address=self.PCA_ADDY)

def main():
    rclpy.init()
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()