from typing import List
import rclpy
from rclpy.node import Node
from board import SCL, SDA
from busio import I2C

from adafruit_pca9685 import PCA9685

from sensor_msgs.msg import JointState

class Robot(Node):
    def __init__(self) -> None:
        super().__init__("robot")
        self.i2c = I2C(SCL, SDA)

        self.links = ["link_name"]
        params = [
            ('pca_address', 0x40),
            ('pca_frequency', 50)
        ]
        self.servo_mapping = {}
        
        for link in self.links:
            params.append((f'servo_mapping.{link}.port', 0))
            params.append((f'servo_mapping.{link}.maximum', 0xFFFF))
            params.append((f'servo_mapping.{link}.minimum', 0x0001))
            self.servo_mapping[link] = {}
        
        self.declare_parameters(
            namespace='',
            parameters=params
        )
       
        self.PCA_ADDY = 0
        self.PCA_FREQ = 0
        self.load_parameters()
        self.connect_to_pca(self.i2c)
        
        self.get_logger().info(f'servo mapping: {self.servo_mapping}')
        
        self.joint_state_subscriber = self.create_subscription(JointState, "joint_command", self.joint_command_callback, 10)
    
    def load_parameters(self):
        self.PCA_ADDY = self.get_parameter('pca_address').get_parameter_value().integer_value
        self.PCA_FREQ = self.get_parameter('pca_frequency').get_parameter_value().integer_value
        for link in self.links:
            self.servo_mapping[link]['port'] = self.get_parameter(f'servo_mapping.{link}.port').get_parameter_value().integer_value
            self.servo_mapping[link]['maximum'] = self.get_parameter(f'servo_mapping.{link}.maximum').get_parameter_value().integer_value
            self.servo_mapping[link]['minimum'] = self.get_parameter(f'servo_mapping.{link}.minimum').get_parameter_value().integer_value

    def connect_to_pca(self, i2c:I2C):
        self.pca = PCA9685(i2c, address=self.PCA_ADDY)
    
    def joint_command_callback(self, data:JointState):
        for i, name in enumerate(data.name):
            self.get_logger().info(f'link name: {name} VS stored: {self.servo_mapping.keys()}')
            if name not in self.servo_mapping.keys():
                self.get_logger().info('Skipped')
                continue
            effort = data.effort[i]
            old_min = 0
            old_max = 100
            old_range = old_max - old_min
            new_min = self.servo_mapping[name]['minimum']
            new_max = self.servo_mapping[name]['maximum']
            new_range = new_max-new_min
                        
            duty_cycle = (((effort - old_min) * new_range)/old_range)+new_min
            
            self.get_logger().info(f"Spinning servo: {self.servo_mapping[name]['port']} to duty cycle: {duty_cycle}")

            
            self.pca.channels[self.servo_mapping[name]['port']].duty_cycle = duty_cycle

def main():
    rclpy.init()
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()