from typing import List
import rclpy
from rclpy.node import Node
from board import SCL, SDA
from busio import I2C

from adafruit_pca9685 import PCA9685

from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray

from rcl_interfaces.msg import SetParametersResult

import math
import time
from busio import I2C

class Line_Follower(object):
    def __init__(self, i2c:I2C, address=0x11, references=[300, 300, 300, 300, 300]):
        self.i2c = i2c
        self.address = address
        self._references = references

    def read_raw(self):
        for i in range(0, 5):
            try:
                raw_result = bytearray(10)
                self.i2c.readfrom_into(self.address, raw_result, end=10)
                Connection_OK = True
                break
            except Exception as e:
                print(e)
                Connection_OK = False

        if Connection_OK:
            return raw_result
        else:
            print("Error accessing %2X" % self.address)
            return False

    def read_analog(self, trys=5):
        for _ in range(trys):
            raw_result = self.read_raw()
            if raw_result:
                analog_result = [0, 0, 0, 0, 0]
                for i in range(0, 5):
                    high_byte = raw_result[i*2] << 8
                    low_byte = raw_result[i*2+1]
                    analog_result[i] = high_byte + low_byte
                    if analog_result[i] > 1024:
                        continue
                return analog_result
        else:
            raise IOError("Line follower read error. Please check the wiring.")   

class Robot(Node):
    def __init__(self) -> None:
        super().__init__("robot")
        self.i2c = I2C(SCL, SDA)
        
        self.line_follower = Line_Follower(self.i2c)
        self.line_timer = self.create_timer(1/30, self.line_array)
        self.line_publisher = self.create_publisher(Int64MultiArray, '/lineArray', 10)

        self.servo_joint_names = ["bridge_latch_joint", "mechanism_lift_joint", "mechanism_package_joint", "mechanism_thruster_joint", "small_package_sweeper_joint"]
        params = [
            ('pca_address', 0x40),
            ('pca_frequency', 50)
        ]
        self.servo_mapping = {}
        
        for joint_name in self.servo_joint_names:
            params.append((f'servo_mapping.{joint_name}.port', 15))
            params.append((f'servo_mapping.{joint_name}.maximum', 1))
            params.append((f'servo_mapping.{joint_name}.minimum', 0))
            self.servo_mapping[joint_name] = {}
        
        self.declare_parameters(
            namespace='',
            parameters=params
        )
       
        self.PCA_ADDY = 0
        self.PCA_FREQ = 0
        self.load_parameters()
        self.connect_to_pca(self.i2c)
        
        self.get_logger().info(f'servo mapping: {self.servo_mapping}')
    
        self.add_on_set_parameters_callback(self.parameters_callback)
    
        self.joint_state_subscriber = self.create_subscription(JointState, "/grr/joint_command", self.joint_command_callback, 10)
    
    def parameters_callback(self, params):
        self.get_logger().info(f"{params}")
        self.load_parameters()
        return SetParametersResult(successful=True)
    
    def load_parameters(self):
        self.PCA_ADDY = self.get_parameter('pca_address').get_parameter_value().integer_value
        self.PCA_FREQ = self.get_parameter('pca_frequency').get_parameter_value().integer_value
        self.get_logger().info(f"ADDY: {self.PCA_ADDY} FREQ: {self.PCA_FREQ}")
        for joint_name in self.servo_joint_names:
            self.servo_mapping[joint_name]['port'] = self.get_parameter(f'servo_mapping.{joint_name}.port').get_parameter_value().integer_value
            self.servo_mapping[joint_name]['maximum'] = self.get_parameter(f'servo_mapping.{joint_name}.maximum').get_parameter_value().integer_value
            self.servo_mapping[joint_name]['minimum'] = self.get_parameter(f'servo_mapping.{joint_name}.minimum').get_parameter_value().integer_value

    def connect_to_pca(self, i2c:I2C):
        self.pca = PCA9685(i2c, address=self.PCA_ADDY)
        self.pca.frequency = self.PCA_FREQ
    
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
            
            self.get_logger().info(f"Spinning servo: {self.servo_mapping[name]['port']} to duty cycle: {duty_cycle} at frequency {self.pca.frequency}")

            self.pca.channels[self.servo_mapping[name]['port']].duty_cycle = int(duty_cycle)
            
    def line_array(self):
        vals = self.line_follower.read_analog()
        self.line_publisher.publish(Int64MultiArray(data=vals))
        

def main():
    rclpy.init()
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()