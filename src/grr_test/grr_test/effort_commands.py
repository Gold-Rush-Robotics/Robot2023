import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

class EffortCommandPublisher(Node):
    def __init__(self):
        super().__init__('effort_command_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.effort_cmd_msg = Float64MultiArray()
        self.effort_cmd_msg.data = [1.0,1.0]  # replace with your desired effort command

    def timer_callback(self):
        self.publisher_.publish(self.effort_cmd_msg)
        self.get_logger().info('Publishing: "%s"' % self.effort_cmd_msg.data)

def main(args=None):
    rclpy.init(args=args)
    effort_command_publisher = EffortCommandPublisher()
    rclpy.spin(effort_command_publisher)
    effort_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()