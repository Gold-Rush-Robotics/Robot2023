import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, '/grr_cmake_controller/cmd_vel_unstamped', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 1.0

    def timer_callback(self):
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Publishing: "%s"' % self.twist_msg.linear.x)

def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()