import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class MessageCounter(Node):
    def __init__(self):
        super().__init__('message_counter')
        self.subscription = self.create_subscription(
            JointState,
            '/drive_command',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.msg_count = 0

    def listener_callback(self, msg):
        self.msg_count += 1

def main(args=None):
    rclpy.init(args=args)

    message_counter = MessageCounter()

    start_time = time.time()
    while time.time() - start_time < 10:  # run for 10 seconds
        rclpy.spin_once(message_counter)

    print(f'Received {message_counter.msg_count} messages in 10 seconds')

    message_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()