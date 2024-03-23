import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10)
        self.subscription
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.roll = 0
        self.yaw = 0
        self.pitch = 0

    def listener_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.quaternion_to_euler(orientation_list)
    
    def timer_callback(self):
        self.get_logger().info('Orientation: Roll: %f, Pitch: %f, Yaw: %f' % (self.roll, self.pitch, self.yaw))

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()