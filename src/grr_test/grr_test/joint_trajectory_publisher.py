import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_trajectory)

    def publish_joint_trajectory(self):
        joint_trajectory = JointTrajectory()

        # Set joint names
        joint_trajectory.joint_names = ['small_package_grabber_roller_1_joint','small_package_sweeper_roller_joint']

        # Create a JointTrajectoryPoint
        point = JointTrajectoryPoint()

        # Set position individually
        point.velocities = [0.5, 0.5]
        point.positions = [0.5, 0.5]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0


        # Add the point to the trajectory
        joint_trajectory.points = [point]

        self.publisher_.publish(joint_trajectory)
        self.get_logger().info('Published JointTrajectory message')

def main(args=None):
    rclpy.init(args=args)

    joint_trajectory_publisher = JointTrajectoryPublisher()

    rclpy.spin(joint_trajectory_publisher)

    # Destroy the node explicitly
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()