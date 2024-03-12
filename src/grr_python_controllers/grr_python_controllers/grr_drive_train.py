import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Float32

class PIDcontroller:

    #Constructor
    def __init__(self, Kp, Ki, Kd, dt) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.dt = dt

    #update the control singal
    def update(self, error) -> float:
        
        #Porportional response
        P = self.Kp * error
        
        #Integral response
        self.integral += error * self.dt
        I = self.Ki * self.integral

        #Derivative response
        D = self.Kd * ((error - self.prev_error)/ self.dt)

        #Output
        control_signal = P + I + D

        #Update prev_error
        self.prev_error = error

        #return
        return control_signal




class DriveTrain(Node):
    
    #Constructor
    def __init__(self, tPID, rPID, dt) -> None:
        super().__init__('DriveTrain')

        # Define publishers
        self.velocity_publisher = self.create_publisher(Twist, "grr_cmake_controller/cmd_vel_unstamped", dt)
        # TODO: Does a publisher have to have a hz rate? Like can I just publish when I want? Will publish true for this constantly mess up future trips?
        self.arrival_publisher = self.create_publisher(Bool, "arrived_at_goal", dt)

        # Define subscribers
        self.odom_subscriber = self.create_subscription(Pose, "odom", self.getOdom, dt)
        self.goal_subscriber = self.create_subscription(Pose, "goal", self.getGoal, dt)

        # Define Current pose parameters
        self.currentPose = Pose()
        self.currentPose.position.x = 0.0
        self.currentPose.position.y = 0.0
        self.currentPose.orientation.w = 0.0
        self.currentPose.orientation.x = 0.0
        self.currentPose.orientation.y = 0.0
        self.currentPose.orientation.z = 0.0

        # Define goal parameter
        self.goal = Pose()
        self.goal.position.x = 0.0
        self.goal.position.y = 0.0
        self.currentPose.orientation.w = 0.0
        self.currentPose.orientation.x = 0.0
        self.currentPose.orientation.y = 0.0
        self.currentPose.orientation.z = 0.0

        # Initialize PID controller
        self.translateXPID = PIDcontroller(tPID[0], tPID[1], tPID[2], (dt * 0.01))
        self.translateYPID = PIDcontroller(tPID[0], tPID[1], tPID[2], (dt * 0.01))
        self.rotatePID = PIDcontroller(rPID[0], rPID[1], rPID[2], (dt * 0.01))


    # gets the current odometry of the robot
    def getOdom(self, odom):
        self.currentPose.position.x = odom.position.x
        self.currentPose.position.y = odom.position.y
        self.currentPose.orientation.w = odom.orientation.w
        self.currentPose.orientation.z = odom.orientation.z
        self.setVelocity()


    # Update goal of the drive train
    def getGoal(self, goal) -> None:


        
        if self.goal.position.x != goal.position.x or self.goal.position.y != goal.position.y or self.goal.orientation.z != goal.orientation.z:

            print("goal Recieved")
            
            arrival = Bool()
            arrival.data = False
            self.arrival_publisher.publish(arrival)

            # Get the linear goals
            self.goal.position.x = goal.position.x
            self.goal.position.y = goal.position.y

            # Get the rotational goals
            self.goal.orientation.w = goal.orientation.w
            self.goal.orientation.z = goal.orientation.z

    # sets the current planned x and y movement
    def setVelocity(self):

        min_val = 0.1
        min_rot = 0.5

        # Define the error (difference between where you are and where you want to be)
        x_error = self.goal.position.x - self.currentPose.position.x
        y_error = self.goal.position.y - self.currentPose.position.y

        # Get the angle error
        z_error = self.normalize_angle(self.quatToEuler(self.goal) - self.quatToEuler(self.currentPose))

        # Build the message
        velocity = Twist()
        velocity.linear.x = self.minimumfunction(self.translateXPID.update(x_error), min_val)
        velocity.linear.y = self.minimumfunction(self.translateYPID.update(y_error), min_val)
        velocity.angular.z = self.minimumfunction(self.rotatePID.update(z_error), min_rot)

        print(velocity)

        # Publish
        self.velocity_publisher.publish(velocity)

        # Check to see if we have arrived
        self.closeEnough(x_error, y_error, z_error)

    # Checks to see if the robot is close enough to the goal to cancel further movement
    def closeEnough(self, x_error, y_error, z_error):
        
        if (abs(x_error) <= 0.01) and (abs(y_error) <= 0.01) and (abs(z_error) <= 0.01):
            self.goal.position.x = self.currentPose.position.x
            self.goal.position.y = self.currentPose.position.y
            self.goal.orientation.w = self.currentPose.orientation.w
            self.goal.orientation.z = self.currentPose.orientation.z

            arrival = Bool()
            arrival.data = True
            self.arrival_publisher.publish(arrival)

    # Converts Quaternions to Euler angles
    # So we don't really care about most of the Quaternion because we are only going to rotate about the Z axis
    def quatToEuler(self, odom):
        QW = odom.orientation.w
        QZ = odom.orientation.z

        SyCp = 2 * (QW * QZ)
        CyCp = 1 - (2 * (QZ * QZ))

        angles = np.arctan2(SyCp, CyCp)


        return angles
    
    # Takes in a value and takes in a minimum 
    def minimumfunction(self, val, min):

        valsign = np.sign(val)

        val = abs(val)

        val = max(val, min)

        return valsign * val
    
    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))


def main():
    
    #Define PID values
    translatePID = [0.2, 0, 0]
    rotatePID = [0.2, 0, 0]
    
    rclpy.init()

    driveTrain = DriveTrain(translatePID, rotatePID, 10)
    rclpy.spin(driveTrain)

if __name__ == '__main__':
    main()


