#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

class RobotMover:
    def __init__(self):
        # Initialize the node
        rospy.init_node('robot_mover', anonymous=True)

        # Create a publisher to send velocity commands
        self.vel_pub = rospy.Publisher('/car/diff_drive_controller/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to get the robot's current position
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Current position of the robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Target position
        self.target_x = 0.0
        self.target_y = 0.0

        # Rate for the control loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        # Update the robot's current position based on odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Get the yaw from the quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = tf.transformations.euler_from_quaternion(orientation_list)

    def move_to_target(self):
        while not rospy.is_shutdown():
            # Calculate the distance and angle to the target
            distance = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
            angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)

            # Create a Twist message for velocity command
            command = Twist()

            # Check if the robot needs to rotate to face the target
            if abs(angle_to_target - self.current_yaw) > 0.1:  # 0.1 radians tolerance
                command.angular.z = 0.3 if angle_to_target > self.current_yaw else -0.3
            elif distance > 0.1:  # 0.1 meters tolerance
                command.linear.x = 0.5
            else:
                rospy.loginfo("Reached the target!")
                break

            # Publish the velocity command
            self.vel_pub.publish(command)

            # Wait for the next control loop iteration
            self.rate.sleep()

if __name__ == '__main__':
    robot_mover = RobotMover()
    try:
        robot_mover.move_to_target()
    except rospy.ROSInterruptException:
        pass
