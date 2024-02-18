#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def set_velocity():
    # Initialize the ROS node
    rospy.init_node('ambulance_velocity_controller', anonymous=True)

    # Publisher to send commands to 'cmd_vel'
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Create a Twist message
        vel_msg = Twist()

        # Set linear and angular velocities
        vel_msg.linear.x = 2.0  # Forward velocity
        vel_msg.angular.z = 0.0  # Rotational velocity

        # Publish the message
        pub.publish(vel_msg)

        # Wait until the next cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        set_velocity()
    except rospy.ROSInterruptException:
        pass
