#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

# Global variables to store the latest data from Gazebo and ROS for the car's speed and position
gazebo_speed = 0.0
gazebo_x_pos = 0.0
ros_topic_speed = 0.0
ros_topic_x_pos = 0.0

def gazebo_state_callback(msg):
    global gazebo_speed, gazebo_x_pos
    # Extract speed and position data for the car from the ModelStates message provided by Gazebo
    car_index = 2  # The index of the car in the ModelStates array
    gazebo_speed = round(msg.twist[car_index].linear.x, 3)  # Car's speed from Gazebo
    gazebo_x_pos = round(msg.pose[car_index].position.x, 3)  # Car's position from Gazebo

def odom_callback(msg):
    global ros_topic_speed, ros_topic_x_pos
    # Extract speed and position data from the Odometry message provided by a ROS topic
    ros_topic_speed = round(msg.twist.twist.linear.x, 3)  # Car's speed from ROS topic
    ros_topic_x_pos = round(msg.pose.pose.position.x, 3)  # Car's position from ROS topic

def display_info(event):
    # Display the car's speed and position from both Gazebo (physics adjusted) and ROS (no physics applied)
    print(f"Real Gazebo Speed: {gazebo_speed} m/s")
    print(f"Real Gazebo X Position: {gazebo_x_pos} m")
    print(f"ROS Topic Speed: {ros_topic_speed} m/s")
    print(f"ROS Topic X Position: {ros_topic_x_pos} m")
    print("\n")

def main():
    rospy.init_node('info_display')

    # Subscribe to Gazebo's model states and ROS's odometry data
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_state_callback)
    rospy.Subscriber("/car/diff_drive_controller/odom", Odometry, odom_callback)

    # Timer to call display_info every 0.5 seconds to refresh the displayed data
    rospy.Timer(rospy.Duration(0.5), display_info)

    rospy.spin()  # Keep the script running and responding to data

if __name__ == '__main__':
    main()
