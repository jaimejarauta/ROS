#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

# Global variables to store the latest data
gazebo_speed = 0.0
gazebo_x_pos = 0.0
ros_topic_speed = 0.0
ros_topic_x_pos = 0.0

def gazebo_state_callback(msg):
    global gazebo_speed, gazebo_x_pos
    # Assuming the car model is the third in the list (index 2)
    car_index = 2
    gazebo_speed = round(msg.twist[car_index].linear.x, 3)
    gazebo_x_pos = round(msg.pose[car_index].position.x, 3)

def odom_callback(msg):
    global ros_topic_speed, ros_topic_x_pos
    ros_topic_speed = round(msg.twist.twist.linear.x, 3)
    ros_topic_x_pos = round(msg.pose.pose.position.x, 3)

def display_info(event):
    print(f"Real Gazebo Speed: {gazebo_speed} m/s")
    print(f"Real Gazebo X Position: {gazebo_x_pos} m")
    print(f"ROS Topic Speed: {ros_topic_speed} m/s")
    print(f"ROS Topic X Position: {ros_topic_x_pos} m")
    print("\n")

def main():
    rospy.init_node('info_display')

    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_state_callback)
    rospy.Subscriber("/car/diff_drive_controller/odom", Odometry, odom_callback)

    # Timer to refresh the display every 0.5 seconds
    rospy.Timer(rospy.Duration(0.5), display_info)

    rospy.spin()  # This will keep the script running indefinitely

if __name__ == '__main__':
    main()
