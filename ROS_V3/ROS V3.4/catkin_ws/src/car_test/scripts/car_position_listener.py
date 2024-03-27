#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
import os

def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')

def callback(data):
    if 'car' in data.name:
        index = data.name.index('car')
        position = data.pose[index].position
        clear_screen()
        print(f"Car position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}")

def listener():
    rospy.init_node('car_position_listener', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
