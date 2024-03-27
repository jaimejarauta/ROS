#!/usr/bin/python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import os

# Global variables for the car's position
x = 0.0

def model_states_callback(msg):
    global x
    # Assuming the car's index is 2 in the ModelStates message
    x = msg.pose[2].position.x

def clear_screen():
    os.system('clear')  # Use 'cls' for Windows

# Initialize the ROS node
rospy.init_node("speed_controller")

# Subscribe to the Gazebo model states topic
sub = rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

# Publisher for commanding the car's velocity
pub = rospy.Publisher("/car/diff_drive_controller/cmd_vel", Twist, queue_size=1)

# Function to move the car to the goal along the x-axis
def goTo(goal_x):
    global x, sub, pub

    # Define the Twist message for velocity command
    speed = Twist()
    arrived = False

    while not arrived and not rospy.is_shutdown():
        inc_x = goal_x - x

        clear_screen()
        print(f"Current x position: x={x:.3f}")

        # Control logic based on the current x position
        if inc_x > 0:
            speed.linear.x = 100  # Move forward
        else:
            speed.linear.x = -100  # Move backward

        # Check if the goal is reached
        if abs(inc_x) < 1:
            arrived = True
            speed.linear.x = 0  # Stop the car
            print("Destination reached!")

        # Publish the speed command
        pub.publish(speed)
        rospy.sleep(1)  # Sleep to allow for message processing

if __name__ == "__main__":
    goTo(0)  # Set the x-coordinate goal here
