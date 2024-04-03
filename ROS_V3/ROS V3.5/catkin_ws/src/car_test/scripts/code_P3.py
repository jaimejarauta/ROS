#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import display_info
import os

# Global variables for the car's position, speed, and orientation
x = 0.0
gazebo_speed = 0.0
initial_heading = None
current_yaw = 0.0

def model_states_callback(msg):
    global x, gazebo_speed, initial_heading, current_yaw
    # Assuming the car's index is 2 in the ModelStates message
    car_index = 2

    # Update the real Gazebo x position and speed
    x = msg.pose[car_index].position.x
    gazebo_speed = msg.twist[car_index].linear.x

    orientation_q = msg.pose[car_index].orientation
    _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    if initial_heading is None:
        initial_heading = current_yaw

def clear_screen():
    os.system('clear')  # Use 'cls' for Windows

# Initialize the ROS node
rospy.init_node("speed_controller")

# Subscribe to the Gazebo model states topic
rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

# Publisher for commanding the car's velocity
pub = rospy.Publisher("/car/diff_drive_controller/cmd_vel", Twist, queue_size=1)

# Function to move the car to the goal along the x-axis
def goTo(goal_x):
    global x, gazebo_speed, initial_heading, current_yaw 

    prev_direction = None
    speed_increment = 20
    current_speed = 20
    changing_direction = False

    max_x = -10000
    min_x = 10000

    speed = Twist()
    arrived = False

    while not arrived and not rospy.is_shutdown():
        
        clear_screen()

        display_info.display_info(0)
        print("\n")
        print(f"Current x Gazebo pos: x={x:.3f}")
        print(f"Current x Gazebo vel: vel_x_G={gazebo_speed:.3f}")
        print(f"Current x ROS Topic vel: vel_x_ROS={speed.linear.x:.3f}")
        print(f"Current yaw: {current_yaw:.3f}")
        print(f"Max_x = {max_x:.3f}  Min_x = {min_x:.3f}")

        if x > max_x:
            max_x = x

        if x < min_x:
            min_x = x

        # Determine current direction based on Gazebo speed
        current_direction = 'forward' if gazebo_speed >= 0 else 'backward'

        # Check if direction changed
        if current_direction != prev_direction:

            if prev_direction is not None:  # This ensures we skip the initial condition where prev_direction is None
                changing_direction = True
                current_speed = abs(current_speed) + speed_increment  # Increase the speed

                if current_direction == 'backward':
                    current_speed = -current_speed  # Change direction

            prev_direction = current_direction  # Update the prev_direction for the next iteration

        if changing_direction:
            speed.linear.x = current_speed
            changing_direction = False  # Reset the flag after applying the speed change
        else:
            # Maintain the current speed and direction if there is no change in direction
            speed.linear.x = current_speed 

        # Set speed based on current direction
        speed.linear.x = current_speed

        # Update prev_direction for the next iteration
        prev_direction = current_direction
        
        # Angular control to correct the orientation
        if initial_heading is not None:
            angular_correction = -3 * (current_yaw - initial_heading)
            speed.angular.z = angular_correction if abs(angular_correction) > 0.1 else 0

        # Check if the goal is reached
        if abs(goal_x - x) < 1:  # Assuming a small threshold for reaching the goal
            arrived = True
            speed.linear.x = 0  # Stop the car
            speed.angular.z = 0  # Stop turning
            print("Destination reached!")

        # Publish the speed command
        pub.publish(speed)
        rospy.sleep(0.1)  # Sleep to allow for message processing

if __name__ == "__main__":
    goTo(0)  # Set the x-coordinate goal here
