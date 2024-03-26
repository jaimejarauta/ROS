#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import threading

# Global variables for speed control and acceleration direction
current_speed = 0.0
acceleration_rate = 0.04

def userInput():
    global current_speed, acceleration_rate
    while not rospy.is_shutdown():
        try:
            new_speed = float(input("Enter new linear x speed: "))
            # Determine the direction of acceleration based on the new_speed sign
            acceleration_rate = abs(acceleration_rate) if new_speed >= 0 else -abs(acceleration_rate)
            current_speed = new_speed
        except ValueError:
            print("Please enter a valid float number for speed.")

def accelerateCar(pub):
    global current_speed, acceleration_rate
    rate = rospy.Rate(10)  # 10Hz
    speed = Twist()

    while not rospy.is_shutdown():
        # Apply the acceleration rate
        if current_speed * acceleration_rate > 0:  # Check if they're in the same direction
            current_speed += acceleration_rate
        speed.linear.x = current_speed
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('speed_controller', anonymous=True)
        pub = rospy.Publisher('/car/diff_drive_controller/cmd_vel', Twist, queue_size=1)

        # Start a separate thread for user input
        input_thread = threading.Thread(target=userInput)
        input_thread.daemon = True  # Ensure the thread exits when the main program does
        input_thread.start()

        # Continue with the main acceleration control
        accelerateCar(pub)

    except rospy.ROSInterruptException:
        pass
