#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from numpy import arctan2

x = 0
y = 0
theta = 0
base_theta = 0

def newOdom(msg):
    global x
    global y
    global theta
    global base_theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    global base_theta

    if base_theta == 0:  # Set the initial orientation as the base direction
        base_theta = theta

rospy.init_node("speed_controller")
sub = rospy.Subscriber("/car/diff_drive_controller/odom", Odometry, newOdom)
pub = rospy.Publisher("/car/diff_drive_controller/cmd_vel", Twist, queue_size = 1)

speed = Twist()

goal = Point()
goal.x = 25
goal.y = 0

def goTo(goal):
    global x
    global y
    global theta
    global sub
    global pub
    global speed
    global base_theta

    arrived = False
    while not arrived and not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y - y
        angle_to_goal = arctan2(inc_y, inc_x)

        heading_error = base_theta - theta
        correction_factor = 0.5

        if abs(angle_to_goal - theta) > 0.25:
            speed.angular.z = (angle_to_goal - theta) * correction_factor
            speed.linear.x = 0.1 if speed.linear.x >= 0 else -0.1
        else:
            if speed.linear.x >= 0: 
                speed.linear.x = speed.linear.x + 0.01
            else:
                 speed.linear.x = speed.linear.x - 0.01
                 
            speed.angular.z = heading_error * correction_factor

        if abs(inc_x) <1:
            arrived = True
            print("destination reached!")
             
        pub.publish(speed)

goTo(goal)