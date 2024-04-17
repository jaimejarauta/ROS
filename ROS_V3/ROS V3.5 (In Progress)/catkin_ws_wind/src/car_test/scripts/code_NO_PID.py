#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from numpy import arctan2

x = 0
y = 0
theta = 0

def newOdom(msg):
    global x

    x = msg.pose.pose.position.x

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")
rate = rospy.Rate(50)  # 10 Hz
sub = rospy.Subscriber("/car/diff_drive_controller/odom", Odometry, newOdom)
pub = rospy.Publisher("/car/diff_drive_controller/cmd_vel", Twist, queue_size = 1)

speed = Twist()

goal = Point()
goal.x = 40
goal.y = 0

def goTo(goal):
    global x
    global sub
    global pub
    global speed

    arrived = False
    while not arrived:
        inc_x = goal.x -x

        if inc_x > 0:
            speed.linear.x = speed.linear.x + 0.04
        else:
            speed.linear.x = speed.linear.x - 0.04

        if abs(inc_x) <1:
        	arrived = True
        	print("destination reached!")
        pub.publish(speed)


goTo(goal)
a = int(input("want to go back ? (0/1)  "))
if (a == 1):
    goal.x = 0
    goal.y = 0
    goTo(goal)
