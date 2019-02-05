#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi


turn_speed = .3
forward_speed = 0.22
wall_thresh = 0.3
# Function is called every time there's a scan
# global keyword is needed so g_range_ahead is available outside of the function
def scan_callback(msg):
    global g_range_ahead
    global g_range_left
    # get the elements w/in a certain range (30 degrees each direction)

    # TODO list comprehensions to replaces bad data (0's) with ok data ( >= 3.5)
    angle_range_ahead = msg.ranges[0:30] + msg.ranges[-30:]
    angle_range_left = msg.ranges[226:286]

    g_range_ahead = min(angle_range_ahead)
    g_range_left = min(angle_range_left)


# turns a robot an input number of degrees to the right, (default 90)
def turn_right(cmd_pub, angle=90):
    start_time = rospy.Time.now().secs
    turn_time = angle*pi/180 / turn_speed
    while(rospy.Time.now().secs - turn_time < start_time):
        turn_twist = Twist()
        turn_twist.angular.z =  -turn_speed
        cmd_pub.publish(turn_twist)


# turns left a given number of degrees (default 90) then moves forward a little but (default 1 second)
def turn_left_and_go_a_little(cmd_pub, angle=90, x_time=1):
    start_time = rospy.Time.now().secs
    turn_time = angle*pi/180 / turn_speed
    while(rospy.Time.now().secs - turn_time < start_time):
        turn_twist = Twist()
        turn_twist.angular.z =  turn_speed
        cmd_pub.publish(turn_twist)

    start_time = rospy.Time.now().secs
    while(rospy.Time.now().secs - x_time < start_time and g_range_ahead > wall_thresh):
        forward_twist = Twist()
        forward_twist.linear.x = forward_speed
        cmd_pub.publish(forward_twist)



# Main program
g_range_ahead = 1 # anything to start
g_range_left = 1

# Declare a subscriber to message 'scan' with message class LaserScan
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Create a Publisher object. queue_size=1 means that messages that are
# published but not handled by received are lost beyond queue size.
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Initialize this program as a node
rospy.init_node('maze')

forward_twist = Twist()
forward_twist.linear.x = forward_speed

# rate object gets a sleep() method which will sleep 1/200 seconds
rate = rospy.Rate(10)


while not rospy.is_shutdown():
    # condition 1: can turn left
    if g_range_ahead > wall_thresh and g_range_left > wall_thresh:
        turn_left_and_go_a_little(cmd_vel_pub)
    # condition 2: can't turn left
    elif g_range_ahead > wall_thresh:
        cmd_vel_pub.publish(forward_twist)
    # condition 3: can't go straight
    else:
        turn_right(cmd_vel_pub)

    rate.sleep()
