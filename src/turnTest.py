#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from math import pi

# Function is called every time there's a scan
# Saves the smallest distance, which would be the closest obstacle
# This is without regard to the direction of the obstacle!
# global keyword is needed so g_range_ahead is available outside of the function
def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)


def angle_callback(msg):
    global z_angle
    z_angle = msg.data

# Main program
g_range_ahead = 1 # anything to start
z_angle = 1

# Declare a subscriber to message 'scan' with message class LaserScan
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# subscriber to the angle_node which converets odom quarternions to euler angles
angle_sub = rospy.Subscriber('/angle', Float64, angle_callback)

# Same code can be a publisher and a subscriber, this is no problem
# be ready to publish the cmd_vel topic of class Twist
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Declare us to be a node
rospy.init_node('turnTest')
state_change_time = rospy.Time.now()

# driving_forward: forward(true) vs. spin inplace (false)
#   TRUE: until x seconds pass or get close to an obstacle, go forward
#   FALSE: until y seconds pass, spin in place
driving_forward = True
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    print("***")
    print(rospy.Time.now().secs, rospy.Time.now().nsecs)
    print(state_change_time.secs, state_change_time.nsecs)
    print(rospy.Time.now() > state_change_time, g_range_ahead, driving_forward)

    # check whether antyhing is closer than x meters or
    # time for driving foreward is over, then start spinning in place
    if driving_forward:
        if (g_range_ahead < 0.2 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(3)
            start_angle = z_angle

    # check whether time to spin is over, then go back to driving
    else: # we're not driving_forward
        if z_angle > start_angle + 1.57:
            driving_forward = True # we're done spinning, time to go forward!
            state_change_time = rospy.Time.now() + rospy.Duration(30)

    # start_angle = odom_angle
    # turn_twist = Twist()
    # turn_twist.angular.z = -1
    # cmd_pub.publish(turn_twist)
    # while start_angle - 1.57 < odom_angle:
    #     continue
    #     print("start_angle", start_angle, " odom_angle ", odom_angle)
    # turn_twist.angular.z = 0
    # cmd_pub.publish(turn_twist)

    # Create an all zero Twist() message. Note a new one is created each loop
    twist = Twist()

    # Depending on whether we are driving foreward, set linear or angular
    if driving_forward:
        twist.linear.x = 0.2
    else:
        twist.angular.z = 1.0

    # Publish cmd_vel with the desired motion
    cmd_vel_pub.publish(twist)

    # Sleep for 1/rate seconds
    rate.sleep()
