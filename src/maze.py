#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Function is called every time there's a scan
# global keyword is needed so g_range_ahead is available outside of the function
def scan_callback(msg):
    global g_range_ahead
    # get the elements w/in a certain range (30 degrees each direction)
    angle_range_ahead = msg.ranges[0:30] + msg.ranges[-30:]

    angle_range_right = msg.ranges[59:119]
    angle_range_left = msg.ranges[226:286]

    print(len(angle_range_ahead))  # if we are getting all the indices we want


    g_range_ahead = min(angle_range_ahead)

# Main program
g_range_ahead = 1 # anything to start

# Declare a subscriber to message 'scan' with message class LaserScan
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Create a Publisher object. queue_size=1 means that messages that are
# published but not handled by received are lost beyond queue size.
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Initialize this program as a node
rospy.init_node('maze')

forward_twist = Twist()
forward_twist.linear.x = 0.22
right_twist = Twist()
right_twist.angular.z = 1
left_twist = Twist()
left_twist.angular.z = -1

# if driving_forward = 1 - forward, 0 - left, 2 - right
driving_direction = 1

# if starting out, go forward
starting_out = True

# rate object gets a sleep() method which will sleep 1/200 seconds
rate = rospy.Rate(20)

# starting out, if no wall, go straight then turn right

while not rospy.is_shutdown():
    if starting_out:
        starting_out = False
        while(g_range_ahead >= 0.3):
            cmd_vel_pub.publish(forward_twist)
        # then turn right
        driving_direction = 2

    
    else: # not starting out
        if driving_forward == 1:
            if can_turn_left:
                #turn left
                driving_forward = 0
            
            if g_range_ahead < 0.3:
                print("bumped into object! Going to turn")
                if can_turn_left:
                    #turn left
                    driving_direction = 0
                else:
                    #turn right
                    driving_direction = 2

            else:
                print("moving forward", forward_twist.linear.x)
        else: # Rotating!
            print("rotating")

            if g_range_ahead < 0.3:
                print("keep rotating!")
            else: #Drive forward!
                print("driving forward")
                driving_direction = 1

        if driving_direction == 1:
            cmd_vel_pub.publish(forward_twist)
        elif driving_direction == 0:
            cmd_vel_pub.publish(left_twist)
        elif driving_direction == 2:
            cmd_vel_pub.publish(right_twist)
    rate.sleep()
