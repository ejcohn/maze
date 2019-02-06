#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi

global wall_thresh
turn_speed = .3
forward_speed = 0.22
wall_thresh = 0.3
# Function is called every time there's a scan
# global keyword is needed so g_range_ahead is available outside of the function
def scan_callback(msg):
    global g_range_ahead
    global g_range_left
    # get the elements w/in a certain range (30 degrees each direction)

    angle_range_ahead = msg.ranges[0:30] + msg.ranges[-30:]
    angle_range_left = msg.ranges[226:286]
    # replace infinity readings (0.0) w/ a real number (4)
    angle_range_ahead = [4 if x == 0.0 else x for x in angle_range_ahead]
    angle_range_left = [4 if x == 0.0 else x for x in angle_range_left]

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

def can_turn_left():
    if g_range_left > wall_thresh + .22:
        return True
    else:
        return False

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
right_twist = Twist()
right_twist.angular.z = 1
left_twist = Twist()
left_twist.angular.z = -1
wall_follow = Twist()
wall_follow.linear.x = forward_speed

# if driving_forward = 1 - forward, 0 - left, 2 - right
driving_direction = 1

# if starting out, go forward
starting_out = True

# rate object gets a sleep() method which will sleep 1/200 seconds
rate = rospy.Rate(10)

state_change_time = 0

# starting out, if no wall, go straight then turn right

while not rospy.is_shutdown():
    if starting_out:
        starting_out = False
        while(g_range_ahead >= wall_thresh):
            cmd_vel_pub.publish(forward_twist)
        # then turn right
        turn_right(cmd_vel_pub)
        driving_direction = 2


    else: # not starting out
        if driving_forward == 1:
            # drive forward
            wall_follow.angular.z = 0 + (g_range_left - wall_thresh)/1.5
            cmd_vel_pub.publish(wall_follow)
            if can_turn_left():
                #turn left
                turn_left_and_go_a_little(cmd_vel_pub)

            if g_range_ahead < wall_thresh:
                print("bumped into object! Going to turn")
                if can_turn_left():
                    #turn left
                    driving_direction = 0
                else:
                    #turn right
                    driving_direction = 2

            else:
                print("moving forward", forward_twist.linear.x)
        else: # Rotating!
            print("turning")

            if driving_direction == 0:
                # turn left
                turn_left_and_go_a_little(cmd_vel_pub)
            elif driving_direction == 2:
                # turn right
                turn_right(cmd_vel_pub)

            # continue driving forward
            driving_direction = 1

    rate.sleep()
