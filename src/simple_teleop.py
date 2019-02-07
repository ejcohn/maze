#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from math import pi

rospy.init_node('simple_teleop')  # initialize node
global turn_speed
global wall_thresh
global forward_speed
turn_speed = 0.6
wall_thresh = 0.3
forward_speed = 0.22

def key_callback(msg):  # this function may not be necessary?
    global key
    key = str(msg)
    key = key[7:-1]  # slice to only get the key that was input

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
    start_angle = odom_angle
    while odom_angle < start_angle + 1.57:
        turn_twist = Twist()
        turn_twist.angular.z = -turn_speed #Infinite loop currently!! doesn't work!!
        cmd_pub.publish(turn_twist)
    #while abs(start_angle - angle) < angle
    #start_time = rospy.Time.now().secs
    #turn_time = (angle*pi/180 / turn_speed) / 10
    # while(rospy.Time.now().secs - turn_time < start_time):
    #     turn_twist = Twist()
    #     turn_twist.angular.z =  -turn_speed
    #     cmd_pub.publish(turn_twist)


# turns left a given number of degrees (default 90) then moves forward a little but (default 1 second)
def turn_left_and_go_a_little(cmd_pub, angle=90, x_time=1):
    start_time = rospy.Time.now().secs
    turn_time = (angle*pi/180 / turn_speed) / 10
    while(rospy.Time.now().secs - turn_time < start_time):
        turn_twist = Twist()
        turn_twist.angular.z =  turn_speed
        cmd_pub.publish(turn_twist)

    start_time = rospy.Time.now().secs
    while(rospy.Time.now().secs - x_time < start_time and g_range_ahead > wall_thresh):
        forward_twist = Twist()
        forward_twist.linear.x = forward_speed
        cmd_pub.publish(forward_twist)

def odom_callback(msg): 
    quaternion = msg.pose.pose.orientation
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    global odom_angle
    odom_angle = tf.transformations.euler_from_quaternion(explicit_quat)[2]


key = None  # default to null
odom_angle = None
left_vals = []
right_vals = []
g_range_ahead = 1
g_range_left = 1

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # command publisher

key_sub = rospy.Subscriber('keys', String, key_callback, queue_size=10)  # keystroke subscriber

laser_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

main_drive = Twist()
main_drive.linear.x = 0
main_drive.angular.z = 0

rate = rospy.Rate(20)  # to match key_publisher

hint_counter = 0  # a counter of how many times to wait before printing the controls
print('*****----------*****')  # initial breaker line to separate from roslaunch output
while not rospy.is_shutdown():
    if hint_counter % 11 == 0:  # hint every 10 commands
        print('controls:\nleft: l | right: r\nview important lidar values: v')  # choosing bad controls because wasdx has been done before
        hint_counter += 1  # so the hint does not get published over and over

    if not key == None:

        print('debug keystroke:', key)  # for debugging

    # BUG: sometimes the program will read a valid command as an invalid command, e.g. 'keystroke 'f' not a valid command' even though it is...
    if key == 'f' or key == 'F':  # allow for caps
        main_drive.linear.x += 0.1
    elif key == 'l' or key == 'L':
        turn_left_and_go_a_little(cmd_vel_pub)
    elif key == 'r' or key == 'R':
        turn_right(cmd_vel_pub)
    elif key == 'v' or key == 'V':
        print('left', g_range_left)
        print('ahead', g_range_ahead)
    elif key == 'o' or key == 'O':
        print('odom', odom_angle)
    elif key == None:
        continue
    else:
        print('keystroke {} not a valid command'.format(key))

    if not key == None:
        print(key, main_drive.linear.x, main_drive.angular.z)  # print the current velocity of the robot
        hint_counter += 1
    key = None  # reset key to nothing

    # Publish cmd_vel with the desired motion
    cmd_vel_pub.publish(main_drive)

    # Sleep for 1/rate seconds
    rate.sleep()
