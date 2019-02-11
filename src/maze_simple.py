#!/usr/bin/env python
# the goal of this script is to be a more "simple" version of the maze runner in maze.py
# whether this goal is accomlished of not is up to interpretation...
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import pi

rospy.init_node('maze')

# callback functions
# laser scanner
def scan_callback(msg):
    global g_range_ahead
    global g_range_left
    # get the elements w/in a certain range (30 degrees each direction)

    angle_range_ahead = msg.ranges[0:30] + msg.ranges[-30:]
    angle_range_left = msg.ranges[-120:-60]    # for real robots w/ clockwise lidar
    # angle_range_left - msg.ranges[60:120]    # for simulated robots w/ counterclockwise lidars

    # replace infinity readings (0.0) w/ a real number (4)
    angle_range_ahead = [4 if x == 0.0 else x for x in angle_range_ahead]
    angle_range_left = [4 if x == 0.0 else x for x in angle_range_left]

    g_range_ahead = min(angle_range_ahead)
    g_range_left = min(angle_range_left)


#odometry callback - only produces euler yaw (which ranges from pi to -pi)
def odom_callback(msg):
    global z_orientation
    quat = msg.pose.pose.orientation
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    z_orientation = tf.transformations.euler_from_quaternion(quat_list)


# default global variables
g_range_ahead = 4
g_range_left = 4
z_orientation = 0
# variables for use during navigation
states = ["no wall", "cornered", "following wall", "lost wall", "pick up left"]
robot_state = states[0]  # set initial state to no wall
past_state = None
movement_twist = Twist()
x_vel = 0.22  # can be between 0 and 0.22 for TB3 burger
z_vel = 0.2   # can be between 0 and 2.84 for TB3 burger
wall_threshold = 0.25  # should not be lower than 0.2
start_angle = 0
target_angle = 0
target_buffer = 0.1  # 0.1 rads = about 6 degrees
start_time = 0
sleep_rate = rospy.Rate(20)

# publishers/ subscribers
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)


while not rospy.is_shutdown():
    #search for a wall ahead
    if robot_state == states[0]:
        if g_range_ahead < wall_threshold:
            robot_state = states[1]
            past_state = states[0]
            start_angle = z_orientation
        else:
            movement_twist.linear.x = x_vel
            movement_twist.angular.z = 0


    # turn right because the robot is in a corner between a wall ahead and a left wall
    elif robot_state == states[1]:
        # turn right
        if not robot_state = past_state:  # if first loop through this state
            past_state = states[1]
            movement_twist.linear.x = 0
            target_angle = start_angle - 1.57
            if target_angle < -pi:
                # compensate for crossing the pos/neg pi line
                target_angle += 2*pi
        if abs(target_angle - z_orientation) < target_buffer:  # if the robot's oriantation is within a certain range of the target
            robot_state = states[2]  # go to wall following
            start_angle = z_orientation
        else:
            movement_twist.angular.z = -z_vel


    #follow a left wall until either the wall drops off or the robot is cornered
    elif robot_state == states[2]:
        if g_range_left < wall_thresh and angle_range_left < wall_thresh:
            robot_state = [1]  # go to cornered
        elif g_range_left > wall_thresh + 0.15:
            robot_state = [3]  # go to left turn
        else:
            movement_twist.linear.x = x_vel
            movement_twist.angular.z = target_angle - z_orientation



    #left wall dropped off, so turn left and follow
    elif robot_state == states[3]:
        # if the robot is turning after having followed a wall
        if not robot_state = past_state:
            past_state = states[3]
            movement_twist.linear.x = 0
            target_angle = start_angle + 1.57  # right turn
            if target_angle > pi:
                # compensate for crossing from pos to neg readings from odom
                target_angle -= 2*pi

        if abs(target_angle - z_orientation) < target_buffer:  # if the robot's oriantation is within a certain range of the target
            robot_state = states[4]
        else:
            movement_twist.angular.z = z_vel

            
    # go forward for a little bit until the left lidar pick up a wall
    elif robot_state == states[4]:
        if not robot_state = past_state:
            past_state = states[4]
            start_time = rospy.Time.now().secs

        if rospy.Time.now().secs > start_time + 2.5:
            robot_state = states[0]  # lost wall
        elif g_range_left < wall_threshold + 0.1:
            robot_state = states[2]  # picked up wall, go to wall following
        else:
            movement_twist.linear.x = x_vel
            movement_twist.angular.z = 0

    else:
        robot_state = states[0]  # reset state if robot_state is compromised
        movement_twist.linear.x = 0
        movement_twist.angular.z = 0

    # publish movement to robot
    cmd_vel_pub.publish(movement_twist)
    # if the state changes, print the state to the terminal
    if not robot_state == past_state:
        print(robot_state)
    rospy.sleep(sleep_rate)
