import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from math import pi

global wall_thresh

turn_speed = .3
forward_speed = 0.22
wall_thresh = 0.3

def angle_callback(msg):
    global z_angle
    z_angle = msg.data

# Function is called every time there's a scan
# global keyword is needed so g_range_ahead is available outside of the function
def scan_callback(msg):
    global g_range_ahead
    global g_range_left
    # get the elements w/in a certain range (30 degrees each direction)

    angle_range_ahead = msg.ranges[0:30] + msg.ranges[-30:]
    angle_range_left = msg.ranges[-120:-60]
    # replace infinity readings (0.0) w/ a real number (4)
    angle_range_ahead = [4 if x == 0.0 else x for x in angle_range_ahead]
    angle_range_left = [4 if x == 0.0 else x for x in angle_range_left]


    g_range_ahead = min(angle_range_ahead)
    g_range_left = min(angle_range_left)

# Main program
g_range_ahead = 4 # anything to start
g_range_left = 1
z_angle = 0
