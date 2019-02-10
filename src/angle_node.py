#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import pi
from std_msgs.msg import Float64

rospy.init_node('angle_node')  # initialize node

def odom_callback(msg):
    quaternion = msg.pose.pose.orientation  # get quarternion from Odometry message
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]  # turn it into a usable quaternion for the upcoming method
    global odom_angle
    odom_angle = tf.transformations.euler_from_quaternion(explicit_quat)[2]  # get just the z-axis angle

odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

angle_pub = rospy.Publisher('/angle', Float64, queue_size=10)

rate = rospy.Rate(20)  # to match key_publisher
odom_angle = None

while not rospy.is_shutdown():

    angle_pub.publish(odom_angle)
