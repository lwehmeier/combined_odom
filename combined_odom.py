#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import struct
global scaling
global rel_g
global rel_m
global abs_g
global abs_m
global odom_pub
rel_m=Vector3()
rel_g=Vector3()
abs_g = Quaternion()
abs_m = Point()
def callbackG(data):
    global rel_g
    global abs_g
    abs_g = data.pose.pose.orientation
    rel_g = data.twist.twist.angular
def callbackM(data):
    global rel_m
    rel_m = data.twist.twist.linear
    global abs_m
    abs_m = data.pose.pose.position
def my_callback(event):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom_frame_combined"
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom.pose.pose = Pose(abs_m, abs_g)
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(rel_m, rel_g)
    odom_pub.publish(odom)

rospy.init_node('odom')
rospy.Timer(rospy.Duration(0.1), my_callback)
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=3)
rospy.Subscriber("/gyro/odom", Odometry, callbackG)
rospy.Subscriber("/mouse/odom", Odometry, callbackM)
r = rospy.Rate(10) # 10hz
rospy.spin()
