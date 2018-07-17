#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
import struct
global scaling
global rel_g
global rel_m
global abs_g
global abs_m
global odom_pub
global tf_orient
tf_orient = None
rel_m=Vector3()
rel_g=Vector3()
abs_g = Quaternion(0,0,0,1)
abs_m = Point()
def callbackG(data):
    global rel_g
    global abs_g
    abs_g = data.pose.pose.orientation
    #abs_g = data.pose.pose.orientation
    #rel_g = data.twist.twist.angular
def callbackM(data):
    global rel_m
    rel_m = data.twist.twist.linear
    global abs_m
    abs_m = data.pose.pose.position
def my_callback(event):
    current_time = rospy.Time.now()
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom.pose.pose = Pose(abs_m, abs_g)
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(rel_m, rel_g)
    odom_pub.publish(odom)
    global tf_lin
    tf_lin = (odom.pose.pose.position.x, odom.pose.pose.position.y, 0.)
    global tf_orient
    tf_orient = (odom.pose.pose.orientation.x,  odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
#    odom_broadcaster.sendTransform(
#        (odom.pose.pose.position.x, odom.pose.pose.position.y, 0.),
#        (odom.pose.pose.orientation.x,  odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
#        current_time,
#        "base_footprint",
#        "odom"
#        )
def broadcastTF(event):
    if tf_orient is None:
        odom_broadcaster.sendTransform(
                (abs_m.x, abs_m.y, 0),
                (abs_g.x,abs_g.y,abs_g.z, abs_g.w),
                rospy.Time.now(),
                "base_footprint",
                "odom"
                )
    else:
        odom_broadcaster.sendTransform(tf_lin, tf_orient, rospy.Time.now(),"base_footprint","odom")

rospy.init_node('odom')
global odom_broadcaster
odom_broadcaster=tf.TransformBroadcaster()
rospy.Timer(rospy.Duration(0.1), my_callback)
rospy.Timer(rospy.Duration(1.0/10),broadcastTF)
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=3)
rospy.Subscriber("/gyro/odom", Odometry, callbackG)
rospy.Subscriber("/estimator/odom", Odometry, callbackM)
r = rospy.Rate(10)
rospy.spin()
