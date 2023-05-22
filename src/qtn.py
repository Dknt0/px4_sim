#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion
import tf
import sys

# ZYX
quaternion = tf.transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

quaternion_2 = tf.transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
q_2 = Quaternion([quaternion_2[3],quaternion_2[0],quaternion_2[1],quaternion_2[2]])

# ZYX 代表姿态
quaternion_1 = tf.transformations.quaternion_from_euler(0, 0, 0)
q_ = Quaternion([quaternion_1[3],quaternion_1[0],quaternion_1[1],quaternion_1[2]])

q_ = q_*q
# local_pose.pose.orientation.w = q_[0]
# local_pose.pose.orientation.x = q_[1]
# local_pose.pose.orientation.y = q_[2]
# local_pose.pose.orientation.z = q_[3]

print(q * q_2)