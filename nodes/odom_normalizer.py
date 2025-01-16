#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np

def normalize_quaternion(qx, qy, qz, qw):
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm == 0:
        return 0.0, 0.0, 0.0, 1.0  # Default to unit quaternion
    return qx / norm, qy / norm, qz / norm, qw / norm

def odom_callback(msg):
    # Normalize quaternion
    qx, qy, qz, qw = normalize_quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )

    # Create new message
    odom_normalized = Odometry()
    odom_normalized.header = msg.header
    odom_normalized.child_frame_id = msg.child_frame_id

    # Copy position
    odom_normalized.pose.pose.position = msg.pose.pose.position

    # Set normalized quaternion
    odom_normalized.pose.pose.orientation.x = qx
    odom_normalized.pose.pose.orientation.y = qy
    odom_normalized.pose.pose.orientation.z = qz
    odom_normalized.pose.pose.orientation.w = qw

    # Copy covariance
    odom_normalized.pose.covariance = msg.pose.covariance
    odom_normalized.twist = msg.twist  # Copy twist data

    # Publish normalized odom
    pub.publish(odom_normalized)

if __name__ == "__main__":
    rospy.init_node("odom_normalizer")

    # Publisher
    pub = rospy.Publisher("/odom_normalized", Odometry, queue_size=10)

    # Subscriber
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.spin()
