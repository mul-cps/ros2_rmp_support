#!/usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def odom_callback(msg):
    # Create a TransformStamped message
    odom_trans = TransformStamped()
    odom_trans.header.stamp = msg.header.stamp
    odom_trans.header.frame_id = msg.header.frame_id

    # Set child_frame_id explicitly (e.g., base_link)
    odom_trans.child_frame_id = "base_link"

    # Set the translation
    odom_trans.transform.translation.x = msg.pose.pose.position.x
    odom_trans.transform.translation.y = msg.pose.pose.position.y
    odom_trans.transform.translation.z = msg.pose.pose.position.z

    # Set the rotation
    odom_trans.transform.rotation = msg.pose.pose.orientation

    # Publish the transform
    tf_broadcaster.sendTransform(odom_trans)

    # print debug info
    rospy.loginfo("Position: x={}, y={}, z={}".format(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ))

    rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ))

if __name__ == "__main__":
    rospy.init_node("tf2_broadcaster")

    # Create a TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to the /odom topic
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.loginfo("TF2 broadcaster node is running...")

    # Keep the node running
    rospy.spin()
