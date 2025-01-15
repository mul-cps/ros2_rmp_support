#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import quaternion_from_euler


def odom_callback(msg, broadcaster):
    # Create a TransformStamped object
    transform = TransformStamped()

    # Header
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"  # Parent frame
    transform.child_frame_id = "base_link"  # Child frame

    # Translation from Pose
    transform.transform.translation.x = msg.pose.position.x
    transform.transform.translation.y = msg.pose.position.y
    transform.transform.translation.z = msg.pose.position.z

    # Rotation (quaternion already in the pose message)
    transform.transform.rotation = msg.pose.orientation

    # Broadcast the transform
    broadcaster.sendTransform(transform)


def publish_transform_from_odom():
    rospy.init_node("odom_to_base_link_broadcaster")

    # Create a TransformBroadcaster object
    broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to /odom topic
    rospy.Subscriber("/odom", PoseStamped, odom_callback, broadcaster)

    # Spin to keep the node running and processing callbacks
    rospy.spin()


if __name__ == "__main__":
    try:
        publish_transform_from_odom()
    except rospy.ROSInterruptException:
        pass
