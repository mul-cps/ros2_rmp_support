#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf

def odom_callback(msg):
    # Extract position and orientation from the odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    
    # Broadcast the transformation
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (position.x, position.y, position.z),  # Translation
        (orientation.x, orientation.y, orientation.z, orientation.w),  # Rotation (quaternion)
        rospy.Time.now(),  # Timestamp
        msg.child_frame_id if msg.child_frame_id else "base_link",  # Child frame ID
        msg.header.frame_id if msg.header.frame_id else "odom"  # Parent frame ID
    )

def main():
    rospy.init_node('odom_tf_publisher', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.loginfo("TF Publisher for /odom topic is running...")
    rospy.spin()

if __name__ == '__main__':
    main()
