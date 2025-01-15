#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

# Global variables for cumulative pose
x = 0.0
y = 0.0
theta = 0.0  # Orientation in radians

def odom_callback(msg):
    global x, y, theta, br

    # Time interval
    current_time = rospy.Time.now()
    dt = (current_time - odom_callback.last_time).to_sec()
    odom_callback.last_time = current_time

    # Extract linear and angular velocities
    linear_x = msg.twist.twist.linear.x
    linear_y = msg.twist.twist.linear.y
    angular_z = msg.twist.twist.angular.z

    # Compute the change in position and orientation
    delta_x = (linear_x * math.cos(theta) - linear_y * math.sin(theta)) * dt
    delta_y = (linear_x * math.sin(theta) + linear_y * math.cos(theta)) * dt
    delta_theta = angular_z * dt

    # Update cumulative position and orientation
    x += delta_x
    y += delta_y
    theta += delta_theta

    # Normalize theta to the range [-pi, pi]
    theta = (theta + math.pi) % (2 * math.pi) - math.pi

    # Create a TransformStamped message
    t = TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"

    # Set translation
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    # Set rotation (quaternion)
    q = quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # Broadcast the transform
    br.sendTransform(t)

def main():
    global br

    rospy.init_node("odom_to_base_link_tf_from_twist")
    
    # Initialize TransformBroadcaster
    br = tf2_ros.TransformBroadcaster()
    
    # Initialize last time for integration
    odom_callback.last_time = rospy.Time.now()
    
    # Subscribe to the /odom topic
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    rospy.loginfo("Broadcasting /odom to /base_link transform using twist")
    rospy.spin()

if __name__ == "__main__":
    main()
