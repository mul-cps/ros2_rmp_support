#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler
import math

# Variables for odometry calculations
x, y, th = 0.0, 0.0, 0.0
vx, vy, vth = 0.0, 0.0, 0.0
last_time = None

# Publisher for odometry
odom_pub = None

# Callback function for processing cmd_vel messages
def this_node_callback(cmd_vel_msg):
    global x, y, th, vx, vy, vth, last_time

    # Print cmd_vel message data to the console
    rospy.loginfo("------------------------------------------------")
    rospy.loginfo(f"linear x: {cmd_vel_msg.linear.x}")
    rospy.loginfo(f"linear y: {cmd_vel_msg.linear.y}")
    rospy.loginfo(f"linear z: {cmd_vel_msg.linear.z}")
    rospy.loginfo(f"angular z: {cmd_vel_msg.angular.z}")
    rospy.loginfo("------------------------------------------------")

    current_time = rospy.Time.now()

    # Calculate velocity values based on cmd_vel
    vx = cmd_vel_msg.linear.x * math.cos(th)
    vy = cmd_vel_msg.linear.x * math.sin(th)
    vth = cmd_vel_msg.angular.z

    # Compute odometry
    dt = (current_time - last_time).to_sec()
    delta_x = vx * dt
    delta_y = vy * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # Create a quaternion from yaw
    odom_quat = quaternion_from_euler(0, 0, th)

    # Publish the transform over tf
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        (x, y, 0.0),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # Publish the odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # Set position
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = Quaternion(*odom_quat)

    # Set velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    # Publish the odometry message
    odom_pub.publish(odom)

    last_time = current_time

def main():
    global odom_pub, last_time

    rospy.init_node("odom_to_robot_tf_generator", anonymous=True)
    rospy.loginfo("Odom to robot generator is started...")

    rospy.Subscriber("measured_vel", Twist, this_node_callback)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

    last_time = rospy.Time.now()

    rospy.spin()

if __name__ == '__main__':
    main()
