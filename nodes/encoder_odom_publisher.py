#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from segway_msgs.msg import EncoderTicks  # Replace with actual message type
import math

class EncoderOdometry:
    def __init__(self):
        rospy.init_node("encoder_odom_publisher")

        # Robot parameters (update these according to your robot)
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.3  # Distance between wheels (m)
        self.ticks_per_revolution = 1000  # Encoder ticks per wheel revolution

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_l_ticks = None
        self.last_r_ticks = None
        self.last_time = None

        # ROS publishers (Updated topic to /odom_new)
        self.odom_pub = rospy.Publisher("/odom_new", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # ROS subscriber
        rospy.Subscriber("/ticks_fb", EncoderTicks, self.ticks_callback)

        rospy.spin()

    def ticks_callback(self, msg):
        if self.last_l_ticks is None or self.last_r_ticks is None or self.last_time is None:
            self.last_l_ticks = msg.l_ticks
            self.last_r_ticks = msg.r_ticks
            self.last_time = msg.ticks_timestamp / 1e6  # Convert to seconds
            return

        # Compute time delta
        current_time = msg.ticks_timestamp / 1e6  # Convert to seconds
        dt = current_time - self.last_time
        if dt <= 0:
            return  # Skip if time is invalid

        # Compute tick differences
        delta_l = msg.l_ticks - self.last_l_ticks
        delta_r = msg.r_ticks - self.last_r_ticks

        # Convert ticks to meters
        meters_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        delta_s_l = delta_l * meters_per_tick
        delta_s_r = delta_r * meters_per_tick

        # Compute linear and angular velocity
        v = (delta_s_l + delta_s_r) / 2.0 / dt
        omega = (delta_s_r - delta_s_l) / self.wheel_base / dt

        # Integrate motion (basic differential drive kinematics)
        delta_theta = omega * dt
        self.theta += delta_theta
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Create odometry message
        odom = Odometry()
        odom.header = Header(stamp=rospy.Time.from_sec(current_time), frame_id="odom")
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (as quaternion)
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocities
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Publish odometry to /odom_new
        self.odom_pub.publish(odom)

        # Publish TF transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            q,
            rospy.Time.from_sec(current_time),
            "base_link",
            "odom"
        )

        # Update previous values
        self.last_l_ticks = msg.l_ticks
        self.last_r_ticks = msg.r_ticks
        self.last_time = current_time

if __name__ == "__main__":
    try:
        EncoderOdometry()
    except rospy.ROSInterruptException:
        pass
