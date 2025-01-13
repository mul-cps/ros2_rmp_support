#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <signal.h>

volatile sig_atomic_t flag = 0;

void signal_handler(int signum) {
  flag = 1;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, tf::TransformBroadcaster& odom_broadcaster) {
  // Create transform message
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = msg->header.stamp;
  odom_trans.header.frame_id = msg->header.frame_id;
  odom_trans.child_frame_id = msg->child_frame_id;

  // Copy translation and rotation
  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;
  odom_trans.transform.rotation = msg->pose.pose.orientation;

  // Publish the transform
  odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;

  // Register the signal handler
  signal(SIGINT, signal_handler);

  // Create the transform broadcaster
  tf::TransformBroadcaster odom_broadcaster;

  // Subscribe to the /odom topic
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 10,
      boost::bind(odomCallback, _1, boost::ref(odom_broadcaster)));

  ROS_INFO("TF broadcaster node running...");

  // Spin until the node is stopped
  ros::Rate rate(10.0); // Adjust rate if necessary
  while (ros::ok() && !flag) {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("TF broadcaster node shutting down.");
  ros::shutdown();
  return 0;
}
