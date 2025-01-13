#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <signal.h>

volatile sig_atomic_t flag = 0;

void signal_handler(int signum) {
  flag = 1;
}

class OdomToBaseLinkPublisher {
public:
  OdomToBaseLinkPublisher() : nh_() {
    // Subscribe to the /odom topic
    odom_sub_ = nh_.subscribe("/odom", 10, &OdomToBaseLinkPublisher::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Prepare the transform message
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg->header.stamp;
    odom_trans.header.frame_id = msg->header.frame_id; // Should be "odom"
    odom_trans.child_frame_id = msg->child_frame_id;   // Should be "base_link"

    // Copy translation and rotation
    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation = msg->pose.pose.orientation;

    // Publish the transform
    odom_broadcaster_.sendTransform(odom_trans);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster odom_broadcaster_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_base_link_publisher");

  // Register the signal handler
  signal(SIGINT, signal_handler);

  OdomToBaseLinkPublisher publisher;

  ros::Rate r(50); // Set loop rate
  while (ros::ok() && !flag) {
    ros::spinOnce(); // Handle callbacks
    r.sleep();
  }

  ROS_INFO("Odometry to base_link transform node shutting down.");
  ros::shutdown();
  return 0;
}
