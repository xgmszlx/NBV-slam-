#include <geometry_msgs/PoseStamped.h>
#include <hnbv_msgs/FeaturePointArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_state_publisher");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/hnbv/slam/pose", 1);
  ros::Publisher status_pub = nh.advertise<std_msgs::String>("/hnbv/slam/status", 1, true);
  ros::Publisher features_pub = nh.advertise<hnbv_msgs::FeaturePointArray>("/hnbv/slam/features", 1);

  ros::Rate rate(10.0);
  while (ros::ok()) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    pose_pub.publish(pose);

    std_msgs::String status;
    status.data = "SCAFFOLD";
    status_pub.publish(status);

    hnbv_msgs::FeaturePointArray features;
    features.header = pose.header;
    features_pub.publish(features);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

