#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "masked_pointcloud_node");
  ros::NodeHandle nh;
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/hnbv/map/masked_points", 1);
  ROS_INFO("masked_pointcloud_node scaffold started; RGB-D back-projection will be implemented in M3");
  ros::spin();
  return 0;
}

