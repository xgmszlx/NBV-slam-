#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_projection_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double resolution = 0.05;
  int width = 200;
  int height = 200;
  private_nh.param("resolution", resolution, resolution);
  private_nh.param("width", width, width);
  private_nh.param("height", height, height);

  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/hnbv/map/occupancy", 1, true);
  ros::Rate rate(1.0);
  while (ros::ok()) {
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";
    grid.info.resolution = resolution;
    grid.info.width = static_cast<unsigned int>(width);
    grid.info.height = static_cast<unsigned int>(height);
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.position.x = -0.5 * width * resolution;
    grid.info.origin.position.y = -0.5 * height * resolution;
    grid.data.assign(static_cast<std::size_t>(width * height), -1);
    map_pub.publish(grid);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

