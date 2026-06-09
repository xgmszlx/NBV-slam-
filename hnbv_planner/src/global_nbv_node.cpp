#include <geometry_msgs/PoseStamped.h>
#include <hnbv_msgs/PlannerDebug.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace {

class GlobalNbvNode {
 public:
  GlobalNbvNode() : private_nh_("~") {
    private_nh_.param("camera_fov_deg", camera_fov_deg_, 90.0);
    private_nh_.param("max_range_m", max_range_m_, 5.0);
    private_nh_.param("lambda_d", lambda_d_, 0.8);
    private_nh_.param("lambda_theta", lambda_theta_, 1.0);

    map_sub_ = nh_.subscribe("/hnbv/map/occupancy", 1, &GlobalNbvNode::onMap, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/hnbv/planner/global_goal", 1, true);
    debug_pub_ = nh_.advertise<hnbv_msgs::PlannerDebug>("/hnbv/planner/global_debug", 1, true);
  }

 private:
  void onMap(const nav_msgs::OccupancyGridConstPtr& map) {
    if (map->data.empty()) {
      ROS_WARN_THROTTLE(5.0, "global_nbv_node received an empty occupancy grid");
      return;
    }

    geometry_msgs::PoseStamped goal;
    goal.header = map->header;
    goal.pose.position.x = map->info.origin.position.x + 0.5 * map->info.width * map->info.resolution;
    goal.pose.position.y = map->info.origin.position.y + 0.5 * map->info.height * map->info.resolution;
    goal.pose.orientation.w = 1.0;
    goal_pub_.publish(goal);

    hnbv_msgs::PlannerDebug debug;
    debug.header = map->header;
    debug.planner_name = "global_mi_nbv";
    debug.selected_pose = goal;
    debug.selected_score = 0.0;
    debug.selected_information_gain = 0.0;
    debug.selected_cost = 0.0;
    debug_pub_.publish(debug);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher debug_pub_;
  double camera_fov_deg_;
  double max_range_m_;
  double lambda_d_;
  double lambda_theta_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_nbv_node");
  GlobalNbvNode node;
  ros::spin();
  return 0;
}

