#include <geometry_msgs/Twist.h>
#include <hnbv_msgs/FeaturePointArray.h>
#include <hnbv_msgs/TrackedObjectArray.h>
#include <ros/ros.h>

namespace {

class LocalNbvNode {
 public:
  LocalNbvNode() : private_nh_("~") {
    private_nh_.param("rate_hz", rate_hz_, 10.0);
    private_nh_.param("max_linear_velocity", max_linear_velocity_, 0.2);
    private_nh_.param("max_angular_velocity", max_angular_velocity_, 0.2);

    tracks_sub_ = nh_.subscribe("/hnbv/semantics/tracks", 1, &LocalNbvNode::onTracks, this);
    features_sub_ = nh_.subscribe("/hnbv/slam/features", 1, &LocalNbvNode::onFeatures, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/hnbv/planner/local_cmd_vel", 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_), &LocalNbvNode::onTimer, this);
  }

 private:
  void onTracks(const hnbv_msgs::TrackedObjectArrayConstPtr& tracks) {
    last_dynamic_count_ = 0;
    for (const auto& object : tracks->objects) {
      if (object.geometry_dynamic) {
        ++last_dynamic_count_;
      }
    }
  }

  void onFeatures(const hnbv_msgs::FeaturePointArrayConstPtr& features) {
    last_feature_count_ = features->points_px.size();
  }

  void onTimer(const ros::TimerEvent&) {
    geometry_msgs::Twist cmd;
    if (last_dynamic_count_ > 0 && last_feature_count_ > 0) {
      cmd.linear.x = 0.5 * max_linear_velocity_;
      cmd.angular.z = 0.5 * max_angular_velocity_;
    }
    cmd_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber tracks_sub_;
  ros::Subscriber features_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;
  double rate_hz_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  std::size_t last_dynamic_count_ = 0;
  std::size_t last_feature_count_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_nbv_node");
  LocalNbvNode node;
  ros::spin();
  return 0;
}

