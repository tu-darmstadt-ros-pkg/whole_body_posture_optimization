#include <whole_body_planning_server/whole_body_planning_server.h>
#include <whole_body_posture_optimization/util/parameter_utils.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace param_utils = whole_body_posture_optimization::param_utils;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("driving_controller_node");

  whole_body_planning_server::WholeBodyPlanningServer driving_controller(node);
  RCLCPP_INFO_STREAM(node->get_logger(), "Finished initializing driving controller");

  std::string tsdf_load_path = param_utils::param(*node, "tsdf_load_path", std::string(""));
  if (!tsdf_load_path.empty()) {
    driving_controller.loadTsdfFromPath(tsdf_load_path);
  }

  // Debug
  nav_msgs::msg::Path path;
  path.header.stamp = node->now();
  path.header.frame_id = "world";
  double path_start = param_utils::param(*node, "path_start", 0.0);
  double path_stop = param_utils::param(*node, "path_stop", 1.5);
  double path_step = param_utils::param(*node, "path_step", 0.02);
  for (double x = path_start; x < path_stop; x += path_step) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, 0, 0);
    pose_msg.pose = tf2::toMsg(pose);

    path.poses.push_back(pose_msg);
  }

  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("path_command", rclcpp::QoS(10).transient_local());
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin_some(node);
  path_pub->publish(path);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
