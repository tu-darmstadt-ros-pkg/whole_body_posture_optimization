#include <rclcpp/rclcpp.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include "whole_body_posture_optimization/util/timing.h"
#include "whole_body_posture_optimization/util/parameter_utils.h"

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

#ifdef WBPO_HAVE_HEIGHTMAP
#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_world_heightmap/io.h>
#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#endif

#include <whole_body_planning_plotting_utils/data_log.h>

using whole_body_planning_plotting_utils::DataLog;
namespace param_utils = whole_body_posture_optimization::param_utils;

nav_msgs::msg::Path path_;
bool new_path_received_ = false;
double default_path_resolution;

nav_msgs::msg::Path createPath(double start_x, double stop_x, double step, const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "world";
  double dir = start_x < stop_x ? 1 : -1;
  for (double x = start_x; dir * x < dir * stop_x; x+=dir*step) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = path.header.frame_id;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    pose.translation() = Eigen::Vector3d(x, 0, 0) + offset;
    pose_msg.pose = tf2::toMsg(pose);

    path.poses.push_back(pose_msg);
  }
  return path;
}

bool isNullQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  return quat.x == 0.0 && quat.y == 0.0 && quat.z == 0.0 && quat.w == 0.0;
}

nav_msgs::msg::Path createStraightPath(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, double step) {
  // Compute steps
  Eigen::Vector3d start_position, goal_position;
  tf2::fromMsg(start.pose.position, start_position);
  tf2::fromMsg(goal.pose.position, goal_position);
  Eigen::Vector3d position_difference = goal_position - start_position;
  int num_points = std::floor(position_difference.norm() / step);

  // Set up path
  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  path.poses.reserve(num_points+1);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = start.header.frame_id; // frame_id should be the same for every pose and path

  // Compute orientation on path
  Eigen::Vector3d position_difference_no_z = position_difference;
  position_difference_no_z.z() = 0;

  // Compute angle between start orientation and path
  double backwards;
  if (isNullQuaternion(start.pose.orientation)) {
    backwards = 1.0;
  } else {
    Eigen::Quaterniond start_orientation;
    tf2::fromMsg(start.pose.orientation, start_orientation);
    Eigen::Vector3d start_direction = start_orientation * Eigen::Vector3d::UnitX();
    double angle_to_path = std::acos(start_direction.dot(position_difference_no_z)/(position_difference_no_z.norm() * start_direction.norm()));
    backwards = angle_to_path < M_PI_2 ? 1.0 : -1.0;
  }

  // Path orientation points x-axis to goal (in 2D)
  Eigen::Quaterniond path_orientation;
  path_orientation.setFromTwoVectors(backwards * Eigen::Vector3d::UnitX(), position_difference_no_z);
  pose.pose.orientation = tf2::toMsg(path_orientation);

  // Fill path
  path.poses.emplace_back(start); // Use start orientation for this one
  if (isNullQuaternion(start.pose.orientation)) {
    // If start orientation is not set, point to goal
    path.poses.back().pose.orientation = pose.pose.orientation;
  }
  for (int i = 1; i < num_points; ++i) {
    double t = static_cast<double>(i) / num_points;
    Eigen::Vector3d position = start_position + t * position_difference;
    pose.pose.position = tf2::toMsg(position);
    path.poses.emplace_back(pose);
  }
  path.poses.emplace_back(goal);
  if (isNullQuaternion(goal.pose.orientation)) {
    // If goal orientation is not set, point to goal
    path.poses.back().pose.orientation = pose.pose.orientation;
  }
  return path;
}

// This function assumes, that the last pose of the base path matches the first pose of the path to append
void appendPath(nav_msgs::msg::Path& base_path, const nav_msgs::msg::Path& to_append) {
  // Paths should at least contain start and goal
  if (to_append.poses.size() < 2) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("whole_body_posture_optimization_node"), "Path is shorter than 2");
    return;
  }
  if (base_path.poses.empty()) {
    base_path.poses = to_append.poses;
  } else {
    base_path.poses.reserve(base_path.poses.size() + to_append.poses.size() - 1);
    // Skip first pose because we assume it is identical to the last pose of the base path
    for (size_t i = 1; i < to_append.poses.size(); ++i) {
      base_path.poses.emplace_back(to_append.poses[i]);
    }
  }

}

nav_msgs::msg::Path createPath(const nav_msgs::msg::Path& sparse_path, double path_step=0.05) {
  // Just return short paths
  if (sparse_path.poses.size() < 2) {
    return sparse_path;
  }

  // Connect the given points with straight paths
  nav_msgs::msg::Path dense_path;
  dense_path.header = sparse_path.header;
  for (size_t i = 0; i < sparse_path.poses.size() - 1; ++i) {
    nav_msgs::msg::Path path_part = createStraightPath(sparse_path.poses[i], sparse_path.poses[i + 1], path_step);
    appendPath(dense_path, path_part);
  }
  return dense_path;
}

nav_msgs::msg::Path createPathFromParameters(rclcpp::Node& node) {
  double path_start, path_stop, path_step;
  if (param_utils::getParam(node, "path_start", path_start) &&
      param_utils::getParam(node, "path_stop", path_stop) &&
      param_utils::getParam(node, "path_step", path_step)) {
    return createPath(path_start, path_stop, path_step);
  } else {
    return {};
  }
}

void pathReceivedCallback(const nav_msgs::msg::Path::ConstSharedPtr& path_msg) {
  path_ = *path_msg;
  new_path_received_ = true;
}

void sparsePathReceivedCallback(const nav_msgs::msg::Path::ConstSharedPtr& path_msg) {
  path_ = createPath(*path_msg, default_path_resolution);
  new_path_received_ = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("whole_body_posture_optimization_node");

  // Subscribers
  default_path_resolution = param_utils::param(*node, "path_step", 0.05);
  auto sparse_path_sub = node->create_subscription<nav_msgs::msg::Path>("sparse_goal_path", 10, &sparsePathReceivedCallback);
  auto path_sub = node->create_subscription<nav_msgs::msg::Path>("goal_path", 10, &pathReceivedCallback);

  // Load environment
  auto sdf_model_node = node->create_sub_node("sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_node);
  sdf_model->loadFromServer(sdf_model_node);

  // Create pose predictor
  std::string pose_predictor_name = param_utils::param(*node, "pose_predictor", std::string("sdf_contact_estimation::SDFContactEstimation"));
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor;
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    // Note: ShapeModel and SDFContactEstimation read their parameters from the node root
    // (same convention as the ported sdf_contact_estimation_node).
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(node);
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(node, shape_model, sdf_model);
    sdf_pose_predictor->enableVisualisation(false);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
  } else if (pose_predictor_name == "hector_heightmap_pose_prediction::HeightmapPosePredictor") {
#ifdef WBPO_HAVE_HEIGHTMAP
    auto heightmap_pub = node->create_publisher<grid_map_msgs::msg::GridMap>("heightmap", rclcpp::QoS(10).transient_local());
    // Load heightmap
    std::string map_path;
    if (!param_utils::getParam(*node, "heightmap.map_bag_file_path", map_path)) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load mandatory parameter 'heightmap.map_bag_file_path'");
      return 0;
    }
    hector_world_heightmap::MapBag<double>::Ptr map_bag = hector_world_heightmap::io::loadFromFile<double>(map_path);
    if (!map_bag) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load map from path '" << map_path << "'");
      return 0;
    }
    heightmap_pub->publish(hector_world_heightmap_ros::message_conversions::heightmapToMsg<double>(map_bag->getMap(0)));
    urdf::Model urdf;
    std::string robot_description;
    if (!param_utils::getParam(*node, "robot_description", robot_description) || !urdf.initString(robot_description)) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load urdf.");
      return 0;
    }
    auto robot_heightmap_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<double>>(map_bag->resolution(), urdf);
    robot_heightmap_provider->disableHeightmapCache();
    auto heightmap_pose_predictor = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<double>>(map_bag, robot_heightmap_provider);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);
#else
    RCLCPP_ERROR_STREAM(node->get_logger(), "This package was built without heightmap pose prediction support "
                        "(build with -DWITH_HEIGHTMAP_POSE_PREDICTION=ON).");
    return 0;
#endif
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Unknown pose predictor '" << pose_predictor_name << "'.");
    return 0;
  }

  // Initialize logging
  std::string logging_folder = param_utils::param(*node, "flipper_optimization_logging_folder", std::string(""));
  if (!logging_folder.empty()) {
    DataLog::getInstance().getTable("flipper_optimization").enableLogging();
    DataLog::getInstance().getTable("arm_optimization").enableLogging();
  }

  whole_body_posture_optimization::WholeBodyPostureOptimization po(node, pose_predictor, sdf_model);

  // Create path from parameters
  path_ = createPathFromParameters(*node);
  new_path_received_ = !path_.poses.empty();

  std::string trajectory_statistics_save_folder;
  bool save_trajectory_statistics = param_utils::getParam(*node, "trajectory_statistics_save_folder", trajectory_statistics_save_folder);

  // Spin and wait for new paths
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (new_path_received_) {
      DataLog::getInstance().getTable("flipper_optimization").clear();
      // Compute optimal postures
      auto result = po.findOptimalPostures(path_);
      std::vector<size_t> trajectory_start_indices;
      auto trajectory = po.computeTimedTrajectory(result, trajectory_start_indices);
      po.publishTrajectory(trajectory);

      // Print and save results
      whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
      whole_body_posture_optimization::Timing::printTimeInfos();
      if (!logging_folder.empty()) {
        DataLog::getInstance().saveTablesToFolder(logging_folder);
      }
      if (save_trajectory_statistics && !result.empty() && result.back().success) {
        std::string time_stamp = whole_body_posture_optimization::getTimestamp();
        std::string save_path = trajectory_statistics_save_folder + "/trajectory_statistics-" + time_stamp + ".csv";
        if (whole_body_posture_optimization::WholeBodyPostureOptimization::saveTrajectoryStatistics(result, trajectory, trajectory_start_indices, save_path)) {
          RCLCPP_INFO_STREAM(node->get_logger(), "Saved trajectory statistics in " << save_path);
        }
      }

      new_path_received_ = false;
    }
    try {
      rate.sleep();
    } catch (const std::runtime_error&) {
      break; // context invalidated by Ctrl-C during sleep
    }
  }

  rclcpp::shutdown();
  return 0;
}
