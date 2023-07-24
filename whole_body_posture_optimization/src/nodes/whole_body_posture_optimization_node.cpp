#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <whole_body_posture_optimization/timing.h>

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_world_heightmap/io.h>
#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <grid_map_msgs/GridMap.h>

#include <whole_body_planning_plotting_utils/data_log.h>

using whole_body_planning_plotting_utils::DataLog;

nav_msgs::Path path_;
bool new_path_received_ = false;

nav_msgs::Path createPath(double start_x, double stop_x, double step, const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()) {
  nav_msgs::Path path;
  path.header.frame_id = "world";
  double dir = start_x < stop_x ? 1 : -1;
  for (double x = start_x; dir * x < dir * stop_x; x+=dir*step) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = path.header.frame_id;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//     if (x == start_x) {
//       pose = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
//     }

    pose.translation() = Eigen::Vector3d(x, 0, 0) + offset;
    tf::poseEigenToMsg(pose, pose_msg.pose);

    path.poses.push_back(pose_msg);
  }
  return path;
}

bool isNullQuaternion(const geometry_msgs::Quaternion& quat) {
  return quat.x == 0.0 && quat.y == 0.0 && quat.z == 0.0 && quat.w == 0.0;
}

nav_msgs::Path createStraightPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double step) {
  // Compute steps
  Eigen::Vector3d start_position, goal_position;
  tf::pointMsgToEigen(start.pose.position, start_position);
  tf::pointMsgToEigen(goal.pose.position, goal_position);
  Eigen::Vector3d position_difference = goal_position - start_position;
  int num_points = floor(position_difference.norm() / step);

  // Set up path
  nav_msgs::Path path;
  path.header.frame_id = start.header.frame_id;
  path.poses.reserve(num_points+1);
  geometry_msgs::PoseStamped pose;
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
    tf::quaternionMsgToEigen(start.pose.orientation, start_orientation);
    Eigen::Vector3d start_direction = start_orientation * Eigen::Vector3d::UnitX();
    double angle_to_path = std::acos(start_direction.dot(position_difference_no_z)/(position_difference_no_z.norm() * start_direction.norm()));
    backwards = angle_to_path < M_PI_2 ? 1.0 : -1.0;
  }

  // Path orientation points x-axis to goal (in 2D)
  Eigen::Quaterniond path_orientation;
  path_orientation.setFromTwoVectors(backwards * Eigen::Vector3d::UnitX(), position_difference_no_z);
  tf::quaternionEigenToMsg(path_orientation, pose.pose.orientation);

  // Fill path
  path.poses.emplace_back(start); // Use start orientation for this one
  if (isNullQuaternion(start.pose.orientation)) {
    // If start orientation is not set, point to goal
    path.poses.back().pose.orientation = pose.pose.orientation;
  }
  for (unsigned int i = 1; i < num_points; ++i) {
    double t = static_cast<double>(i) / num_points;
    Eigen::Vector3d position = start_position + t * position_difference;
    tf::pointEigenToMsg(position, pose.pose.position);
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
void appendPath(nav_msgs::Path& base_path, const nav_msgs::Path& to_append) {
  // Paths should at least contain start and goal
  if (to_append.poses.size() < 2) {
    ROS_WARN_STREAM("Path is shorter than 2");
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

nav_msgs::Path createPath(const nav_msgs::Path& sparse_path) {
  // Just return short paths
  if (sparse_path.poses.size() < 2) {
    return sparse_path;
  }

  // Connect the given points with straight paths
  nav_msgs::Path dense_path;
  dense_path.header = sparse_path.header;
  for (size_t i = 0; i < sparse_path.poses.size() - 1; ++i) {
    nav_msgs::Path path_part = createStraightPath(sparse_path.poses[i], sparse_path.poses[i + 1], 0.05);
    appendPath(dense_path, path_part);
  }
  return dense_path;
}

nav_msgs::Path createPathFromParameters(const ros::NodeHandle& nh) {
  double path_start, path_stop, path_step;
  if (nh.getParam("path_start", path_start) &&
      nh.getParam("path_stop", path_stop) &&
      nh.getParam("path_step", path_step)) {
    return createPath(path_start, path_stop, path_step);
  } else {
    return {};
  }
}

void pathReceivedCallback(const nav_msgs::PathConstPtr& path_msg) {
  path_ = *path_msg;
  new_path_received_ = true;
}

void sparsePathReceivedCallback(const nav_msgs::PathConstPtr& path_msg) {
  path_ = createPath(*path_msg);
  new_path_received_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
  ros::NodeHandle pnh("~");

  // Subscribers
  ros::Subscriber sparse_path_sub = pnh.subscribe("sparse_goal_path", 10, &sparsePathReceivedCallback);
  ros::Subscriber path_sub = pnh.subscribe("goal_path", 10, &pathReceivedCallback);

  ros::Publisher heightmap_pub = pnh.advertise<grid_map_msgs::GridMap>("heightmap", 10, true);

  // Load environment
  ros::NodeHandle sdf_model_nh(pnh, "sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_nh);
  sdf_model->loadFromServer(sdf_model_nh);

  // Create pose predictor
  std::string pose_predictor_name = pnh.param<std::string>("pose_predictor", "sdf_contact_estimation::SDFContactEstimation");
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor;
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    ros::NodeHandle pose_predictor_nh(pnh, "sdf_pose_predictor");
    ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model);
    sdf_pose_predictor->enableVisualisation(false);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
  } else if (pose_predictor_name == "hector_heightmap_pose_prediction::HeightmapPosePredictor") {
    // Load heightmap
    ros::NodeHandle heightmap_nh(pnh, "heightmap");
    std::string map_path;
    if (!heightmap_nh.getParam("map_bag_file_path", map_path)) {
      ROS_ERROR_STREAM("Could not load mandatory parameter '" << heightmap_nh.getNamespace() << "/map_bag_file_path'");
      return 0;
    }
    hector_world_heightmap::MapBag<double>::Ptr map_bag = hector_world_heightmap::io::loadFromFile<double>(map_path);
    if (!map_bag) {
      ROS_ERROR_STREAM("Failed to load map from path '" << map_path << "'");
      return 0;
    }
    heightmap_pub.publish(hector_world_heightmap_ros::message_conversions::heightmapToMsg<double>(map_bag->getMap(0)));
    urdf::Model urdf;
    if (!urdf.initParam("/robot_description")) {
      ROS_ERROR_STREAM("Failed to load urdf.");
      return 0;
    }
    auto robot_heightmap_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<double>>(map_bag->resolution(), urdf);
    robot_heightmap_provider->disableHeightmapCache();
    auto heightmap_pose_predictor = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<double>>(map_bag, robot_heightmap_provider);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);
  } else {
    ROS_ERROR_STREAM("Unknown pose predictor '" << pose_predictor_name << "'.");
    return 0;
  }

  // Initialize logging
  std::string logging_folder = pnh.param("flipper_optimization_logging_folder", std::string(""));
  if (!logging_folder.empty()) {
    DataLog::getInstance().getTable("flipper_optimization").enableLogging();
  }

  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, pose_predictor, sdf_model);

  // Create path from parameters
  path_ = createPathFromParameters(pnh);
  new_path_received_ = !path_.poses.empty();

  std::string trajectory_statistics_save_folder;
  bool save_trajectory_statistics = pnh.getParam("trajectory_statistics_save_folder", trajectory_statistics_save_folder);

  // Spin and wait for new paths
  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::spinOnce();
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
          ROS_INFO_STREAM("Saved trajectory statistics in " << save_path);
        }
      }

      new_path_received_ = false;
    }
    rate.sleep();
  }

  return 0;
}
