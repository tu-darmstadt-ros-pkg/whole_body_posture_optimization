#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <whole_body_posture_optimization/timing.h>

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

nav_msgs::Path path_;
bool new_path_received_ = false;

nav_msgs::Path createPath(double start_x, double stop_x, double step, const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()) {
  nav_msgs::Path path;
  double dir = start_x < stop_x ? 1 : -1;
  for (double x = start_x; dir * x < dir * stop_x; x+=dir*step) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";

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

nav_msgs::Path createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double step) {
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
  // Goal orientation points x-axis to goal
  Eigen::Quaterniond path_orientation;
  path_orientation.setFromTwoVectors(Eigen::Vector3d::UnitX(), position_difference);
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
  if (path_msg->poses.size() == 2) {
    path_ = createPath(path_msg->poses[0], path_msg->poses[1], 0.05);
  } else {
    path_ = *path_msg;
  }
  new_path_received_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
  ros::NodeHandle pnh("~");

  // Subscribers
  ros::Subscriber path_sub = pnh.subscribe("goal_path", 10, &pathReceivedCallback);

  // Load environment
  ros::NodeHandle sdf_model_nh(pnh, "sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_nh);
  sdf_model->loadFromServer(sdf_model_nh);

  // Create pose predictor
  // Create robot model
  ros::NodeHandle pose_predictor_nh(pnh, "pose_predictor");
  ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(false);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, pose_predictor, sdf_model);

  // Create path from parameters
  path_ = createPathFromParameters(pnh);
  new_path_received_ = !path_.poses.empty();

//  std::string stability_info_save_folder;
//  if (pnh.getParam("stability_info_save_folder", stability_info_save_folder)) {
//    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
//    std::string save_path = stability_info_save_folder + "/stability_" + time_stamp + ".dat";
//    whole_body_posture_optimization::WholeBodyPostureOptimization::saveResultInfo(result, trajectories, save_path);
//  }

  // Spin and wait for new paths
  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::spinOnce();
    if (new_path_received_) {
      // Compute optimal postures
      auto result = po.findOptimalPostures(path_);
      auto trajectory = po.computeTimedTrajectory(result);

      // Print and save results
      whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
      whole_body_posture_optimization::Timing::printTimeInfos();

      new_path_received_ = false;
    }
    rate.sleep();
  }

  return 0;
}
