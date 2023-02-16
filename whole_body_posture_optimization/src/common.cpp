#include <whole_body_posture_optimization/common.h>

#include <eigen_conversions/eigen_msg.h>
#include <ctime>
#include <ceres/gradient_checker.h>
#include <moveit/robot_state/conversions.h>


namespace whole_body_posture_optimization {

void deleteAllMarkers(const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.header.frame_id = "world"; // this has to be set correctly for the deletion to work
  array.markers.push_back(marker);
  pub.publish(array);
}

void updateMoveitRobotState(moveit::core::RobotState &state, const Eigen::Isometry3d &pose)
{
  sensor_msgs::JointState virtual_link_joint_states;
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_x");
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_y");
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_z");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_x");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_y");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_z");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_w");

  virtual_link_joint_states.position.resize(7);
  virtual_link_joint_states.position[0] = pose.translation().x();
  virtual_link_joint_states.position[1] = pose.translation().y();
  virtual_link_joint_states.position[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.linear());
  virtual_link_joint_states.position[3] = quat.x();
  virtual_link_joint_states.position[4] = quat.y();
  virtual_link_joint_states.position[5] = quat.z();
  virtual_link_joint_states.position[6] = quat.w();
  moveit::core::jointStateToRobotState(virtual_link_joint_states, state);
}

std::string line(int length) {
  std::string l = "";
  for (int i = 0; i < length; i++) {
    l += "-";
  }
  return l;
}

std::string getTimestamp()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

bool checkGradient(const ceres::CostFunction* cost_function)
{
  ceres::NumericDiffOptions numeric_diff_options;
  ceres::GradientChecker gradient_checker(cost_function, nullptr, numeric_diff_options);
  ceres::GradientChecker::ProbeResults probe_results;
  std::vector<double> x = std::vector<double>(static_cast<size_t>(cost_function->parameter_block_sizes()[0]), 0);
  double const* x_ptr = &x[0];
  double const *const *parameters_ptr = &x_ptr;
  if (!gradient_checker.Probe(parameters_ptr, 1e-5, &probe_results)) {
    ROS_ERROR_STREAM("Gradient check failed. Max relative error: " << probe_results.maximum_relative_error);
    ROS_ERROR_STREAM("Gradient: " << probe_results.jacobians[0]);
    ROS_ERROR_STREAM("Numeric gradient: " << probe_results.numeric_jacobians[0]);
    return false;
  }
  return true;
}

std::string poseToString(const Eigen::Isometry3d& pose)
{
  Eigen::Vector3d rpy = sdf_contact_estimation::rotToNormalizedRpy(pose.linear());
  Eigen::Vector3d xyz = pose.translation();
  std::string s = "[" + std::to_string(xyz(0)) + ", " + std::to_string(xyz(1)) + ", " + std::to_string(xyz(2)) + ", "
      + std::to_string(rpy(0)) + ", " + std::to_string(rpy(1)) + ", " + std::to_string(rpy(2)) + "]";
  return s;
}

std::string robotTrajectoryToString(const robot_trajectory::RobotTrajectory& trajectory)
{
  std::stringstream ss;
  ss << "[";
  for (unsigned int j = 0; j < trajectory.getRobotModel()->getVariableCount(); ++j) {
    ss << trajectory.getRobotModel()->getVariableNames()[j] << ", ";
  }
  ss << "]" << std::endl;
  for (unsigned int i = 0; i < trajectory.getWayPointCount(); ++i) {
    const robot_state::RobotState& state = trajectory.getWayPoint(i);
    ss << i << ": ";
    ss << "t = " << trajectory.getWayPointDurationFromStart(i);
    // Position
    ss << " [";
    for (size_t j = 0; j < state.getVariableCount(); ++j) {
      ss << state.getVariablePosition(static_cast<int>(j)) << ", ";
    }
    ss << "]" << std::endl;
    // Velocity
    ss << " [";
    for (size_t j = 0; j < state.getVariableCount(); ++j) {
      ss << state.getVariableVelocity(static_cast<int>(j)) << ", ";
    }
    ss << "]" << std::endl;
    // Acceleration
    ss << " [";
    for (size_t j = 0; j < state.getVariableCount(); ++j) {
      ss << state.getVariableAcceleration(static_cast<int>(j)) << ", ";
    }
    ss << "]" << std::endl;
  }
  return ss.str();
}

double computeTrajectoryDuration(const std::vector<robot_trajectory::RobotTrajectory>& trajectories) {
  double trajectory_duration = 0.0;
  for (const auto& trajectory: trajectories) {
    trajectory_duration += trajectory.getDuration();
  }
  return trajectory_duration;
}

Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& quaternion, const std::string& convention) {
  double temp[5];
  if (convention == "zyx") {
    temp[0] =  2*(quaternion.x() * quaternion.y() + quaternion.w() * quaternion.z());
    temp[1] =     quaternion.w() * quaternion.w() + quaternion.x() * quaternion.x() - quaternion.y() *quaternion.y() - quaternion.z() * quaternion.z();
    temp[2] = -2*(quaternion.x() * quaternion.z() - quaternion.w() * quaternion.y());
    temp[3] =  2*(quaternion.y() * quaternion.z() + quaternion.w() * quaternion.x());
    temp[4] =     quaternion.w() * quaternion.w() - quaternion.x() * quaternion.x() - quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z();
  } else if (convention == "xyz") {
    temp[0] =  -2*(quaternion.y() * quaternion.z() - quaternion.w() * quaternion.x());
    temp[1] =      quaternion.w() * quaternion.w() - quaternion.x() * quaternion.x() - quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z();
    temp[2] =   2*(quaternion.x() * quaternion.z() + quaternion.w() * quaternion.y());
    temp[3] =  -2*(quaternion.x() * quaternion.y() - quaternion.w() * quaternion.z());
    temp[4] =      quaternion.w() * quaternion.w() + quaternion.x() * quaternion.x() - quaternion.y() * quaternion.y() - quaternion.z() * quaternion.z();
  } else {
    throw std::invalid_argument("not supported");
  }

  Eigen::Vector3d angles;
  angles(0) = atan2(temp[0], temp[1]);
  angles(1) = asin(temp[2]);
  angles(2) = atan2(temp[3], temp[4]);

  return angles;
}

std::vector<double> getVariablePositions(const robot_state::RobotStateConstPtr& state, const std::vector<std::string>& variables) {
  std::vector<double> positions;
  positions.reserve(variables.size());
  for (const std::string& variable: variables) {
    positions.push_back(state->getVariablePosition(variable));
  }
  return positions;
}

}
