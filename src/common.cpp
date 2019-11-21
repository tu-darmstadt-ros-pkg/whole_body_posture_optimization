#include <whole_body_posture_optimization/common.h>

#include <ctime>
#include <ceres/gradient_checker.h>

namespace whole_body_posture_optimization {

void updateMoveitRobotState(moveit::core::RobotState &state, const Eigen::Affine3d &pose)
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
  for (unsigned int i = 0; i < length; i++) {
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
  std::vector<double> x = std::vector<double>(cost_function->parameter_block_sizes()[0], 0);
  double const* x_ptr = &x[0];
  double const *const *parameters_ptr = &x_ptr;
  if (!gradient_checker.Probe(parameters_ptr, 1e-5, &probe_results)) {
    ROS_ERROR_STREAM("Gradient check failed. Max relative error: " << probe_results.maximum_relative_error);
    ROS_ERROR_STREAM("Gradient: " << probe_results.jacobians[0]);
    ROS_ERROR_STREAM("Numeric gradient: " << probe_results.numeric_jacobians[0]);
  }
}

}
