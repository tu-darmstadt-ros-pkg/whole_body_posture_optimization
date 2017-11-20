#include <whole_body_posture_optimization/arm/arm_optimization.h>

namespace whole_body_posture_optimization {

ArmOptimization::ArmOptimization(const ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model)
{
  ROS_INFO_STREAM("Creating arm optimization");
  loadParameters(nh);
  robot_model_ = robot_model;
}

bool ArmOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("joints", free_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'joints' in namespace'" << nh.getNamespace() << "'.");
    return false;
  }
}

ArmOptResult ArmOptimization::optimize(const std::vector<double>& initial_pose, const std::vector<Eigen::Vector3d> &contact_points)
{
  ROS_INFO_STREAM("Starting optimization");
  nlopt::opt opt(nlopt::LD_SLSQP,  free_joints_.size());

  // Define parameter bounds
  // TODO read from urdf
  opt.set_lower_bounds(lower_bounds_);
  opt.set_upper_bounds(upper_bounds_);

  // Add objective function
  CeresCostFunctionWrapper stability_cost_(StabilityError::Create(robot_model_, free_joints_, contact_points, initial_pose));
  opt.set_min_objective(CeresCostFunctionWrapper::wrap, &stability_cost_);

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
  opt.set_ftol_rel(1e-6);
  opt.set_maxeval(50);
//  opt.set_ftol_abs(0.01);

  // Fill in initial joint state (from previous time step)
  std::vector<double> parameters(free_joints_.size());
  std::vector<unsigned int> param_idx_to_jnt_idx = robot_model_->getBodyIndices(free_joints_);
  for (unsigned int i = 0; i < parameters.size(); i++) {
    parameters[i] = initial_pose[param_idx_to_jnt_idx[i]];
  }

  double minf;
  auto start = std::chrono::high_resolution_clock::now();
  nlopt::result nlopt_result;
  try {
    nlopt_result = opt.optimize(parameters, minf);
  }
  catch (std::invalid_argument e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  catch (nlopt::roundoff_limited e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Optimization took " << elapsed.count()*1000.0 << " ms.");

  ROS_INFO_STREAM("Optimization returned " << resultToString(nlopt_result));
  ROS_INFO_STREAM("Result: " << vecToString(parameters));

  ArmOptResult result;
  result.joint_angles = parameters;
  robot_model_->computeCenterOfMass(parameters, result.center_of_mass);

  return result;
}

}
