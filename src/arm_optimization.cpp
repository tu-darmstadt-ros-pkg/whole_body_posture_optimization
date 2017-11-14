#include <whole_body_posture_optimization/arm_optimization.h>

namespace whole_body_posture_optimization {

ArmOptimization::ArmOptimization(const std::string &robot_description)
  : robot_description_(robot_description)
{
  com_calculation_ = std::make_shared<CenterOfMass>(robot_description_);
  stability_cost_ = std::make_shared<CeresCostFunctionWrapper>(StabilityError::Create(com_calculation_));
}

ArmOptResult ArmOptimization::optimize(const std::vector<double>& initial_pose, const std::vector<Eigen::Vector3d> &contact_points)
{
  nlopt::opt opt(nlopt::LD_SLSQP,  3);

  // Define parameter bounds
  // TODO read from urdf
  opt.set_lower_bounds(lower_bounds_);
  opt.set_upper_bounds(upper_bounds_);

  // Add objective function
  opt.set_min_objective(CeresCostFunctionWrapper::wrap, &stability_cost_);

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
  opt.set_ftol_rel(1e-6);
  opt.set_maxeval(50);
//  opt.set_ftol_abs(0.01);

  // Fill in initial joint state (from previous time step)
  std::vector<double> parameters = initial_pose;

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
  com_calculation_->computeCenterOfMass(parameters, result.center_of_mass);

  return result;
}

}
