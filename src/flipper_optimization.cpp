#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

namespace whole_body_posture_optimization {

FlipperOptimization::FlipperOptimization(const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen,
                                         const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation)
  : robot_shape_gen_(robot_shape_gen), contact_estimation_(contact_estimation)
{}

std::vector<double> FlipperOptimization::optimize(const Eigen::Affine3d &robot_pose, const Eigen::Affine3d &base_to_com, const std::vector<double>& init_position)
{
  ROS_INFO_STREAM("Starting flipper optimization");
  // Global, derivative-free
//  nlopt::opt opt(nlopt::GN_DIRECT_L,  robot_shape_gen_->getDof());
//    nlopt::opt opt(nlopt::GN_ORIG_DIRECT_L,  robot_shape_gen_->getDof());
//  nlopt::opt opt(nlopt::GN_CRS2_LM,  robot_shape_gen_->getDof()); // similar results to direct_l, 154
//  nlopt::opt opt(nlopt::GN_ISRES,  robot_shape_gen_->getDof()); // similar result to direct_l
//    nlopt::opt opt(nlopt::GN_ISRES,  robot_shape_gen_->getDof()); // similar result to direct_l

  // Local, derivative-free
//  nlopt::opt opt(nlopt::LN_COBYLA,  robot_shape_gen_->getDof()); // similar result to direct_l
    nlopt::opt opt(nlopt::LN_NELDERMEAD,  robot_shape_gen_->getDof()); // similar result to direct_l

  // Global, derivative
//  nlopt::opt opt(nlopt::GD_STOGO,  robot_shape_gen_->getDof()); // good result, but sloooow (3x compared to direct_l)


  // Local, derivative
//  nlopt::opt opt(nlopt::LD_LBFGS,  robot_shape_gen_->getDof()); // didn't work at all, only 0s



  // Define parameter bounds
  opt.set_lower_bounds(robot_shape_gen_->getLowerLimits());
  opt.set_upper_bounds(robot_shape_gen_->getUpperLimits());

  // Add objective function
  CeresCostFunctionWrapper support_area_cost(ConvexHullAreaError::Create(robot_shape_gen_, contact_estimation_, robot_pose, base_to_com), 1);
  opt.set_min_objective(CeresCostFunctionWrapper::wrap, &support_area_cost);

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
  opt.set_ftol_rel(1e-3);
//  opt.set_maxeval(10);
//  opt.set_stopval(-0.510003);
//  opt.set_ftol_abs(0.01);

  // Fill in initial joint state (from previous time step)
  std::vector<double> parameters = init_position;

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

  // Estimating again for debug output
  std::vector<double> gradient;
  support_area_cost(parameters, gradient);


  return parameters;
}

}
