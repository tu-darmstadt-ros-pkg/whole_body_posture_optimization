#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

#include <whole_body_posture_optimization/timing.h>

namespace whole_body_posture_optimization {

FlipperOptimization::FlipperOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel> &robot_model, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen,
                                         const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation)
  : contact_estimation_(contact_estimation), robot_model_(robot_model), robot_shape_gen_(robot_shape_gen)
{
  if (!loadParameters(nh)) {
    ROS_ERROR_STREAM("Loading flipper parameters failed.");
  }
  if (!loadLimits()) {
    ROS_ERROR_STREAM("Loading flipper limits failed");
  }
  param_idx_to_jnt_idx_ = robot_model_->getJointIndices(free_joints_);
  start_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>("start_state", 10, true);
  result_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>("result_state", 10, true);
  contact_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("contact_points", 10, true);
  result_com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("result_com_pose", 10, true);
}

bool FlipperOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("joints", free_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'joints' in namespace'" << nh.getNamespace() << "'.");
    return false;
  }

  nh.param("regularization_weight", convex_hull_error_parameters_.regularization_weight, convex_hull_error_parameters_.regularization_weight);
  nh.param("lorentzian_sigma", convex_hull_error_parameters_.lorentzian_sigma, convex_hull_error_parameters_.lorentzian_sigma);
  nh.param("track_contact_weight", convex_hull_error_parameters_.track_contact_weight, convex_hull_error_parameters_.track_contact_weight);
  nh.param("body_contact_weight", convex_hull_error_parameters_.body_contact_weight, convex_hull_error_parameters_.body_contact_weight);

  return true;
}

FlipperOptimizationResult FlipperOptimization::optimize(const Eigen::Affine3d &robot_pose, const Eigen::Affine3d &base_to_com, const std::vector<double>& init_joint_angles,
                                                        const std::vector<double> &previous_joint_angles, const sdf_contact_estimation::ContactEstimationResult &init_contact)
{
//  ROS_INFO_STREAM("Starting flipper optimization");
  Eigen::Vector3d com_pos = base_to_com.translation();
  ROS_DEBUG_STREAM("COM position: [" << com_pos[0] << ", " << com_pos[1] << ", " << com_pos[2] << "]");
  // Global, derivative-free
  nlopt::opt opt(nlopt::GN_DIRECT_L,  free_joints_.size());
//  nlopt::opt opt(nlopt::GN_DIRECT,  robot_shape_gen_->getDof());
//    nlopt::opt opt(nlopt::GN_ORIG_DIRECT_L,  robot_shape_gen_->getDof());
//  nlopt::opt opt(nlopt::GN_CRS2_LM,  robot_shape_gen_->getDof()); // similar results to direct_l, 154
//  nlopt::opt opt(nlopt::GN_ISRES,  robot_shape_gen_->getDof()); // similar result to direct_l
//    nlopt::opt opt(nlopt::GN_ISRES,  robot_shape_gen_->getDof()); // similar result to direct_l

  // Local, derivative-free
//  nlopt::opt opt(nlopt::LN_COBYLA, free_joints_.size()); // violates bounds
//    nlopt::opt opt(nlopt::LN_NELDERMEAD,  free_joints_.size()); // violates bounds
//    nlopt::opt opt(nlopt::LN_SBPLX,  free_joints_.size()); // violates bounds

  // Global, derivative
//  nlopt::opt opt(nlopt::GD_STOGO,  robot_shape_gen_->getDof()); // good result, but sloooow (3x compared to direct_l)

  // Local, derivative
//  nlopt::opt opt(nlopt::LD_LBFGS,  robot_shape_gen_->getDof()); // didn't work at all, only 0s

  // Define parameter bounds
  opt.set_lower_bounds(lower_bounds_);
  opt.set_upper_bounds(upper_bounds_);

  // Add objective function
  std::vector<double> previous_flipper_angles;
  if (!previous_joint_angles.empty()) {
    previous_flipper_angles = jointVectorToParamVector(previous_joint_angles);
  }
  int verbosity_level = 0;
  ceres_nlopt_wrapper::CeresCostFunctionWrapper support_area_cost(
        ConvexHullAreaError::Create(free_joints_.size(), contact_estimation_, robot_pose, base_to_com, previous_flipper_angles, init_contact, convex_hull_error_parameters_),
        verbosity_level);
  support_area_cost.setName("support_area_cost");
  opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &support_area_cost);

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
//  opt.set_ftol_rel(1e-2);
  opt.set_maxeval(10);
//  opt.set_maxeval(200);
//  opt.set_maxtime(0.070);
//  opt.set_stopval(-0.510003);
//  opt.set_ftol_abs(0.01);

  std::vector<double> parameters = jointVectorToParamVector(init_joint_angles);
  // Publish start state
  robot_shape_gen_->updateShape(parameters);
  sdf_contact_estimation::publishShape(robot_shape_gen_->getShape(), start_state_pub_, robot_pose, "world", Eigen::Vector3d(1, 0, 0));

  START_TIMING("Flipper optimization");
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
  STOP_TIMING_AVG;

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Flipper optimization took " << support_area_cost.getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                  << ceres_nlopt_wrapper::vecToString(parameters));

//  ROS_INFO_STREAM("Optimization returned " << ceres_nlopt_wrapper::resultToString(nlopt_result));
//  ROS_INFO_STREAM("Result: " << ceres_nlopt_wrapper::vecToString(parameters));

  FlipperOptimizationResult result;
  result.flipper_angles = parameters;
  result.joint_state = init_joint_angles;
  for (unsigned int i = 0; i < parameters.size(); i++) {
    result.joint_state[param_idx_to_jnt_idx_[i]] = parameters[i];
  }

  // Estimating again for debug output and result
  contact_estimation_->updateJointStates(parameters);
  result.contact_result = contact_estimation_->estimateSupportPolygon(robot_pose, base_to_com, init_contact);
  sdf_contact_estimation::publishShape(robot_shape_gen_->getShape(), result_state_pub_, result.contact_result.estimated_pose, "world", Eigen::Vector3d(1, 1, 0));
  visualization_msgs::MarkerArray contact_points_array_msg;
  sdf_contact_estimation::createPointMarkerArray(result.contact_result.contact_points, contact_points_array_msg, "world", "contact_points", Eigen::Vector3d(1, 0.5, 0), 0.03);
  sdf_contact_estimation::createPointMarkerArray(result.contact_result.convex_hull_points, contact_points_array_msg, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
  Eigen::Affine3d result_com_pose = result.contact_result.estimated_pose * base_to_com;
  sdf_contact_estimation::publishPose(result_com_pub_, result_com_pose, "world");
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(result.contact_result.convex_hull_points, result_com_pose.translation());
//  double stability_min = *std::min_element(stabilities.begin(), stabilities.end());
//  ROS_INFO_STREAM("Stability after flipper optimization: " << stability_min);
  createConvexHullMarkerArray(result.contact_result.convex_hull_points, stabilities, contact_points_array_msg);
  sdf_contact_estimation::deleteAllMarkers(contact_points_pub_);
  contact_points_pub_.publish(contact_points_array_msg);

  return result;
}

sdf_contact_estimation::ContactEstimationResult FlipperOptimization::computeContactEstimation(const std::vector<double> &joint_angles, const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const sdf_contact_estimation::ContactEstimationResult &last_result)
{
  std::vector<double> flipper_angles = jointVectorToParamVector(joint_angles);
  contact_estimation_->updateJointStates(flipper_angles);
  return contact_estimation_->estimateSupportPolygon(pose, base_to_com, last_result);
}

sdf_contact_estimation::ContactEstimationResult FlipperOptimization::computeContactEstimation(const std::vector<double> &joint_angles, const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com)
{
  std::vector<double> flipper_angles = jointVectorToParamVector(joint_angles);
  contact_estimation_->updateJointStates(flipper_angles);
  return contact_estimation_->estimateSupportPolygon(pose, base_to_com);
}

std::vector<double> FlipperOptimization::jointVectorToParamVector(const std::vector<double> &joint_vec)
{
  std::vector<double> parameters(free_joints_.size());
  for (unsigned int i = 0; i < parameters.size(); i++) {
    parameters[i] = joint_vec[param_idx_to_jnt_idx_[i]];
  }
  return parameters;
}

bool FlipperOptimization::loadLimits()
{
  for (const std::string& link: free_joints_) {
    double upper, lower;
    if (!robot_model_->getJointLimits(link, lower, upper)) {
      return false;
    }
    upper_bounds_.push_back(upper);
    lower_bounds_.push_back(lower);
  }
  return true;
}

}
