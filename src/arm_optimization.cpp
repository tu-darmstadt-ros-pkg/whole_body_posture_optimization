#include <whole_body_posture_optimization/arm/arm_optimization.h>

namespace whole_body_posture_optimization {

ArmOptimization::ArmOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model,
                                 const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation)
  : robot_model_(robot_model), contact_estimation_(contact_estimation)
{
  if (!loadParameters(nh)) {
    ROS_ERROR_STREAM("Loading arm parameters failed.");
  }
  if (!loadLimits()) {
    ROS_ERROR_STREAM("Loading arm limits failed");
  }
  param_idx_to_jnt_idx_ = robot_model_->getJointIndices(free_joints_);
  result_com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("result_com", 10, true);
  convex_hull_pub_ = nh.advertise<visualization_msgs::MarkerArray>("convex_hull", 10, true);
}

bool ArmOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("joints", free_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'joints' in namespace'" << nh.getNamespace() << "'.");
    return false;
  }
  return true;
}

bool ArmOptimization::loadLimits()
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

std::vector<double> ArmOptimization::jointVectorToParamVector(const std::vector<double> &joint_vec)
{
  std::vector<double> parameters(free_joints_.size());
  for (unsigned int i = 0; i < parameters.size(); i++) {
    parameters[i] = joint_vec[param_idx_to_jnt_idx_[i]];
  }
  return parameters;
}

ArmOptimizationResult ArmOptimization::optimize(const std::vector<double>& initial_angles, const std::vector<Eigen::Vector3d> &contact_points, const Eigen::Affine3d& robot_pose,
                                                const std::vector<double>& previous_joint_angles)
{
  ROS_INFO_STREAM("Starting arm optimization");
  nlopt::opt opt(nlopt::LD_SLSQP,  free_joints_.size());
//  nlopt::opt opt(nlopt::LD_MMA,  free_joints_.size());
//    nlopt::opt opt(nlopt::LN_COBYLA,  free_joints_.size());


  // Define parameter bounds
  opt.set_lower_bounds(lower_bounds_);
  opt.set_upper_bounds(upper_bounds_);

  ROS_INFO_STREAM("lower bounds: " << ceres_nlopt_wrapper::vecToString(lower_bounds_));
  ROS_INFO_STREAM("upper bounds: " << ceres_nlopt_wrapper::vecToString(upper_bounds_));

  // Add objective function
  std::vector<double> previous_parameters = jointVectorToParamVector(previous_joint_angles);
  ceres_nlopt_wrapper::CeresCostFunctionWrapper stability_cost_(StabilityError::Create(robot_model_, free_joints_, contact_points, initial_angles, previous_parameters, robot_pose), 0);
  opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &stability_cost_);

  // Add constraints
  // TODO move constraints creation to constructor
  std::vector<int> param_idx_to_chain_idx = robot_model_->getParamToChainIndexMapping(free_joints_);
  std::vector<unsigned int> chain_idx_to_jnt_idx = robot_model_->getChainToJointIndexMapping();
  std::vector<double> default_chain_angles(chain_idx_to_jnt_idx.size());
  for (unsigned int chain_idx = 0; chain_idx < default_chain_angles.size(); chain_idx++) {
    default_chain_angles[chain_idx] = initial_angles[chain_idx_to_jnt_idx[chain_idx]];
  }
  std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> constraints;
  for (const std::string& collision_link: robot_model_->getCollisionGroupNames()) {
//    if (collision_link != "gripper_roll_link") {
//      continue;
//    }
    ROS_DEBUG_STREAM("Creating collision constraints for link: " << collision_link);
    for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
      ceres::CostFunction* cost_function = ContactConstraint::Create(robot_model_, free_joints_, param_idx_to_chain_idx, default_chain_angles, robot_pose, contact_estimation_, sphere, collision_link);
      auto wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function);
      constraints.push_back(wrapper);
      opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
    }
  }
  ROS_INFO_STREAM("Created " << constraints.size() << " collision constraints.");

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
  opt.set_ftol_rel(1e-3);
  opt.set_maxeval(100);
//  opt.set_ftol_abs(0.01);

  // Fill in initial joint state (from previous time step)
  std::vector<double> parameters = jointVectorToParamVector(initial_angles);

  double minf;
  auto start = std::chrono::high_resolution_clock::now();
  nlopt::result nlopt_result;
  try {
    nlopt_result = opt.optimize(parameters, minf);
  }
  catch (std::invalid_argument e) {
    ROS_ERROR_STREAM("Optimization failed with invalid argument error: " << e.what());
  }
  catch (nlopt::roundoff_limited e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Arm optimization took " << elapsed.count()*1000.0 << " ms.");

  ROS_INFO_STREAM("Optimization returned " << ceres_nlopt_wrapper::resultToString(nlopt_result));
  ROS_INFO_STREAM("Result: " << ceres_nlopt_wrapper::vecToString(parameters));

  // Check for collisions in solution  
  if (!ceres_nlopt_wrapper::checkConstraints(parameters, constraints, 0.001)) {
    ROS_WARN_STREAM("Detected collision!!");
  }

  ArmOptimizationResult result;
  result.joint_angles = initial_angles;
  for (unsigned int i = 0; i < parameters.size(); i++) {
    result.joint_angles[param_idx_to_jnt_idx_[i]] = parameters[i];
  }
  // Compute and publish result com
  robot_model_->computeCenterOfMass(result.joint_angles, result.center_of_mass);
  Eigen::Affine3d com_pose = Eigen::Affine3d::Identity();
  com_pose.translation() = robot_pose * result.center_of_mass;
  sdf_contact_estimation::publishPose(result_com_pub_, com_pose, "world");

  // Compute and publish result stabilities
  visualization_msgs::MarkerArray convex_hull_array_msg;
  sdf_contact_estimation::createPointMarkerArray(contact_points, convex_hull_array_msg, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(contact_points, com_pose.translation());
  double stability_min = *std::min_element(stabilities.begin(), stabilities.end());
  ROS_INFO_STREAM("Stability after arm optimization: " << stability_min);
  createConvexHullMarkerArray(contact_points, stabilities, convex_hull_array_msg);
  sdf_contact_estimation::deleteAllMarkers(convex_hull_pub_);
  convex_hull_pub_.publish(convex_hull_array_msg);

  return result;
}

}
