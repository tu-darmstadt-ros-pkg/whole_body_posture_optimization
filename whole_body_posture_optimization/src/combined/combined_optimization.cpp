#include <whole_body_posture_optimization/combined/combined_optimization.h>

#include <whole_body_posture_optimization/combined/sdf_contact_constraint_no_gradient.h>
#include <whole_body_posture_optimization/combined/self_contact_constraint_no_gradient.h>

#include <whole_body_posture_optimization/timing.h>

namespace whole_body_posture_optimization {

CombinedOptimization::CombinedOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model,
                                 const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation)
  : robot_model_(robot_model), contact_estimation_(contact_estimation), enabled_(true), algorithm_("")
{
  if (!loadParameters(nh)) {
    ROS_ERROR_STREAM("Loading parameters failed.");
  }
  if (!loadLimits(free_joints_)) {
    ROS_ERROR_STREAM("Loading limits failed");
  }
  param_idx_to_jnt_idx_ = robot_model_->getActuatedJointIndices(free_joints_);

  // Load Allowed Collision Matrix (ACM)
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());
  acm_ = planning_scene->getAllowedCollisionMatrix();

  // Compute robot tsdf
  robot_tsdf_ = robot_model_->computeTsdfFromRobotModel(static_link_names_, self_collision_tsdf_truncation_distance_, 0.05);

  //Publisher
  result_com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("result_com", 10, true);
  convex_hull_pub_ = nh.advertise<visualization_msgs::MarkerArray>("convex_hull", 10, true);
  robot_tsdf_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("robot_tsdf", 10, true);
}

bool CombinedOptimization::loadParameters(const ros::NodeHandle &nh)
{
  bool success = true;
  if(!nh.getParam("flipper_joints", free_flipper_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'flipper_joints' in namespace'" << nh.getNamespace() << "'.");
    success = false;
  }
  if(!nh.getParam("arm_joints", free_arm_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'arm_joints' in namespace'" << nh.getNamespace() << "'.");
    success = false;
  }
  free_joints_.resize(free_flipper_joints_.size() + free_arm_joints_.size());
  std::copy(free_flipper_joints_.begin(), free_flipper_joints_.end(), free_joints_.begin());
  std::copy(free_arm_joints_.begin(), free_arm_joints_.end(), free_joints_.begin() + free_flipper_joints_.size());

  nh.param("environment_collision_default_padding", environment_collision_default_padding_, 0.0);
  nh.param<std::map<std::string, double>>("environment_collision_padding", environment_collision_padding_map_, environment_collision_padding_map_);

  // Collision
  nh.param<std::vector<std::string>>("static_link_names", static_link_names_, static_link_names_);
  nh.param("self_collision_padding", self_collision_padding_, 0.0);
  nh.param("self_collision_tsdf_padding", self_collision_tsdf_padding_, 0.0);
  nh.param("self_collision_tsdf_truncation_distance", self_collision_tsdf_truncation_distance_, 0.4);
  nh.param("enabled", enabled_, enabled_);
  nh.param<std::string>("algorithm", algorithm_, "LD_SLSQP");

  std::vector<double> exp_parameters = {1.6, -1.7, 0.0};
  nh.param("exp_parameters", exp_parameters, exp_parameters);
  if (exp_parameters.size() != 3) {
    ROS_ERROR_STREAM("Invalid size for parameter 'exp_parameter'. Got " << exp_parameters.size() << " expected 3");
    success = false;
    stability_cost_parameters_.exp_parameters[0] = stability_cost_parameters_.exp_parameters[1] = stability_cost_parameters_.exp_parameters[2] = 0;
  } else {
    std::copy(begin(exp_parameters), end(exp_parameters), stability_cost_parameters_.exp_parameters);
  }
  nh.param("lorentzian_sigma", stability_cost_parameters_.lorentzian_sigma, stability_cost_parameters_.lorentzian_sigma);
  nh.param("regularization_weight", stability_cost_parameters_.regularization_weight, stability_cost_parameters_.regularization_weight);

  return success;
}

double CombinedOptimization::getJointPadding(std::string joint_name, CollisionType collision_type)
{
  std::map<std::string, double>* padding_map;
  switch(collision_type) {
    case CollisionType::ENVIRONMENT:
      padding_map = &environment_collision_padding_map_;
      break;
    default:
      padding_map = nullptr;
  }
  bool found;
  double padding = 0.0;
  if (padding_map) {
    try {
      padding = padding_map->at(joint_name);
      found = true;
      ROS_DEBUG_STREAM("Using padding " << padding << " for link '" << joint_name << "'.");
    }  catch (const std::out_of_range&) {
      found = false;
    }
  } else {
    found = false;
  }
  if (found) {
    return padding;
  } else {
    return getDefaultJointPadding(collision_type);
  }
}

double CombinedOptimization::getDefaultJointPadding(CollisionType collision_type)
{
  switch (collision_type) {
    case ENVIRONMENT: return environment_collision_default_padding_;
    case SELF_COLLISION: return self_collision_padding_;
    case SELF_COLLISION_TSDF: return self_collision_tsdf_padding_;
    default: ROS_WARN_STREAM("Unknown collision type. Defaulting to zero padding.");
  }
  return 0.0;
}

bool CombinedOptimization::loadLimits(const std::vector<std::string>& joints)
{
  for (const std::string& link: joints) {
    double upper, lower;
    if (!robot_model_->getJointLimits(link, lower, upper)) {
      return false;
    }
    upper_bounds_.push_back(upper);
    lower_bounds_.push_back(lower);
  }
  return true;
}

std::vector<double> CombinedOptimization::jointVectorToParamVector(const std::vector<double> &joint_vec)
{
  std::vector<double> parameters(free_joints_.size());
  for (unsigned int i = 0; i < parameters.size(); i++) {
    parameters[i] = joint_vec[param_idx_to_jnt_idx_[i]];
  }
  return parameters;
}

CombinedOptimizationResult CombinedOptimization::optimize(const std::vector<double>& initial_state, const Eigen::Isometry3d &robot_pose, const std::vector<double>& previous_state,
                                                          const sdf_contact_estimation::ContactEstimationResult& init_contact)
{
  nlopt::opt opt(ceres_nlopt_wrapper::stringToAlgorithm(algorithm_), free_joints_.size());


  // Define parameter bounds
  opt.set_lower_bounds(lower_bounds_);
  opt.set_upper_bounds(upper_bounds_);

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
//  opt.set_ftol_rel(1e-5);
//  opt.set_xtol_abs(0.0001);
  opt.set_maxeval(300);
//  opt.set_maxtime(0.1);
  opt.set_ftol_abs(0.00001);

//  ROS_INFO_STREAM("lower bounds: " << ceres_nlopt_wrapper::vecToString(lower_bounds_));
//  ROS_INFO_STREAM("upper bounds: " << ceres_nlopt_wrapper::vecToString(upper_bounds_));

  // Add objective function
  Eigen::Isometry3d* result_robot_pose = new Eigen::Isometry3d();
  *result_robot_pose = Eigen::Isometry3d::Identity();
  std::vector<double> previous_parameters;
  if (!previous_state.empty()) {
    previous_parameters = jointVectorToParamVector(previous_state);
  }

  ceres_nlopt_wrapper::CeresCostFunctionWrapper stability_cost(
        CombinedStabilityCost::Create(robot_model_, contact_estimation_, free_joints_, free_flipper_joints_, stability_cost_parameters_, initial_state, previous_parameters, robot_pose,
                                      init_contact, result_robot_pose), 0);
  stability_cost.setName("stability_cost");
  stability_cost.enableInfCheck(true);
  stability_cost.enableNanCheck(true);
  opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &stability_cost);

  // Add constraints
  std::vector<std::string> collision_group_names = robot_model_->getCollisionGroupNames();
  std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> environment_constraints;
  START_TIMING("Env. collision const. creation")
  // Environment collision constraints
  for (const std::string& collision_link: collision_group_names) {
    ROS_DEBUG_STREAM("Creating environment collision constraints for link: " << collision_link);
    for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
      CollisionObject collision_object(sphere, collision_link, getJointPadding(collision_link, ENVIRONMENT));
      ceres::CostFunction* cost_function = SdfContactConstraintNoGradient::Create(
            robot_model_, param_idx_to_jnt_idx_, initial_state, result_robot_pose, contact_estimation_->getSdf(), collision_object);
      std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function);
      wrapper->setName("constraint_environment_" + collision_link);
      environment_constraints.push_back(wrapper);
      opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
    }
  }
//  ROS_INFO_STREAM("Created " << environment_constraints.size() << " environment collision constraints.");
  STOP_TIMING_AVG

//  // self-collision tsdf constraints
  START_TIMING("Self-collision tsdf const. creation")
  sdf_contact_estimation::publishTsdfSlice(robot_tsdf_pub_, robot_tsdf_, robot_pose);
  sdf_contact_estimation::SdfWrapper robot_base_tsdf;
  robot_base_tsdf.setTsdfMap(robot_tsdf_, self_collision_tsdf_truncation_distance_);

  std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> self_collision_tsdf_constraints;
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  for (const std::string& collision_link: collision_group_names) {
    if (collision_link == "arm_link_0" || collision_link == "arm_link_1") { // arm_link_0 is always in collision, TODO remove hardcoded links
      continue;
    }
    for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
      CollisionObject collision_object(sphere, collision_link, getJointPadding(collision_link, SELF_COLLISION_TSDF));
      ceres::CostFunction* cost_function = SdfContactConstraintNoGradient::Create(
            robot_model_, param_idx_to_jnt_idx_, initial_state, &identity, robot_base_tsdf, collision_object);
      std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function);
      wrapper->setName("constraint_self_collision_tsdf_" + collision_link);
      self_collision_tsdf_constraints.push_back(wrapper);
      opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
    }
  }
//  ROS_INFO_STREAM("Created " << self_collision_tsdf_constraints.size() << " self-collision tsdf constraints.");
  STOP_TIMING_AVG

  // self-collision distance constraints
  START_TIMING("Self-collision const. creation")
  std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> self_collision_constraints;
  // Create constraint for each unique link-pair
  for (unsigned int i = 0; i < collision_group_names.size()-1; i++) {
    std::string link_name_1 = collision_group_names[i];
    for (unsigned int j = i+1; j < collision_group_names.size(); j++) {
      std::string link_name_2 = collision_group_names[j];

      // Check acm
      collision_detection::AllowedCollision::Type allowed_collision_type;
      if (acm_.getAllowedCollision(link_name_1, link_name_2, allowed_collision_type)) {
        if (allowed_collision_type == collision_detection::AllowedCollision::ALWAYS) {
          ROS_DEBUG_STREAM("Skipping collision between '" << link_name_1 << "' and '" << link_name_2 << "'.");
          continue;
        }
      }

      // Constraint for each sphere-pair
      for (const collision_detection::CollisionSphere& sphere1: robot_model_->getCollisionSpheres(link_name_1)) {
        for (const collision_detection::CollisionSphere& sphere2: robot_model_->getCollisionSpheres(link_name_2)) {
          CollisionObject obj1(sphere1, link_name_1, getJointPadding(link_name_1, SELF_COLLISION));
          CollisionObject obj2(sphere2, link_name_2, getJointPadding(link_name_2, SELF_COLLISION));
          ceres::CostFunction* cost_function = SelfContactConstraintNoGradient::Create(
                robot_model_, param_idx_to_jnt_idx_, initial_state, obj1, obj2);
          std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function);
          wrapper->setName("constraint_self_collision_" + link_name_1 + "_vs_" + link_name_2);
          self_collision_constraints.push_back(wrapper);
          opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
        }
      }
    }
  }
//  ROS_INFO_STREAM("Created " << self_collision_constraints.size() << " self-collision constraints.");
  STOP_TIMING_AVG

  // Fill in initial joint state (from previous time step)
  std::vector<double> parameters = jointVectorToParamVector(initial_state);

  START_TIMING("Combined optimization")
  double minf;
  auto start = std::chrono::high_resolution_clock::now();
  nlopt::result nlopt_result;
  try {
    if (enabled_) {
      nlopt_result = opt.optimize(parameters, minf);
    }
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
  STOP_TIMING_AVG

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Combined optimization took " << stability_cost.getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                  << ceres_nlopt_wrapper::vecToString(parameters));

//  ROS_INFO_STREAM("Optimization returned " << ceres_nlopt_wrapper::resultToString(nlopt_result));

  CombinedOptimizationResult result;
  result.joint_angles = initial_state;
  for (unsigned int i = 0; i < parameters.size(); i++) {
    result.joint_angles[param_idx_to_jnt_idx_[i]] = parameters[i];
  }
  // Compute com
  robot_model_->computeCenterOfMass(result.joint_angles, result.center_of_mass);

  // Compute and publish result stabilities
  std::vector<double> flipper_angles(parameters.begin(), parameters.begin() + free_flipper_joints_.size());
  contact_estimation_->updateJointStates(flipper_angles);
  Eigen::Isometry3d base_to_com; base_to_com = Eigen::Translation3d(result.center_of_mass);
  result.contact_result = contact_estimation_->estimateSupportPolygon(robot_pose, base_to_com, init_contact);
  visualization_msgs::MarkerArray convex_hull_array_msg;
  sdf_contact_estimation::createPointMarkerArray(result.contact_result.convex_hull_points, convex_hull_array_msg, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
  sdf_contact_estimation::createConvexHullMarkerArrayWithStabilities(result.contact_result.convex_hull_points, result.contact_result.stabilities, convex_hull_array_msg);
  sdf_contact_estimation::deleteAllMarkers(convex_hull_pub_);
  convex_hull_pub_.publish(convex_hull_array_msg);

  // Publish com
  Eigen::Isometry3d com_pose = Eigen::Isometry3d::Identity();
  com_pose.translation() = result.contact_result.estimated_pose * result.center_of_mass;
  sdf_contact_estimation::publishPose(result_com_pub_, com_pose, "world");

  *result_robot_pose = result.contact_result.estimated_pose;

  // Check for collisions in solution
  std::vector<ceres_nlopt_wrapper::ConstraintViolation> constraint_violations;
  *result_robot_pose = result.contact_result.estimated_pose;
  if (!ceres_nlopt_wrapper::checkConstraints(parameters, environment_constraints, 0.001, constraint_violations)) {
    ROS_WARN_STREAM("Detected environment collision!");
    for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
      ROS_WARN_STREAM(violation.toString());
    }
  }
  if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_tsdf_constraints, 0.001, constraint_violations)) {
    ROS_WARN_STREAM("Detected self-collision with tsdf!");
    for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
      ROS_WARN_STREAM(violation.toString());
    }
  }
  if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_constraints, 0.001, constraint_violations)) {
    ROS_WARN_STREAM("Detected self-collision!");
    for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
      ROS_WARN_STREAM(violation.toString());
    }
  }

  // DEBUG print environment constraint values:
//  for (const std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>& c: environment_constraints) {
//    if (!c) {
//      ROS_ERROR_STREAM("Constraint is null.");
//      continue;
//    }
//    std::vector<double> gradient(parameters.size());
//    double cost = c->operator ()(parameters, gradient);
//    ROS_INFO_STREAM(c->getName() << ":" << cost);
//  }

  delete result_robot_pose;

  return result;
}

sdf_contact_estimation::ContactEstimationResult CombinedOptimization::computeContactEstimation(const std::vector<double>& joint_angles, const Eigen::Isometry3d& pose,
                                                                                               const Eigen::Isometry3d& base_to_com)
{
  std::vector<double> parameters = jointVectorToParamVector(joint_angles);
  ROS_INFO_STREAM("parameter size: " << parameters.size());
  std::vector<double> flipper_angles(parameters.begin(), parameters.begin() + free_flipper_joints_.size());
  ROS_INFO_STREAM("flippers size: " << free_flipper_joints_.size());
  ROS_INFO_STREAM("flipper angles size: " << flipper_angles.size());
  contact_estimation_->updateJointStates(flipper_angles);
  return contact_estimation_->estimateSupportPolygon(pose, base_to_com);
}

}
