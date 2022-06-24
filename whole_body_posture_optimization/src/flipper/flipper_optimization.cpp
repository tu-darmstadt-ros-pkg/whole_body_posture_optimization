#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

#include <whole_body_posture_optimization/timing.h>

#include <geometry_msgs/PoseStamped.h>

namespace whole_body_posture_optimization {

FlipperOptimization::FlipperOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel> &robot_model,
                                         const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor)
  : nh_(nh), robot_model_(robot_model), pose_predictor_(pose_predictor), enabled_(true)
{
  if (!loadParameters(nh)) {
    ROS_ERROR_STREAM("Loading flipper parameters failed.");
  }
  if (!loadLimits()) {
    ROS_ERROR_STREAM("Loading flipper limits failed");
  }
  param_idx_to_jnt_idx_ = getIndexMap(robot_model_->getActuatedJointNames(), free_joints_);

  // Publisher
  start_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>("start_state", 10, true);
  result_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>("result_state", 10, true);
  contact_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("contact_points", 10, true);
  result_com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("result_com_pose", 10, true);

  // Create nlopt optimizer
  parameter_count_ = static_cast<unsigned int>(free_joints_.size());
  if (objective_ == "fast") {
    parameter_count_ += 3;
  }
  std::string algorithm;
  nh.param<std::string>("algorithm", algorithm, "GN_DIRECT_L");
  opt_ = nlopt::opt(algorithm.c_str(), parameter_count_);

  // Define parameter bounds
  opt_.set_lower_bounds(lower_bounds_);
  opt_.set_upper_bounds(upper_bounds_);

  // Set convergence criterion
  ros::NodeHandle stopping_criteria_nh(nh_, "stopping_criteria");
  ceres_nlopt_wrapper::setParametersFromServer(stopping_criteria_nh, opt_);

  // Set local optimizer
  std::string local_algorithm;
  if (nh.getParam("local_algorithm", local_algorithm)) {
    nlopt::opt local_opt(local_algorithm.c_str(), parameter_count_);
    ros::NodeHandle local_stopping_criteria_nh(nh_, "local_stopping_criteria");
    ceres_nlopt_wrapper::setParametersFromServer(local_stopping_criteria_nh, local_opt);

    local_opt.set_lower_bounds(lower_bounds_);
    local_opt.set_upper_bounds(upper_bounds_);
    opt_.set_local_optimizer(local_opt); // makes a copy
  }
}

bool FlipperOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("joints", free_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'joints' in namespace'" << nh.getNamespace() << "'.");
    return false;
  }

  nh.param<std::string>("objective", objective_, "heuristic");
  nh.param("enabled", enabled_, enabled_);

  if (objective_ == "heuristic") {
    flipper_multi_objective_cost_parameters_ = FlipperMultiObjectiveCostParameters::createFromServer(ros::NodeHandle(nh, "heuristic"));
    ROS_INFO_STREAM(flipper_multi_objective_cost_parameters_.toString());
  } else if (objective_ == "stability") {
//    stability_cost_parameters_ = FlipperStabilityCostParameters::createFromServer(ros::NodeHandle(nh, "stability"));
//    ROS_INFO_STREAM(stability_cost_parameters_.toString());
  } else if (objective_ == "fast") {
//    fast_traction_cost_parameters_ = FastTractionCost::Parameters::createFromServer(ros::NodeHandle(nh, "fast"));
  }

  return true;
}

FlipperOptimizationResult FlipperOptimization::optimize(const Eigen::Isometry3d &robot_pose, const std::vector<double>& init_joint_angles,
                                                        const robot_state::RobotStateConstPtr& previous_state)
{
  // Reset seed (for stochastic algorithms)
  nlopt_srand(0);

  // Add objective function

  // Set initial state in pose predictor
  const std::vector<double>* full_state;
  std::vector<double> previous_joint_angles;
  if (previous_state) {
    previous_joint_angles = getVariablePositions(previous_state, robot_model_->getActuatedJointNames());
    full_state = &previous_joint_angles;
  } else {
    full_state = &init_joint_angles;
  }
  std::unordered_map<std::string, double> initial_joint_update;
  initial_joint_update.reserve(robot_model_->getActuatedJointNames().size());
  for (size_t j = 0; j < init_joint_angles.size(); ++j) {
    initial_joint_update[robot_model_->getActuatedJointNames()[j]] = (*full_state)[j];
  }
  pose_predictor_->robotModel()->updateJointPositions(initial_joint_update);

  int verbosity_level = 0;
  std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> objective;
  if (objective_ == "heuristic") {
    objective = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(
        FlipperMultiObjectiveCost::Create(free_joints_, pose_predictor_, robot_pose, previous_state,
                                          flipper_multi_objective_cost_parameters_),
          verbosity_level);
  } else if (objective_ == "stability") {
//    objective = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(
//          FlipperStabilityCost::Create(static_cast<unsigned int>(free_joints_.size()), contact_estimation_, robot_pose, base_to_com, previous_parameters, stability_cost_parameters_),
//          verbosity_level);
  } else if (objective_ == "fast") {
//    objective = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(
//          FastTractionCost::Create(robot_pose, free_joints_, init_flipper_angles, previous_parameters, contact_estimation_, fast_traction_cost_parameters_),
//          verbosity_level);
  } else {
    ROS_ERROR_STREAM("Unknown objective '" << objective_ << "'!");
  }
  objective->setName("flipper_objective");
  objective->enableInfCheck(true);
  objective->enableNanCheck(true);
  opt_.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, objective.get());

  // Publish start state
//  robot_shape_gen_->updateShape(init_flipper_angles);
//  sdf_contact_estimation::publishShape(robot_shape_gen_->getShape(), start_state_pub_, robot_pose, "world", Eigen::Vector3d(1, 0, 0));

  std::vector<double> parameters = mapListToSubset(init_joint_angles, param_idx_to_jnt_idx_);
  ceres_nlopt_wrapper::applyBounds(lower_bounds_, upper_bounds_, parameters);
  if (objective_ == "fast") {
    Eigen::Vector3d pose_rpy = sdf_contact_estimation::rotToNormalizedRpy(robot_pose.linear());
    // z position and roll, pitch orientation parameters
    parameters.push_back(0);
    parameters.push_back(pose_rpy(0));
    parameters.push_back(pose_rpy(1));
  }
  START_TIMING("Flipper optimization")
  double minf = std::nan("");
  auto start = std::chrono::high_resolution_clock::now();
  nlopt::result nlopt_result;
  try {
    if (enabled_) {
      nlopt_result = opt_.optimize(parameters, minf);
      //  ROS_INFO_STREAM("Optimization returned " << ceres_nlopt_wrapper::resultToString(nlopt_result));
    }
  }
  catch (const std::invalid_argument& e) {
    ROS_ERROR_STREAM("Flipper optimization failed (invalid argument): " << e.what());
  }
  catch (const nlopt::roundoff_limited& e) {
    ROS_ERROR_STREAM("Flipper optimization failed (roundoff_limited): " << e.what());
  }
  catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Flipper optimization failed (runtime error): " << e.what());
  }
  STOP_TIMING_AVG

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Flipper optimization took " << objective->getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                  << ceres_nlopt_wrapper::vecToString(parameters) << ", Final cost: " << minf);

  // Estimating again for debug output
  bool cost_function_verbosity = flipper_multi_objective_cost_parameters_.verbose;
  flipper_multi_objective_cost_parameters_.verbose = true;
  if (flipper_multi_objective_cost_parameters_.verbose) {
    std::vector<double> gradient;
    ROS_INFO_STREAM("Final cost: " << (*objective)(parameters, gradient));
  }
  flipper_multi_objective_cost_parameters_.verbose = cost_function_verbosity;

  // Compose result
  FlipperOptimizationResult result;
  result.final_cost = minf;
  // Fill in complete robot state with solution
  result.joint_state = *full_state;
  for (unsigned int i = 0; i < param_idx_to_jnt_idx_.size(); i++) {
    result.joint_state[param_idx_to_jnt_idx_[i]] = parameters[i];
  }
  // Fill in complete flipper state with solution
//  result.flipper_angles = init_flipper_angles;
//  for (unsigned int i = 0; i < free_joints_.size(); ++i) {
//    result.flipper_angles[param_idx_to_flippers_idx_[i]] = parameters[i];
//  }

  // Estimate robot pose with solution
  std::unordered_map<std::string, double> final_joint_update;
  final_joint_update.reserve(free_joints_.size());
  for (size_t j = 0; j < free_joints_.size(); ++j) {
    final_joint_update[free_joints_[j]] = parameters[j];
  }
  pose_predictor_->robotModel()->updateJointPositions(final_joint_update);

  hector_math::Pose<double> result_pose(robot_pose);
  result.stability = pose_predictor_->predictPoseAndSupportPolygon(result_pose, result.support_polygon);
  result.pose = result_pose.asTransform();

  if (objective_ == "fast") {
    Eigen::Vector3d pose_rpy = sdf_contact_estimation::rotToNormalizedRpy(robot_pose.linear());
    result.pose = Eigen::Isometry3d(Eigen::AngleAxisd(pose_rpy(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(parameters[free_joints_.size() + 2], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(parameters[free_joints_.size() + 1], Eigen::Vector3d::UnitX()));
    result.pose.translation() = Eigen::Vector3d(robot_pose.translation()(0), robot_pose.translation()(1), robot_pose.translation()(2) + parameters[free_joints_.size() + 0]);
  }
  ROS_INFO("New robot position: [%f, %f, %f]", result.pose.translation()(0), result.pose.translation()(1), result.pose.translation()(2));

  // Publish result
  // Final robot shape
//  sdf_contact_estimation::publishShape(robot_shape_gen_->getShape(), result_state_pub_, result_pose, "world", Eigen::Vector3d(1, 1, 0));
  // COM pose
//  Eigen::Isometry3d result_com_pose = result.pose * base_to_com;
//  sdf_contact_estimation::publishPose(result_com_pub_, result_com_pose, "world");
  // Contact points and support polygon
//  visualization_msgs::MarkerArray contact_points_array_msg;
//  sdf_contact_estimation::createPointMarkerArray(result.contact_result.contact_points, contact_points_array_msg, "world", "contact_points", Eigen::Vector3d(1, 0.5, 0), 0.03);
//  sdf_contact_estimation::createPointMarkerArray(result.contact_result.convex_hull_points, contact_points_array_msg, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
//  sdf_contact_estimation::createNormalsMarkers(result.contact_result.contact_points, contact_estimation_->getSdf(), contact_points_array_msg, "contact_normals");
//  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(result.contact_result.convex_hull_points, result_com_pose.translation());
//  createConvexHullMarkerArray(result.contact_result.convex_hull_points, stabilities, contact_points_array_msg);
//  sdf_contact_estimation::deleteAllMarkers(contact_points_pub_);
//  contact_points_pub_.publish(contact_points_array_msg);

  return result;
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

  if (objective_ == "fast") {
    // Z offset
    upper_bounds_.push_back(0.5);
    lower_bounds_.push_back(-0.5);
    // Roll
    upper_bounds_.push_back(M_PI_4);
    lower_bounds_.push_back(-M_PI_4);
    // Pitch
    upper_bounds_.push_back(M_PI_2);
    lower_bounds_.push_back(-M_PI_2);
  }
  return true;
}

bool FlipperOptimization::enabled() const
{
  return enabled_;
}

void FlipperOptimization::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

}
