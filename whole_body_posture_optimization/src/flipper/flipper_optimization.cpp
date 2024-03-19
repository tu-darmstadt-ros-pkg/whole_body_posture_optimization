#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

#include <whole_body_posture_optimization/timing.h>
#include <whole_body_posture_optimization/solvers/optimizer_factory.h>

#include <geometry_msgs/PoseStamped.h>
#include <whole_body_planning_plotting_utils/data_log.h>
#include <hector_pose_prediction_ros/visualization.h>


namespace whole_body_posture_optimization {

using whole_body_planning_plotting_utils::DataLog;

using namespace hector_pose_prediction_interface;

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
  param_idx_to_jnt_idx_ = getIndexMap(robot_model_->getActiveJointNames(), free_joints_);

  // Publisher
  support_polygon_pub_ = nh.advertise<visualization_msgs::MarkerArray>("support_polygon", 10, true);

  // Create nlopt optimizer
  parameter_count_ = static_cast<unsigned int>(free_joints_.size());
  ros::NodeHandle solver_nh(nh, "solver");
  opt_server_ = std::make_shared<ceres_nlopt_wrapper::NloptParameterServer>(solver_nh, parameter_count_);
  solver_nh.param("enabled", enabled_, enabled_);

  // Create custom solver
  ros::NodeHandle custom_solver_nh(nh, "custom_solver");
  std::string solver_name;
  bool custom_solver_enabled = custom_solver_nh.param("enabled", false);
  if (custom_solver_enabled && custom_solver_nh.getParam("algorithm", solver_name)) {
    custom_solver_ = createSolverByName(solver_name);
    if (custom_solver_) {
      custom_solver_->loadParametersFromNamespace(custom_solver_nh);
    }
  }

  // Initialize logging
  if (DataLog::getInstance().getTable("flipper_optimization").enabled()) {
    std::vector<std::string> data_labels = {
      "pose_x",
      "pose_y",
      "pose_z",
      "optimization_time",
      "custom_solver_iterations",
      "custom_solver_cost",
      "iterations",
      "cost",
      "result_pose_x",
      "result_pose_y",
      "result_pose_z",
      "stability"
    };
    DataLog::getInstance().getTable("flipper_optimization").initialize(data_labels);
  }

  // Set local optimizer
  std::string local_algorithm;
  bool use_local_solver = false;
  if (nh.getParam("local_solver/enabled", use_local_solver) && use_local_solver) {
    ros::NodeHandle local_solver_nh(nh, "local_solver");
    local_opt_server_ = std::make_shared<ceres_nlopt_wrapper::NloptParameterServer>(local_solver_nh, parameter_count_);
  }
}

bool FlipperOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("joints", free_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'joints' in namespace'" << nh.getNamespace() << "'.");
    return false;
  }

  flipper_multi_objective_cost_parameters_.loadParametersFromNamespace(ros::NodeHandle(nh, "cost_parameters"));
  ROS_INFO_STREAM(flipper_multi_objective_cost_parameters_.toString());

  return true;
}

FlipperOptimizationResult FlipperOptimization::optimize(const Eigen::Isometry3d &robot_pose, const std::vector<double>& init_joint_angles,
                                                        const robot_state::RobotStateConstPtr& previous_state,
                                                        const robot_state::RobotStateConstPtr& last_solution)
{
  // Reset seed (for stochastic algorithms)
  nlopt_srand(0);

  // Construct solver
  nlopt::opt& opt = opt_server_->getOptimizer();
  if (local_opt_server_) {
    nlopt::opt& local_opt = local_opt_server_->getOptimizer();
    opt.set_local_optimizer(local_opt); // makes a copy
  }

  // Define parameter bounds
  opt.set_lower_bounds(lower_bounds_);
  opt.set_upper_bounds(upper_bounds_);

  // Set initial state in pose predictor
  const std::vector<double>* full_state;
  std::vector<double> previous_joint_angles;
  if (previous_state) {
    previous_joint_angles = getVariablePositions(previous_state, robot_model_->getActiveJointNames());
    full_state = &previous_joint_angles;
  } else {
    full_state = &init_joint_angles;
  }
  std::unordered_map<std::string, double> initial_joint_update;
  initial_joint_update.reserve(robot_model_->getActiveJointNames().size());
  for (size_t j = 0; j < full_state->size(); ++j) {
    initial_joint_update[robot_model_->getActiveJointNames()[j]] = (*full_state)[j];
  }
  pose_predictor_->robotModel()->updateJointPositions(initial_joint_update);

  // Add objective function
  int verbosity_level = 0;
  std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> objective;
  objective = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(
      FlipperMultiObjectiveCost::Create(free_joints_, pose_predictor_, robot_pose, previous_state,
                                        flipper_multi_objective_cost_parameters_, last_solution), verbosity_level);

  objective->setName("flipper_objective");
  objective->enableInfCheck(true);
  objective->enableNanCheck(true);
  opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, objective.get());

  std::vector<double> parameters = mapListToSubset(*full_state, param_idx_to_jnt_idx_);
  ceres_nlopt_wrapper::applyBounds(lower_bounds_, upper_bounds_, parameters);
  START_TIMING("Flipper optimization")
  auto start = std::chrono::high_resolution_clock::now();

  // Custom solver
  double custom_solver_minf = std::nan("");
  if (custom_solver_) {
    custom_solver_->loadSettingsFromNlopt(opt);
    custom_solver_->setMinFunction(objective);
    custom_solver_->optimize(parameters, custom_solver_minf);
  }
  unsigned int custom_solver_iterations = objective->getEvaluations();
  double minf = custom_solver_minf;

  // Nlopt solver
  nlopt::result nlopt_result;
  try {
    if (enabled_) {
      nlopt_result = opt.optimize(parameters, minf);
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
                  << ceres_nlopt_wrapper::vecToString(parameters) << ", Final cost: " << minf << ", " << ceres_nlopt_wrapper::resultToString(nlopt_result));

  // Compose result
  FlipperOptimizationResult result;
  result.final_cost = minf;
  // Fill in complete robot state with solution
  result.joint_state = *full_state;
  for (unsigned int i = 0; i < param_idx_to_jnt_idx_.size(); i++) {
    result.joint_state[param_idx_to_jnt_idx_[i]] = parameters[i];
  }

  // Estimate robot pose with solution
  std::unordered_map<std::string, double> final_joint_update;
  final_joint_update.reserve(free_joints_.size());
  for (size_t j = 0; j < free_joints_.size(); ++j) {
    final_joint_update[free_joints_[j]] = parameters[j];
  }
  pose_predictor_->robotModel()->updateJointPositions(final_joint_update);

  hector_math::Pose<double> result_pose(robot_pose);
  result.stability = pose_predictor_->predictPoseAndContactInformation(result_pose, result.support_polygon, result.contact_information, hector_pose_prediction_interface::ContactInformationFlags::All);
  result.pose = result_pose.asTransform();

  ROS_INFO("New robot position: [%f, %f, %f]", result.pose.translation()(0), result.pose.translation()(1), result.pose.translation()(2));

  // Logging
  if (DataLog::getInstance().getTable("flipper_optimization").enabled()) {
    Eigen::VectorXd data(DataLog::getInstance().getTable("flipper_optimization").getDataSize());
    data << robot_pose.translation().x(),
        robot_pose.translation().y(),
        robot_pose.translation().z(),
        elapsed.count(),
        custom_solver_iterations,
        custom_solver_minf,
        objective->getEvaluations(),
        minf,
        result.pose.translation().x(),
        result.pose.translation().y(),
        result.pose.translation().z(),
        result.stability;

    DataLog::getInstance().getTable("flipper_optimization").addData(data);
  }

  // Write out probability distribution
  if (DataLog::getInstance().getTable("_" + PROBABILITY_DISTRIBUTION_TABLE_NAME).enabled()) {
    const double range = 0.2;
    const double resolution = 0.001;
    const int num_samples = std::floor(range / resolution);
    ROS_INFO_STREAM("Sampling PDF. Range: " << range << ", resolution: " << resolution << ", num. samples: " << num_samples);

    double orig_samples = flipper_multi_objective_cost_parameters_.num_pose_x_samples;
    double orig_resolution = flipper_multi_objective_cost_parameters_.sample_step_pose_x;

    flipper_multi_objective_cost_parameters_.num_pose_x_samples = num_samples;
    flipper_multi_objective_cost_parameters_.sample_step_pose_x = resolution;

    DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).enableLogging();
    DataLog::getInstance().getTable("flipper_multi_objective_cost").enableLogging(false);

    std::vector<double> gradient;
    double cvar = (*objective)(parameters, gradient);
    ROS_INFO_STREAM("CVaR: " << cvar);

    DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).enableLogging(false);
    DataLog::getInstance().getTable("flipper_multi_objective_cost").enableLogging();

    flipper_multi_objective_cost_parameters_.num_pose_x_samples = orig_samples;
    flipper_multi_objective_cost_parameters_.sample_step_pose_x = orig_resolution;
  }


  // Estimating again for debug output
  bool cost_function_verbosity = flipper_multi_objective_cost_parameters_.verbose;
  flipper_multi_objective_cost_parameters_.verbose = true;
  if (flipper_multi_objective_cost_parameters_.verbose) {
    std::vector<double> gradient;
    (*objective)(parameters, gradient);
  }
  flipper_multi_objective_cost_parameters_.verbose = cost_function_verbosity;

  // Publish support polygon and contact information
  visualization_msgs::MarkerArray support_polygon_marker_array;
  deleteAllMarkers(support_polygon_pub_);
  visualization::addSupportPolygonWithContactInformationToMarkerArray(support_polygon_marker_array, result.support_polygon, result.contact_information, "world");
  support_polygon_pub_.publish(support_polygon_marker_array);

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
