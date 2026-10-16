#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

#include "whole_body_posture_optimization/util/timing.h"
#include "whole_body_posture_optimization/util/parameter_utils.h"
#include <whole_body_posture_optimization/solvers/optimizer_factory.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <whole_body_planning_plotting_utils/data_log.h>


namespace whole_body_posture_optimization {

using whole_body_planning_plotting_utils::DataLog;

using namespace hector_pose_prediction_interface;

FlipperOptimization::FlipperOptimization(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<RobotModel> &robot_model,
                                         const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor)
  : node_(node), robot_model_(robot_model), pose_predictor_(pose_predictor), enabled_(true)
{
  param_prefix_ = param_utils::paramPrefix(*node_);
  if (!loadParameters(*node_)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Loading flipper parameters failed.");
  }
  if (!loadLimits()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Loading flipper limits failed");
  }
  param_idx_to_jnt_idx_ = getIndexMap(robot_model_->getActiveJointNames(), free_joints_);
  for (const std::string& joint : coupled_joints_) {
    coupled_idx_to_jnt_idx_.push_back(joint.empty() ? robot_model_->getActiveJointNames().size()
                                                    : getIndexMap(robot_model_->getActiveJointNames(), std::vector<std::string>{joint}).front());
  }

  // Create nlopt optimizer
  parameter_count_ = static_cast<unsigned int>(free_joints_.size());
  opt_server_ = std::make_shared<ceres_nlopt_wrapper::NloptParameterServer>(node_.get(), param_prefix_ + "solver", parameter_count_);
  enabled_ = param_utils::param(*node_, param_prefix_ + "solver.enabled", enabled_);

  // Create custom solver
  bool custom_solver_enabled = param_utils::param(*node_, param_prefix_ + "custom_solver.enabled", false);
  std::string solver_name;
  if (custom_solver_enabled && param_utils::getParam(*node_, param_prefix_ + "custom_solver.algorithm", solver_name)) {
    custom_solver_ = createSolverByName(solver_name);
    if (custom_solver_) {
      custom_solver_->loadParametersFromNamespace(*node_, param_prefix_ + "custom_solver.");
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
  bool use_local_solver = param_utils::param(*node_, param_prefix_ + "local_solver.enabled", false);
  if (use_local_solver) {
    local_opt_server_ = std::make_shared<ceres_nlopt_wrapper::NloptParameterServer>(node_.get(), param_prefix_ + "local_solver", parameter_count_);
  }
}

bool FlipperOptimization::loadParameters(rclcpp::Node& node)
{
  if (!param_utils::getParam(node, param_prefix_ + "joints", free_joints_)) {
    RCLCPP_ERROR_STREAM(node.get_logger(), "Failed to find parameter '" << param_prefix_ << "joints'.");
    return false;
  }
  param_utils::getParam(node, param_prefix_ + "coupled_joints", coupled_joints_);
  if (!coupled_joints_.empty() && coupled_joints_.size() != free_joints_.size()) {
    throw std::invalid_argument(param_prefix_ + "coupled_joints must have one entry (or \"\") per joint.");
  }

  flipper_multi_objective_cost_parameters_.loadParametersFromNamespace(node_->create_sub_node("cost_parameters"));
  RCLCPP_INFO_STREAM(node.get_logger(), flipper_multi_objective_cost_parameters_.toString());

  return true;
}

FlipperOptimizationResult FlipperOptimization::optimize(const Eigen::Isometry3d &robot_pose, const std::vector<double>& init_joint_angles,
                                                        const moveit::core::RobotStateConstPtr& previous_state,
                                                        const moveit::core::RobotStateConstPtr& last_solution)
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
                                        flipper_multi_objective_cost_parameters_, last_solution, coupled_joints_), verbosity_level);

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
  nlopt::result nlopt_result = nlopt::FAILURE;
  try {
    if (enabled_) {
      nlopt_result = opt.optimize(parameters, minf);
    }
  }
  catch (const std::invalid_argument& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Flipper optimization failed (invalid argument): " << e.what());
  }
  catch (const nlopt::roundoff_limited& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Flipper optimization failed (roundoff_limited): " << e.what());
  }
  catch (const std::runtime_error& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Flipper optimization failed (runtime error): " << e.what());
  }
  STOP_TIMING_AVG

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Flipper optimization took " << objective->getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                  << ceres_nlopt_wrapper::vecToString(parameters) << ", Final cost: " << minf << ", " << ceres_nlopt_wrapper::resultToString(nlopt_result));

  // Compose result
  FlipperOptimizationResult result;
  result.final_cost = minf;
  // Fill in complete robot state with solution
  result.joint_state = *full_state;
  for (unsigned int i = 0; i < param_idx_to_jnt_idx_.size(); i++) {
    result.joint_state[param_idx_to_jnt_idx_[i]] = parameters[i];
  }
  for (unsigned int i = 0; i < coupled_idx_to_jnt_idx_.size(); i++) {
    if (coupled_idx_to_jnt_idx_[i] < result.joint_state.size()) {
      result.joint_state[coupled_idx_to_jnt_idx_[i]] = parameters[i];
    }
  }

  // Estimate robot pose with solution
  std::unordered_map<std::string, double> final_joint_update;
  final_joint_update.reserve(free_joints_.size());
  for (size_t j = 0; j < free_joints_.size(); ++j) {
    final_joint_update[free_joints_[j]] = parameters[j];
  }
  for (size_t j = 0; j < coupled_joints_.size(); ++j) {
    if (!coupled_joints_[j].empty()) {
      final_joint_update[coupled_joints_[j]] = parameters[j];
    }
  }
  pose_predictor_->robotModel()->updateJointPositions(final_joint_update);

  hector_math::Pose<double> result_pose(robot_pose);
  result.stability = pose_predictor_->predictPoseAndContactInformation(result_pose, result.support_polygon, result.contact_information, hector_pose_prediction_interface::ContactInformationFlags::All);
  result.pose = result_pose.asTransform();

  RCLCPP_INFO(node_->get_logger(), "New robot position: [%f, %f, %f]", result.pose.translation()(0), result.pose.translation()(1), result.pose.translation()(2));

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
    RCLCPP_INFO_STREAM(node_->get_logger(), "Sampling PDF. Range: " << range << ", resolution: " << resolution << ", num. samples: " << num_samples);

    double orig_samples = flipper_multi_objective_cost_parameters_.num_pose_x_samples;
    double orig_resolution = flipper_multi_objective_cost_parameters_.sample_step_pose_x;

    flipper_multi_objective_cost_parameters_.num_pose_x_samples = num_samples;
    flipper_multi_objective_cost_parameters_.sample_step_pose_x = resolution;

    DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).enableLogging();
    DataLog::getInstance().getTable("flipper_multi_objective_cost").enableLogging(false);

    std::vector<double> gradient;
    double cvar = (*objective)(parameters, gradient);
    RCLCPP_INFO_STREAM(node_->get_logger(), "CVaR: " << cvar);

    DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).enableLogging(false);
    DataLog::getInstance().getTable("flipper_multi_objective_cost").enableLogging();

    flipper_multi_objective_cost_parameters_.num_pose_x_samples = orig_samples;
    flipper_multi_objective_cost_parameters_.sample_step_pose_x = orig_resolution;
  }

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
