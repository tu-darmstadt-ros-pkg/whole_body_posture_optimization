#include <whole_body_posture_optimization/arm/arm_optimization.h>

#include "whole_body_posture_optimization/util/timing.h"
#include "whole_body_posture_optimization/util/common.h"
#include "whole_body_posture_optimization/util/parameter_utils.h"
#include <whole_body_posture_optimization/arm/sdf_contact_constraint.h>
#include <whole_body_posture_optimization/arm/self_contact_constraint.h>

#include <whole_body_planning_plotting_utils/data_log.h>
#include <sdf_contact_estimation/visualisation.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <chrono>
#include <ceres_nlopt_wrapper/optimization_history.h>
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>

namespace whole_body_posture_optimization {

using whole_body_planning_plotting_utils::DataLog;

ArmOptimization::ArmOptimization(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<RobotModel>& robot_model,
                                 const sdf_contact_estimation::SdfModelPtr& sdf_model)
  : node_(node), robot_model_(robot_model), sdf_model_(sdf_model), enabled_(true), algorithm_("")
{
  param_prefix_ = param_utils::paramPrefix(*node_);
  if (!loadParameters(*node_)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Loading arm parameters failed.");
  }
  if (!loadLimits()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Loading arm limits failed");
  }
  param_idx_to_jnt_idx_ = getIndexMap(robot_model->getActiveJointNames(), free_joints_);

  // Load Allowed Collision Matrix (ACM)
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());
  acm_ = planning_scene->getAllowedCollisionMatrix();
  loadAdditionalACMEntries(*node_);

  // Precompute partial robot tsdf
  partial_robot_tsdf_ = robot_model_->computeTsdfFromRobotModel(static_link_names_, static_cast<float>(self_collision_tsdf_truncation_distance_), static_cast<float>(self_collision_tsdf_voxel_size_));

  collision_link_names_ = robot_model_->getCollisionGroupNames();
  for (const auto& link_name: collision_link_names_) {
    auto it = std::find(self_collision_tsdf_allowed_links_.begin(), self_collision_tsdf_allowed_links_.end(), link_name);
    if (it == self_collision_tsdf_allowed_links_.end()) {
      self_collision_tsdf_link_names_.push_back(link_name);
    }
  }

  // Initialize logging
  if (DataLog::getInstance().getTable("arm_optimization").enabled()) {
    std::vector<std::string> data_labels = {
      "pose_x",
      "pose_y",
      "pose_z",
      "optimization_time",
      "iterations",
      "cost",
      "stability"
    };
    DataLog::getInstance().getTable("arm_optimization").initialize(data_labels);
  }

  //Publisher
  robot_tsdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("robot_tsdf", rclcpp::QoS(10).transient_local());

  // Create nlopt optimizer
  opt_server_ = std::make_shared<ceres_nlopt_wrapper::NloptParameterServer>(node_.get(), param_prefix_ + "solver", free_joints_.size());

  // Set local optimizer
  bool use_local_solver = param_utils::param(*node_, param_prefix_ + "local_solver.enabled", false);
  if (use_local_solver) {
    local_opt_server_ = std::make_shared<ceres_nlopt_wrapper::NloptParameterServer>(node_.get(), param_prefix_ + "local_solver", free_joints_.size());
  }
}

bool ArmOptimization::loadParameters(rclcpp::Node& node)
{
  bool success = true;
  if (!param_utils::getParam(node, param_prefix_ + "joints", free_joints_)) {
    RCLCPP_ERROR_STREAM(node.get_logger(), "Failed to find parameter '" << param_prefix_ << "joints'.");
    success = false;
  }

  environment_collision_default_padding_ = param_utils::param(node, param_prefix_ + "environment_collision_default_padding", 0.0);
  environment_collision_padding_map_ = param_utils::getDoubleMap(node, param_prefix_ + "environment_collision_padding", environment_collision_padding_map_);

  static_link_names_ = param_utils::param(node, param_prefix_ + "static_link_names", static_link_names_);
  semi_static_link_names_ = param_utils::param(node, param_prefix_ + "semi_static_link_names", semi_static_link_names_);
  self_collision_default_padding_ = param_utils::param(node, param_prefix_ + "self_collision_default_padding", 0.0);
  self_collision_padding_map_ = param_utils::getDoubleMap(node, param_prefix_ + "self_collision_padding", self_collision_padding_map_);
  self_collision_tsdf_default_padding_ = param_utils::param(node, param_prefix_ + "self_collision_tsdf_default_padding", 0.0);
  self_collision_tsdf_padding_map_ = param_utils::getDoubleMap(node, param_prefix_ + "self_collision_tsdf_padding", self_collision_tsdf_padding_map_);
  self_collision_tsdf_truncation_distance_ = param_utils::param(node, param_prefix_ + "self_collision_tsdf_truncation_distance", 0.4);
  self_collision_tsdf_voxel_size_ = param_utils::param(node, param_prefix_ + "self_collision_tsdf_voxel_size", 0.05);
  self_collision_tsdf_allowed_links_ = param_utils::param(node, param_prefix_ + "self_collision_tsdf_allowed_links", std::vector<std::string>());
  enabled_ = param_utils::param(node, param_prefix_ + "enabled", enabled_);
  algorithm_ = param_utils::param(node, param_prefix_ + "algorithm", std::string("LD_SLSQP"));

  stability_cost_params_.loadParametersFromNamespace(node_->create_sub_node("cost_parameters"));
  RCLCPP_INFO_STREAM(node.get_logger(), stability_cost_params_.toString());

  return success;
}

double ArmOptimization::getJointPadding(std::string joint_name, CollisionType collision_type)
{
  std::map<std::string, double>* padding_map;
  switch(collision_type) {
    case CollisionType::ENVIRONMENT:
      padding_map = &environment_collision_padding_map_;
      break;
    case CollisionType::SELF_COLLISION:
      padding_map = &self_collision_padding_map_;
      break;
    default:
      padding_map = &self_collision_tsdf_padding_map_;
  }
  bool found;
  double padding = 0.0;
  if (padding_map) {
    try {
      padding = padding_map->at(joint_name);
      found = true;
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

double ArmOptimization::getDefaultJointPadding(CollisionType collision_type)
{
  switch (collision_type) {
    case ENVIRONMENT: return environment_collision_default_padding_;
    case SELF_COLLISION: return self_collision_default_padding_;
    case SELF_COLLISION_TSDF: return self_collision_tsdf_default_padding_;
    default: RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown collision type. Defaulting to zero padding.");
  }
  return 0.0;
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

bool ArmOptimization::getEnabled() const
{
  return enabled_;
}

void ArmOptimization::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

ArmOptimizationResult ArmOptimization::optimize(const std::vector<double>& initial_angles, const std::vector<Eigen::Vector3d>& contact_hull_points, const Eigen::Isometry3d& robot_pose,
                                                const std::vector<double>& previous_joint_angles)
{
  ArmOptimizationResult result;

  // Set initial state in pose predictor
  const std::vector<double>* full_state;
  if (!previous_joint_angles.empty()) {
    full_state = &previous_joint_angles;
  } else {
    full_state = &initial_angles;
  }

  result.joint_angles = *full_state;

  // immediately return if there are no contact points with the ground (error)
  if (contact_hull_points.empty()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "No contact points with the ground, aborting arm optimization.");
    result.stability = -1;
    result.center_of_mass = Eigen::Vector3d::Zero();
    return result;
  }

  if (enabled_) {
    nlopt::opt& opt = opt_server_->getOptimizer();
    if (local_opt_server_) {
      nlopt::opt& local_opt = local_opt_server_->getOptimizer();
      opt.set_local_optimizer(local_opt); // makes a copy
    }

    // Define parameter bounds
    opt.set_lower_bounds(lower_bounds_);
    opt.set_upper_bounds(upper_bounds_);

    // Add objective function
    ceres_nlopt_wrapper::OptimizationHistory history;

    std::vector<double> previous_parameters;
    if (!previous_joint_angles.empty()) {
      previous_parameters = mapListToSubset(previous_joint_angles, param_idx_to_jnt_idx_);
    }
    ceres_nlopt_wrapper::CeresCostFunctionWrapper stability_cost(
        StabilityCost::Create(robot_model_, free_joints_, contact_hull_points, *full_state, previous_parameters, robot_pose, stability_cost_params_), 0, history.setObjective());
    stability_cost.setName("stability_cost");
    stability_cost.enableInfCheck(true);
    stability_cost.enableNanCheck(true);
    opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &stability_cost);

    // Add constraints
    opt.remove_inequality_constraints();
    std::vector<std::size_t> param_idx_to_joint_idx = robot_model_->getActiveJointIndices(free_joints_);
    std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> environment_constraints;
    START_TIMING("Env. collision const. creation")
    // Environment collision constraints
    for (const std::string& collision_link: collision_link_names_) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Creating environment collision constraints for link: " << collision_link);
      for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
        CollisionObject collision_object(sphere, collision_link, getJointPadding(collision_link, ENVIRONMENT));
        ceres::CostFunction* cost_function = SdfContactConstraint::Create(
              robot_model_, free_joints_, param_idx_to_joint_idx, *full_state, robot_pose, sdf_model_, collision_object);
        std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function, 0, history.addConstraint());
        wrapper->setName("constraint_environment_" + collision_link);
        environment_constraints.push_back(wrapper);
        opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
      }
    }
    STOP_TIMING_AVG

    // self-collision tsdf constraints
    START_TIMING("Self-collision tsdf const. creation")
    std::shared_ptr<voxblox::TsdfMap> tsdf = robot_model_->computeTsdfFromRobotModel(semi_static_link_names_, static_cast<float>(self_collision_tsdf_truncation_distance_), partial_robot_tsdf_, *full_state);
    sdf_contact_estimation::publishTsdfSlice(robot_tsdf_pub_, tsdf, robot_pose, sdf_model_->getWorldFrame(), 0.4f);
    auto robot_base_tsdf = std::make_shared<sdf_contact_estimation::SdfModel>();
    robot_base_tsdf->loadTsdf(tsdf, self_collision_tsdf_truncation_distance_, false, false);

    std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> self_collision_tsdf_constraints;
    for (const std::string& collision_link: self_collision_tsdf_link_names_) {
      for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
        CollisionObject collision_object(sphere, collision_link, getJointPadding(collision_link, SELF_COLLISION_TSDF));
        ceres::CostFunction* cost_function = SdfContactConstraint::Create(
              robot_model_, free_joints_, param_idx_to_joint_idx, *full_state, Eigen::Isometry3d::Identity(), robot_base_tsdf, collision_object);
        std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function, 0, history.addConstraint());
        wrapper->setName("constraint_self_collision_tsdf_" + collision_link);
        self_collision_tsdf_constraints.push_back(wrapper);
        opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
      }
    }
    STOP_TIMING_AVG

    // self-collision distance constraints
    START_TIMING("Self-collision const. creation")
    std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> self_collision_constraints;
    // Create constraint for each unique link-pair
    for (unsigned int i = 0; i < collision_link_names_.size()-1; i++) {
      std::string link_name_1 = collision_link_names_[i];
      for (unsigned int j = i+1; j < collision_link_names_.size(); j++) {
        std::string link_name_2 = collision_link_names_[j];

        // Check acm
        collision_detection::AllowedCollision::Type allowed_collision_type;
        if (acm_.getAllowedCollision(link_name_1, link_name_2, allowed_collision_type)) {
          if (allowed_collision_type == collision_detection::AllowedCollision::ALWAYS) {
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Skipping collision between '" << link_name_1 << "' and '" << link_name_2 << "'.");
            continue;
          }
        }

        // Constraint for each sphere-pair
        for (const collision_detection::CollisionSphere& sphere1: robot_model_->getCollisionSpheres(link_name_1)) {
          for (const collision_detection::CollisionSphere& sphere2: robot_model_->getCollisionSpheres(link_name_2)) {
            CollisionObject obj1(sphere1, link_name_1, getJointPadding(link_name_1, SELF_COLLISION));
            CollisionObject obj2(sphere2, link_name_2, getJointPadding(link_name_2, SELF_COLLISION));
            ceres::CostFunction* cost_function = SelfContactConstraint::Create(
                  robot_model_, free_joints_, param_idx_to_joint_idx, *full_state, obj1, obj2);
            std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function, 0, history.addConstraint());
            wrapper->setName("constraint_self_collision_" + link_name_1 + "_vs_" + link_name_2);
            self_collision_constraints.push_back(wrapper);
            opt.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
          }
        }
      }
    }
    STOP_TIMING_AVG

    // Fill in initial joint state
    std::vector<double> parameters = jointVectorToParamVector(initial_angles);
    ceres_nlopt_wrapper::applyBounds(lower_bounds_, upper_bounds_, parameters);

    // Check for collisions in initialization
    std::vector<ceres_nlopt_wrapper::ConstraintViolation> constraint_violations;
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, environment_constraints, 0.001, constraint_violations)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Detected environment collision before optimization!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        RCLCPP_WARN_STREAM(node_->get_logger(), violation.toString());
      }
    }
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_tsdf_constraints, 0.001, constraint_violations)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Detected self-collision with tsdf before optimization!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        RCLCPP_WARN_STREAM(node_->get_logger(), violation.toString());
      }
    }
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_constraints, 0.001, constraint_violations)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Detected self-collision before optimization!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        RCLCPP_WARN_STREAM(node_->get_logger(), violation.toString());
      }
    }

    START_TIMING("Arm optimization")
    double minf = std::nan("");
    auto start = std::chrono::high_resolution_clock::now();
    nlopt::result nlopt_result = nlopt::FAILURE;
    try {
      nlopt_result = opt.optimize(parameters, minf);
    }
    catch (const std::invalid_argument& e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Optimization failed with invalid argument error: " << e.what());
    }
    catch (const nlopt::roundoff_limited& e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Optimization failed: " << e.what());
    }
    catch (const std::runtime_error& e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Optimization failed: " << e.what());
    }

    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Arm optimization took " << stability_cost.getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                    << ceres_nlopt_wrapper::vecToString(parameters) << ", Final cost: " << minf << ", " << ceres_nlopt_wrapper::resultToString(nlopt_result));
    result.iterations = stability_cost.getEvaluations();
    result.optimization_time = elapsed.count();

    const ceres_nlopt_wrapper::OptimizationResult* best_result = history.getBestResult();
    if (best_result) {
      parameters = best_result->parameters;
    } else {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Arm optimization history does not contain any valid solution.");
    }


    RCLCPP_INFO_STREAM(node_->get_logger(), "Arm optimization final solution: " << ceres_nlopt_wrapper::vecToString(parameters) << ", cost: " << minf);
    STOP_TIMING_AVG

    // Check for collisions in solution
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, environment_constraints, 0.001, constraint_violations)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Detected environment collision!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        RCLCPP_WARN_STREAM(node_->get_logger(), violation.toString());
      }
    }
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_tsdf_constraints, 0.001, constraint_violations)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Detected self-collision with tsdf!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        RCLCPP_WARN_STREAM(node_->get_logger(), violation.toString());
      }
    }
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_constraints, 0.001, constraint_violations)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Detected self-collision!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        RCLCPP_WARN_STREAM(node_->get_logger(), violation.toString());
      }
    }
    // Compose result
    for (unsigned int i = 0; i < parameters.size(); i++) {
      result.joint_angles[param_idx_to_jnt_idx_[i]] = parameters[i];
    }
    result.final_cost = minf;
  }

  // Compute COM and stability
  result.center_of_mass = robot_model_->computeCenterOfMass(result.joint_angles);
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(contact_hull_points, robot_pose * result.center_of_mass);
  auto min_stability_it = std::min_element(stabilities.begin(), stabilities.end());
  if (min_stability_it != stabilities.end()) {
    result.stability = *min_stability_it;
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Stability is empty.");
    result.stability = -std::numeric_limits<double>::max();
  }

  // Logging
  if (DataLog::getInstance().getTable("arm_optimization").enabled()) {
    Eigen::VectorXd data(DataLog::getInstance().getTable("arm_optimization").getDataSize());
    data << robot_pose.translation().x(),
        robot_pose.translation().y(),
        robot_pose.translation().z(),
        result.optimization_time,
        result.iterations,
        result.final_cost,
        result.stability;

    DataLog::getInstance().getTable("arm_optimization").addData(data);
  }

  return result;
}
void ArmOptimization::loadAdditionalACMEntries(rclcpp::Node& node)
{
  // ROS 2 parameters do not support lists of dicts. The allowed link pairs are
  // configured as a map from link name to a list of links instead:
  //   self_collision_allowed_links:
  //     arm_link_1: [arm_link_3]
  const auto allowed_links = param_utils::getStringListMap(node, param_prefix_ + "self_collision_allowed_links");
  for (const auto& [link1, links]: allowed_links) {
    for (const std::string& link2: links) {
      acm_.setEntry(link1, link2, true);
    }
  }
}

}
