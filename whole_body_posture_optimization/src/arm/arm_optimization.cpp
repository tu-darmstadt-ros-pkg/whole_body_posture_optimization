#include <whole_body_posture_optimization/arm/arm_optimization.h>

#include <whole_body_posture_optimization/timing.h>
#include <sdf_contact_estimation/visualisation.h>
#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/arm/sdf_contact_constraint.h>
#include <whole_body_posture_optimization/arm/self_contact_constraint.h>

#include <moveit/planning_scene/planning_scene.h>
#include <chrono>
#include <ceres_nlopt_wrapper/optimization_history.h>
#include <eigen_conversions/eigen_msg.h>
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>

namespace whole_body_posture_optimization {

ArmOptimization::ArmOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model,
                                 const sdf_contact_estimation::SdfModelPtr& sdf_model)
  : nh_(nh), robot_model_(robot_model), sdf_model_(sdf_model), enabled_(true), algorithm_("")
{
  if (!loadParameters(nh)) {
    ROS_ERROR_STREAM("Loading arm parameters failed.");
  }
  if (!loadLimits()) {
    ROS_ERROR_STREAM("Loading arm limits failed");
  }
  param_idx_to_jnt_idx_ = getIndexMap(robot_model->getActuatedJointNames(), free_joints_);

  // Load Allowed Collision Matrix (ACM)
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());
  acm_ = planning_scene->getAllowedCollisionMatrix();

  // Precompute partial robot tsdf
  partial_robot_tsdf_ = robot_model_->computeTsdfFromRobotModel(static_link_names_, self_collision_tsdf_truncation_distance_, self_collision_tsdf_voxel_size_);

  //Publisher
  result_com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("result_com", 10, true);
  convex_hull_pub_ = nh.advertise<visualization_msgs::MarkerArray>("convex_hull", 10, true);
  robot_tsdf_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("robot_tsdf", 10, true);

  // Create nlopt optimizer
  opt_ = nlopt::opt(algorithm_.c_str(), free_joints_.size());
  // Define parameter bounds
  opt_.set_lower_bounds(lower_bounds_);
  opt_.set_upper_bounds(upper_bounds_);

  // Set convergence criterion
  ros::NodeHandle stopping_criteria_nh(nh_, "stopping_criteria");
  ceres_nlopt_wrapper::setParametersFromServer(stopping_criteria_nh, opt_);

  // Set local optimizer
  std::string local_algorithm;
  if (nh.getParam("local_algorithm", local_algorithm)) {
    nlopt::opt local_opt(local_algorithm.c_str(), free_joints_.size());
    ros::NodeHandle local_stopping_criteria_nh(nh_, "local_stopping_criteria");
    ceres_nlopt_wrapper::setParametersFromServer(local_stopping_criteria_nh, local_opt);

    local_opt.set_lower_bounds(lower_bounds_);
    local_opt.set_upper_bounds(upper_bounds_);
    opt_.set_local_optimizer(local_opt); // makes a copy
  }

  std::string sub_algorithm;
  if (nh.getParam("sub_algorithm", sub_algorithm)) {
    sub_opt_ = std::make_shared<nlopt::opt>(sub_algorithm.c_str(), free_joints_.size());
    // Define parameter bounds
    sub_opt_->set_lower_bounds(lower_bounds_);
    sub_opt_->set_upper_bounds(upper_bounds_);
    ros::NodeHandle sub_stopping_criteria_nh(nh_, "sub_stopping_criteria");
    ceres_nlopt_wrapper::setParametersFromServer(sub_stopping_criteria_nh, *sub_opt_);
  }
}

bool ArmOptimization::loadParameters(const ros::NodeHandle &nh)
{
  bool success = true;
  if(!nh.getParam("joints", free_joints_)) {
    ROS_ERROR_STREAM("Failed to find parameter 'joints' in namespace'" << nh.getNamespace() << "'.");
    success = false;
  }

  nh.param("environment_collision_default_padding", environment_collision_default_padding_, 0.0);
  nh.param<std::map<std::string, double>>("environment_collision_padding", environment_collision_padding_map_, environment_collision_padding_map_);

  nh.param<std::vector<std::string>>("static_link_names", static_link_names_, static_link_names_);
  nh.param<std::vector<std::string>>("semi_static_link_names", semi_static_link_names_, semi_static_link_names_);
  nh.param("self_collision_default_padding", self_collision_default_padding_, 0.0);
  nh.param<std::map<std::string, double>>("self_collision_padding", self_collision_padding_map_, self_collision_padding_map_);
  nh.param("self_collision_tsdf_default_padding", self_collision_tsdf_default_padding_, 0.0);
  nh.param<std::map<std::string, double>>("self_collision_tsdf_padding", self_collision_tsdf_padding_map_, self_collision_tsdf_padding_map_);
  nh.param("self_collision_tsdf_truncation_distance", self_collision_tsdf_truncation_distance_, 0.4);
  nh.param("self_collision_tsdf_voxel_size", self_collision_tsdf_voxel_size_, 0.05);
  nh.param("enabled", enabled_, enabled_);
  nh.param<std::string>("algorithm", algorithm_, "LD_SLSQP");

  stability_error_params_ = StabilityErrorParameters::createFromServer(ros::NodeHandle(nh, "objective_parameters"));
  ROS_INFO_STREAM(stability_error_params_.toString());

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
//      ROS_DEBUG_STREAM("Using padding " << padding << " for link '" << joint_name << "'.");
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
    default: ROS_WARN_STREAM("Unknown collision type. Defaulting to zero padding.");
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
  result.joint_angles = initial_angles;

  // immediately return if there are no contact points with the ground (error)
  if (contact_hull_points.empty()) {
    ROS_ERROR_STREAM("No contact points with the ground, aborting arm optimization.");
    result.stability = -1;
    result.center_of_mass = Eigen::Vector3d::Zero();
    return result;
  }

  if (enabled_) {
    // Add objective function
    ceres_nlopt_wrapper::OptimizationHistory history;
    std::vector<double> previous_parameters;
    if (!previous_joint_angles.empty()) {
      previous_parameters = mapListToSubset(previous_joint_angles, param_idx_to_jnt_idx_);
    }
    ceres_nlopt_wrapper::CeresCostFunctionWrapper stability_cost(
          StabilityError::Create(robot_model_, free_joints_, contact_hull_points, initial_angles, previous_parameters, robot_pose, stability_error_params_), 0, history.setObjective());
    stability_cost.setName("stability_cost");
    stability_cost.enableInfCheck(true);
    stability_cost.enableNanCheck(true);
    opt_.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &stability_cost);

    // Add constraints
    opt_.remove_inequality_constraints();
    std::vector<int> param_idx_to_chain_idx = robot_model_->getParamToChainIndexMapping(free_joints_);
    std::vector<std::size_t> chain_idx_to_jnt_idx = robot_model_->getChainToJointIndexMapping();
    std::vector<double> default_chain_angles(chain_idx_to_jnt_idx.size());
    for (unsigned int chain_idx = 0; chain_idx < default_chain_angles.size(); chain_idx++) {
      default_chain_angles[chain_idx] = initial_angles[chain_idx_to_jnt_idx[chain_idx]];
    }
    std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> environment_constraints;
    START_TIMING("Env. collision const. creation")
    // Environment collision constraints
    std::vector<std::string> collision_group_names = robot_model_->getCollisionGroupNames();
    for (const std::string& collision_link: collision_group_names) {
      ROS_DEBUG_STREAM("Creating environment collision constraints for link: " << collision_link);
      for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
        CollisionObject collision_object(sphere, collision_link, getJointPadding(collision_link, ENVIRONMENT));
        ceres::CostFunction* cost_function = SdfContactConstraint::Create(
              robot_model_, free_joints_, param_idx_to_chain_idx, default_chain_angles, robot_pose, sdf_model_, collision_object);
        std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function, 0, history.addConstraint());
        wrapper->setName("constraint_environment_" + collision_link);
        environment_constraints.push_back(wrapper);
        opt_.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
      }
    }
  //  ROS_INFO_STREAM("Created " << environment_constraints.size() << " environment collision constraints.");
    STOP_TIMING_AVG

    // self-collision tsdf constraints
    START_TIMING("Self-collision tsdf const. creation")
    std::shared_ptr<voxblox::TsdfMap> tsdf = robot_model_->computeTsdfFromRobotModel(semi_static_link_names_, static_cast<float>(self_collision_tsdf_truncation_distance_), partial_robot_tsdf_, initial_angles);
//    sdf_contact_estimation::publishTsdfSlice(robot_tsdf_pub_, tsdf, robot_pose, 0.4f);
    auto robot_base_tsdf = std::make_shared<sdf_contact_estimation::SdfModel>();
    robot_base_tsdf->loadTsdf(tsdf, self_collision_tsdf_truncation_distance_, false, false);

    std::vector<std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>> self_collision_tsdf_constraints;
    for (const std::string& collision_link: collision_group_names) {
      if (collision_link == "arm_link_0") { // arm_link_0 is always in collision, TODO remove hardcoded links
        continue;
      }
      for (const collision_detection::CollisionSphere& sphere: robot_model_->getCollisionSpheres(collision_link)) {
        CollisionObject collision_object(sphere, collision_link, getJointPadding(collision_link, SELF_COLLISION_TSDF));
        ceres::CostFunction* cost_function = SdfContactConstraint::Create(
              robot_model_, free_joints_, param_idx_to_chain_idx, default_chain_angles, Eigen::Isometry3d::Identity(), robot_base_tsdf, collision_object);
        std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function, 0, history.addConstraint());
        wrapper->setName("constraint_self_collision_tsdf_" + collision_link);
        self_collision_tsdf_constraints.push_back(wrapper);
        opt_.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
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
            ceres::CostFunction* cost_function = SelfContactConstraint::Create(
                  robot_model_, free_joints_, param_idx_to_chain_idx, default_chain_angles, obj1, obj2);
            std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> wrapper = std::make_shared<ceres_nlopt_wrapper::CeresCostFunctionWrapper>(cost_function, 0, history.addConstraint());
            wrapper->setName("constraint_self_collision_" + link_name_1 + "_vs_" + link_name_2);
            self_collision_constraints.push_back(wrapper);
            opt_.add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
          }
        }
      }
    }
  //  ROS_INFO_STREAM("Created " << self_collision_constraints.size() << " self-collision constraints.");
    STOP_TIMING_AVG

    // Fill in initial joint state
    std::vector<double> parameters = jointVectorToParamVector(initial_angles);
    ceres_nlopt_wrapper::applyBounds(lower_bounds_, upper_bounds_, parameters);

  //  stability_cost.checkGradient(parameters);

    // Check for collisions in solution
    std::vector<ceres_nlopt_wrapper::ConstraintViolation> constraint_violations;
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, environment_constraints, 0.001, constraint_violations)) {
      ROS_WARN_STREAM("Detected environment collision before optimization!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        ROS_WARN_STREAM(violation.toString());
      }
    }
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_tsdf_constraints, 0.001, constraint_violations)) {
      ROS_WARN_STREAM("Detected self-collision with tsdf before optimization!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        ROS_WARN_STREAM(violation.toString());
      }
    }
    if (!ceres_nlopt_wrapper::checkConstraints(parameters, self_collision_constraints, 0.001, constraint_violations)) {
      ROS_WARN_STREAM("Detected self-collision before optimization!");
      for (const ceres_nlopt_wrapper::ConstraintViolation& violation: constraint_violations) {
        ROS_WARN_STREAM(violation.toString());
      }
    }

    START_TIMING("Arm optimization")
    double minf = std::nan("");
    auto start = std::chrono::high_resolution_clock::now();
    nlopt::result nlopt_result;
    try {
      if (enabled_) {
        nlopt_result = opt_.optimize(parameters, minf);
        ROS_INFO_STREAM("Optimization returned " << ceres_nlopt_wrapper::resultToString(nlopt_result));
      }
    }
    catch (const std::invalid_argument& e) {
      ROS_ERROR_STREAM("Optimization failed with invalid argument error: " << e.what());
    }
    catch (const nlopt::roundoff_limited& e) {
      ROS_ERROR_STREAM("Optimization failed: " << e.what());
    }
    catch (const std::runtime_error& e) {
      ROS_ERROR_STREAM("Optimization failed: " << e.what());
    }

    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
    ROS_INFO_STREAM("Arm optimization took " << stability_cost.getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                    << ceres_nlopt_wrapper::vecToString(parameters) << ", Final cost: " << minf);

    // Subsidiary optimization
    if (sub_opt_) {
      start = std::chrono::high_resolution_clock::now();
      stability_cost.resetEvaluationCounter();
      sub_opt_->set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &stability_cost);

      sub_opt_->remove_inequality_constraints();
      for (const std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>& wrapper: environment_constraints) {
        sub_opt_->add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
      }
      for (const std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>& wrapper: self_collision_tsdf_constraints) {
        sub_opt_->add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
      }
      for (const std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper>& wrapper: self_collision_constraints) {
        sub_opt_->add_inequality_constraint(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, wrapper.get(), 1e-8);
      }

      try {
        nlopt_result = sub_opt_->optimize(parameters, minf);
        ROS_INFO_STREAM("Optimization returned " << ceres_nlopt_wrapper::resultToString(nlopt_result));
      }
      catch (const std::invalid_argument& e) {
        ROS_ERROR_STREAM("Optimization failed with invalid argument error: " << e.what());
      }
      catch (const nlopt::roundoff_limited& e) {
        ROS_ERROR_STREAM("Optimization failed: " << e.what());
      }
      catch (const std::runtime_error& e) {
        ROS_ERROR_STREAM("Optimization failed: " << e.what());
      }

      elapsed = std::chrono::high_resolution_clock::now() - start;
      ROS_INFO_STREAM("Arm optimization took " << stability_cost.getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms. Solution: "
                      << ceres_nlopt_wrapper::vecToString(parameters) << ", Final cost: " << minf);
    }

    const ceres_nlopt_wrapper::OptimizationResult* best_result = history.getBestResult();
    if (best_result) {
      parameters = best_result->parameters;
    } else {
      ROS_WARN_STREAM("Arm optimization history does not contain any valid solution.");
    }


    ROS_INFO_STREAM("Arm optimization final solution: " << ceres_nlopt_wrapper::vecToString(parameters) << ", cost: " << minf);
    STOP_TIMING_AVG

  //  ROS_INFO_STREAM("History: " << std::endl << history.toString());


    // Check for collisions in solution
  //  std::vector<ceres_nlopt_wrapper::ConstraintViolation> constraint_violations;
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
    // Compose result
    for (unsigned int i = 0; i < parameters.size(); i++) {
      result.joint_angles[param_idx_to_jnt_idx_[i]] = parameters[i];
    }
    result.final_cost = minf;
  }

  // Compute COM
  robot_model_->computeCenterOfMass(result.joint_angles, result.center_of_mass);
  Eigen::Isometry3d com_pose = Eigen::Isometry3d::Identity();
  com_pose.translation() = robot_pose * result.center_of_mass;
  // Compute stability
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(contact_hull_points, com_pose.translation());
//  std::vector<double> stabilities;
//  hector_stability_metrics::computeNormalizedEnergyStabilityMargin<double>(contact_hull_points, stabilities, com_pose.translation());
  result.stability = *std::min_element(stabilities.begin(), stabilities.end());

  // Publish result com
  sdf_contact_estimation::publishPose(result_com_pub_, com_pose, "world");
  // Publish result stabilities
  visualization_msgs::MarkerArray convex_hull_array_msg;
  sdf_contact_estimation::createPointMarkerArray(contact_hull_points, convex_hull_array_msg, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
  sdf_contact_estimation::createConvexHullMarkerArrayWithStabilities(contact_hull_points, stabilities, convex_hull_array_msg, 6.0);
  sdf_contact_estimation::deleteAllMarkers(convex_hull_pub_);
  convex_hull_pub_.publish(convex_hull_array_msg);

  return result;
}

}
