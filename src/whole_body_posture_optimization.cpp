#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <iomanip>

#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <voxblox/io/layer_io.h>

#include <whole_body_posture_optimization/timing.h>
#include <whole_body_posture_optimization/common.h>

#include <voxblox_msgs/Layer.h>

INIT_TIMING

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
  : nh_(nh), optimization_type_(SPLIT), stability_limit_(0.0)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(nh_);
  ros::NodeHandle contact_estimation_nh(nh, "contact_estimation");
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(contact_estimation_nh, robot_shape_gen);
  ros::NodeHandle flipper_nh(nh, "flipper_optimization");
  flipper_optimization_ = std::make_shared<FlipperOptimization>(flipper_nh, robot_model_, robot_shape_gen, contact_estimation_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_, contact_estimation_);
//  ros::NodeHandle combined_nh(nh, "combined_optimization");
//  combined_optimization_ = std::make_shared<CombinedOptimization>(combined_nh, robot_model_, contact_estimation_);

  trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 10, true);
  sdf_pub_ = nh_.advertise<voxblox_msgs::Layer>("planning_sdf", 10, true);
}

bool WholeBodyPostureOptimization::loadParameters(const ros::NodeHandle &nh)
{
  // Set ros verbosity level to debug
  bool debug;
  nh.param<bool>("posture_optimization_debug", debug, false);
  if (debug) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  if (!nh.getParam("default_joint_state", default_joint_state_)) {
    ROS_ERROR_STREAM("No default joint state found.");
    return false;
  }

  nh.param("stability_limit", stability_limit_, 0.0);
  nh.param("front_tip_support_area_limit", front_tip_support_area_limit_, 0.5);
  nh.param<bool>("stepping", stepping_, false);
  nh.param<int>("stepping_start", stepping_start_, 0);
  nh.param<std::string>("sdf_save_folder", sdf_save_folder_, "");

  std::string optimization_type_str;
  nh.param<std::string>("optimization_type", optimization_type_str, "split");
  if (optimization_type_str == "split") {
    optimization_type_ = SPLIT;
  } else if (optimization_type_str == "combined") {
    optimization_type_ = COMBINED;
  } else {
    ROS_ERROR_STREAM("Unknown optimization type '" << optimization_type_str << "'.");
  }

  nh.param<std::string>("complete_robot_group", complete_robot_group_, "complete_robot_group");
  nh.param<std::string>("complete_robot_with_world_group", complete_robot_with_world_group_, "complete_robot_group_with_world");

  return true;
}

void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf)
{
  contact_estimation_->loadTsdf(tsdf, 0.4f, true);
  saveSdfToFile(sdf_save_folder_);

  // Publish esdf layer
  const bool only_updated = false;
  voxblox_msgs::Layer layer_msg;
  voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(contact_estimation_->getSdf().getEsdf()->getESDF()->getEsdfLayer(), only_updated,
                                 &layer_msg);
  sdf_pub_.publish(layer_msg);
}

void WholeBodyPostureOptimization::updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  contact_estimation_->loadCloud(cloud, true);
  saveSdfToFile(sdf_save_folder_);
}

bool WholeBodyPostureOptimization::loadSdfFromFile(std::string tsdf_file_path, float truncation_distance)
{
  if (contact_estimation_->loadSdfFromFile(tsdf_file_path, truncation_distance, true)) {
    saveSdfToFile(sdf_save_folder_);
    return true;
  }
  return false;
}

bool WholeBodyPostureOptimization::saveSdfToFile(std::string save_folder)
{
  // Save the current tsdf for later debugging
  if (save_folder != "") {
    std::string time_stamp = getTimestamp();
    std::string save_path =  save_folder + "/tsdf_map_" + time_stamp + ".tsdf";
    ROS_INFO_STREAM("Saving tsdf to " << save_path);
    contact_estimation_->saveTsdfToFile(save_path);
    save_path =  save_folder + "/esdf_map_" + time_stamp + ".esdf";
    contact_estimation_->saveEsdfToFile(save_path);
  }
  return true;
}

const std::shared_ptr<RobotModel> WholeBodyPostureOptimization::getRobotModel() const
{
  return robot_model_;
}

void WholeBodyPostureOptimization::setDefaultJointPositions(const std::vector<double>& default_joint_positions)
{
  default_joint_state_ = default_joint_positions;
}

const std::vector<double> &WholeBodyPostureOptimization::getDefaultJointPositions()
{
  return default_joint_state_;
}

std::vector<PostureOptimizationResult> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::Path& path)
{
  // Iterate over path poses and run optimization
  std::vector<PostureOptimizationResult> results;
  if (path.poses.empty()) {
    ROS_WARN_STREAM("Can't find optimal postures, path is empty.");
    return results;
  }
  PostureOptimizationResult last_result;
  bool first = true;
  int counter = 0;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    ROS_INFO_STREAM("Pose: " << counter);
    counter++;
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    if (first) {
      last_result = findOptimalPosture(pose, getDefaultJointPositions());
      first = false;
    } else {
      if (last_result.result_contact.estimated_pose.translation().x() > pose.translation().x()) {
        // Next pose on path is behind last estimated pose
        // Skip this pose
        // TODO: Movement direction is hardcoded as X axis
        ROS_WARN_STREAM("Skipping this pose because " << pose.translation().x() << " < " << last_result.result_contact.estimated_pose.translation().x() << "!");
        continue;
      }
//      last_result = findOptimalPosture(pose, last_result.result_joint_angles, last_result.result_joint_angles, &last_result.result_contact);
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), &last_result);
    }
    if (!last_result.success) {
      ROS_ERROR_STREAM("Posture optimization failed.");
      return results;
    }
    results.push_back(last_result);

    if (last_result.stability_arm_opt < stability_limit_) {
      ROS_WARN_STREAM("Robot unstable (" << last_result.stability_arm_opt << " < " << stability_limit_ << "). Aborting!");
      break;
    }

    if (stepping_ && stepping_start_ < (counter+1)) {
      std::cout << '\n' << "Press ENTER to continue...";
      do {
      } while (ros::ok() && std::cin.get() != '\n');
    }
    if (!ros::ok()) {
      break;
    }
  }

  return results;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_angles,
                                                                           const PostureOptimizationResult* previous_result)
{
  if (optimization_type_ == SPLIT) {
    return findOptimalPostureSplit(pose, init_joint_angles, previous_result);
  } else if (optimization_type_ == COMBINED) {
    ROS_ERROR_STREAM("Combined optimization currently not supported.");
//    return findOptimalPostureCombined(pose, init_joint_angles, previous_joint_angles, init_contact_result);
    return PostureOptimizationResult();
  } else {
    ROS_ERROR_STREAM("Unknown optimization type");
    return PostureOptimizationResult();
  }
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPostureSplit(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_angles,
                                                                                const PostureOptimizationResult* previous_result)
{
  START_TIMING("findOptimalPostureSplit")
  ROS_INFO_STREAM("**********************************************");
  ROS_INFO_STREAM("Requested pose: " << poseToString(pose));
  ROS_INFO_STREAM("**********************************************");

  PostureOptimizationResult result;
  result.requested_pose = pose;
  result.init_joint_angles = init_joint_angles;

  // Use previous solution as current joint state, if available
  // otherwise, use default
  const std::vector<double>* previous_joint_angles;
  if (previous_result) {
    previous_joint_angles = &previous_result->result_joint_angles;
  } else {
    previous_joint_angles = &init_joint_angles;
  }

  // Compute center of mass
  // TODO we should be able to just take this from the previous time step. com shouldn't have changed
  robot_model_->computeCenterOfMass(*previous_joint_angles, result.init_com);
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
  base_to_com.translation() = result.init_com;

  // Estimate our new position  
  Eigen::Affine3d estimated_pose = pose; // Estimated pose of the robot at the requested position
  // Use last estimated pose to update estimate, if available
  if (previous_result) {
    result.init_contact = previous_result->result_contact;
//    estimated_pose = sdf_contact_estimation::updatePoseFromLastResult(pose, previous_result->result_contact.estimated_pose);
  }
  // Re-run contact estimation at new position
  sdf_contact_estimation::ContactEstimationResult updated_contact;
//  updated_contact false= flipper_optimization_->computeContactEstimation(*previous_joint_angles, estimated_pose, base_to_com);

  // Collect requests to optimization
  std::vector<Eigen::Affine3d> request_com;
  std::vector<Eigen::Affine3d> request_pose;

  // Check if updating contact was successful
  if (!updated_contact.stable) {
    ROS_WARN_STREAM("Robot fell during contact update, using estimate of previous iteration");
  } else {
    // Update successful
    estimated_pose = updated_contact.estimated_pose;
    result.initial_stability = updated_contact.min_stability;

    ROS_INFO_STREAM("Convex hull size: " << updated_contact.support_area);

    // Check if tipping to front makes sense
    if (updated_contact.support_area < front_tip_support_area_limit_) {
      // Find axis we want to tip over
      double min_stability = std::numeric_limits<double>::max();
      unsigned int front_idx = 0;
      for (unsigned int i = 0; i < updated_contact.convex_hull_points.size(); i++) {
        const Eigen::Vector3d& p0 = updated_contact.convex_hull_points[i];
        const Eigen::Vector3d& p1 = updated_contact.convex_hull_points[(i+1) % updated_contact.convex_hull_points.size()];

        // Transform points to base link
        Eigen::Vector3d p0_base = updated_contact.estimated_pose.inverse(Eigen::Isometry) * p0;
        Eigen::Vector3d p1_base = updated_contact.estimated_pose.inverse(Eigen::Isometry) * p1;

        // Check if forward edge
        Eigen::Vector3d a = p1_base - p0_base;
        Eigen::Vector3d cross = Eigen::Vector3d::UnitX().cross(a);
        if (cross.z() < -0.01) {
          // Check if stability is lower
          double stability = updated_contact.stabilities[i];
          if (stability < min_stability) {
            min_stability = stability;
            front_idx = i;
          }
        }
      }
      ROS_INFO_STREAM("Trying to tip over front axis " << front_idx);

      // Move COM to front
      const Eigen::Vector3d& p0 = updated_contact.convex_hull_points[front_idx];
      const Eigen::Vector3d& p1 = updated_contact.convex_hull_points[(front_idx+1) % updated_contact.convex_hull_points.size()];
      Eigen::Vector3d com_world = updated_contact.estimated_pose * result.init_com;

      // Project to 2D
      Eigen::Vector3d p0_xy(p0.x(), p0.y(), 0);
      Eigen::Vector3d p1_xy(p1.x(), p1.y(), 0);
      Eigen::Vector3d com_xy(com_world.x(), com_world.y(), 0);

      // Compute orthogonal projection
      Eigen::Vector3d axis = p1_xy - p0_xy;
      Eigen::Vector3d axis_norm = axis / axis.norm();
      Eigen::Vector3d axis_normal = (Eigen::Matrix3d::Identity() - axis_norm * axis_norm.transpose()) * (p1_xy - com_xy); // l_i
      Eigen::Vector3d axis_normal_norm = axis_normal / axis_normal.norm();

      // Move com across axis
      Eigen::Vector3d com_world_new = com_xy + axis_normal + 0.05 * axis_normal_norm;
      com_world_new.z() = com_world.z();
      Eigen::Affine3d new_base_to_com(Eigen::Translation3d(updated_contact.estimated_pose.inverse(Eigen::Isometry) * com_world_new));

      // rerun contact estimation
      sdf_contact_estimation::ContactEstimationResult front_tip_contact = flipper_optimization_->computeContactEstimation(
            *previous_joint_angles, estimated_pose, new_base_to_com);

      // Push new com and contact to request list
      request_com.push_back(new_base_to_com);
      request_pose.push_back(front_tip_contact.estimated_pose);
    }
  }

  // Push back original updated pose
  request_com.push_back(base_to_com);
  request_pose.push_back(estimated_pose);

  // Collect results
  std::vector<PostureOptimizationResult> results;
  for(unsigned int i = 0; i < request_com.size(); i++) {
    ROS_INFO_STREAM("Estimated pose: " << poseToString(request_pose[i]));
    PostureOptimizationResult prelim_result = result;
    FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(request_pose[i], request_com[i], init_joint_angles, *previous_joint_angles);
    prelim_result.result_contact = flipper_result.contact_result;
    prelim_result.result_joint_angles = flipper_result.joint_state;
    prelim_result.stability_flipper_opt = flipper_result.contact_result.min_stability;
    prelim_result.flipper_cost = flipper_result.final_cost;

    // Optimize the arm
    ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points,
                                                                   flipper_result.contact_result.estimated_pose, *previous_joint_angles);
    prelim_result.result_joint_angles = arm_result.joint_angles;
    prelim_result.result_com = arm_result.center_of_mass;
    prelim_result.stability_arm_opt = arm_result.stability;
    prelim_result.arm_cost = arm_result.final_cost;

    results.push_back(prelim_result);
  }

  // Find best result
  if (!results.empty()) {
    unsigned int best_idx = 10000;
    double best_stability = -std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < results.size(); i++) {
      if (results[i].stability_arm_opt > best_stability) {
        best_idx = i;
        best_stability = results[i].stability_arm_opt;
      }
    }
    if (results.size() > 1) {
      ROS_INFO_STREAM("Chose result " << best_idx);
    }
    result = results[best_idx];
  } else {
    ROS_ERROR_STREAM("No result available!");
  }


  result.success = result.stability_arm_opt > 0;

  // Publish results
  robot_model_->updateRobotState(result.result_contact.estimated_pose, result.result_joint_angles);
  robot_model_->publishRobotState();
  robot_model_->publishCollisionSpheres();

  ROS_INFO_STREAM("Stability: " << result.stability_arm_opt);

  STOP_TIMING_AVG

  return result;
}

//PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPostureCombined(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_angles,
//                                                                                   const std::vector<double>& previous_joint_angles,
//                                                                                   const sdf_contact_estimation::ContactEstimationResult* init_contact_result)
//{
//  START_TIMING("findOptimalPostureCombined")
//  Eigen::Vector3d rpy = sdf_contact_estimation::rotToNormalizedRpy(pose.linear());
//  Eigen::Vector3d xyz = pose.translation();
//  ROS_INFO_STREAM("**********************************************");
//  ROS_INFO("Requested pose: [%f, %f, %f, %f, %f, %f]", xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));
//  ROS_INFO_STREAM("**********************************************");

//  PostureOptimizationResult result;
//  result.requested_pose = pose;
//  result.init_joint_angles = init_joint_angles;

//  if (!previous_joint_angles.empty()) {
//    robot_model_->computeCenterOfMass(previous_joint_angles, result.init_com);
//  } else {
//    robot_model_->computeCenterOfMass(init_joint_angles, result.init_com);
//  }
//  Eigen::Affine3d base_to_com(Eigen::Translation3d(result.init_com));

//  sdf_contact_estimation::ContactEstimationResult updated_init_contact;
//  if (init_contact_result != nullptr) {
//    result.init_contact = *init_contact_result;
//    updated_init_contact = *init_contact_result;
//  } else {
//    if (!previous_joint_angles.empty()) {
//      updated_init_contact = combined_optimization_->computeContactEstimation(previous_joint_angles, pose, base_to_com);
//    } else {
//      updated_init_contact = combined_optimization_->computeContactEstimation(init_joint_angles, pose, base_to_com);
//    }
//  }
//  if (updated_init_contact.convex_hull_points.empty()) {
//    ROS_WARN_STREAM("Updated init contact estimation failed.");
//      updated_init_contact.estimated_pose = pose;
//  } else {
//    result.initial_stability = updated_init_contact.min_stability;
//  }

//  CombinedOptimizationResult combined_result = combined_optimization_->optimize(init_joint_angles, pose, previous_joint_angles, updated_init_contact);
//  result.result_joint_angles = combined_result.joint_angles;
//  result.result_com = combined_result.center_of_mass;
//  result.stability_arm_opt = combined_result.contact_result.min_stability;
//  result.result_contact = combined_result.contact_result;
//  result.stability_flipper_opt = result.initial_stability;

//  robot_model_->updateRobotState(result.result_contact.estimated_pose, result.result_joint_angles);
//  robot_model_->publishRobotState();
//  robot_model_->publishCollisionSpheres();

//  STOP_TIMING_AVG

//  return result;
//}

std::vector<robot_trajectory::RobotTrajectory> WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result)
{
  std::vector<robot_trajectory::RobotTrajectory> trajectories;
  if (opt_result.size() < 2) {
    ROS_WARN_STREAM("Can't compute timed trajectory, path is empty.");
    return trajectories;
  }
  START_TIMING("Trajectory planning")
  std::string group = complete_robot_group_;
  std::string group_with_world = complete_robot_with_world_group_;
  const robot_state::JointModelGroup* joint_model_group = robot_model_->getMoveitRobotModel()->getJointModelGroup(group);

  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());

  pluginlib::ClassLoader<planning_interface::PlannerManager> planner_plugin_loader("moveit_core", "planning_interface::PlannerManager");
  std::vector<std::string> planner_names = planner_plugin_loader.getDeclaredClasses();
  ROS_INFO_STREAM("Available planners:");
  for (const std::string& planner: planner_names) {
    ROS_INFO_STREAM("-- " << planner);
  }
  std::string default_planner_name = planner_names[1];
  planning_interface::PlannerManagerPtr planner_instance(planner_plugin_loader.createUnmanagedInstance(default_planner_name));
  if (!planner_instance->initialize(robot_model_->getMoveitRobotModel(), nh_.getNamespace())) {
    ROS_FATAL_STREAM("Could not initialize planner instance");
    return trajectories;
  }
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = group;
  req.allowed_planning_time = 2.0;

  trajectory_processing::IterativeSplineParameterization time_parameterization;
  robot_trajectory::RobotTrajectory trajectory_tmp(robot_model_->getMoveitRobotModel(), group_with_world);
  for (unsigned int i = 0; i < opt_result.size()-1; i++) {
    // set start state
    robot_model_->updateJointAngles(opt_result[i].result_joint_angles);
    robot_state::RobotState start_state = robot_model_->getMoveitRobotState();
    planning_scene->setCurrentState(start_state);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

    // set goal state
    robot_model_->updateJointAngles(opt_result[i+1].result_joint_angles);
    robot_state::RobotState goal_state = robot_model_->getMoveitRobotState();
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(goal_constraint);


    // compute trajectory from start to goal
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    if (!context) {
      ROS_ERROR_STREAM("Failed to construct planning context. Error: " << res.error_code_);
      return trajectories;
    }
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      break;
    }

    // The moveit trajectory planner can't handle/interpolate the virtual world joint
    // So we do it ourselves and override the respective states
//    Eigen::Vector3d start_position = opt_result[i].result_contact.estimated_pose.translation();
//    Eigen::Vector3d end_position = opt_result[i+1].result_contact.estimated_pose.translation();
    Eigen::Vector3d start_position = opt_result[i].requested_pose.translation();
    start_position.z() = opt_result[i].result_contact.estimated_pose.translation().z();
    Eigen::Vector3d end_position = opt_result[i+1].requested_pose.translation();
    end_position.z() = opt_result[i+1].result_contact.estimated_pose.translation().z();

    Eigen::Vector3d position_diff = end_position - start_position;
    Eigen::Quaterniond start_orientation(opt_result[i].result_contact.estimated_pose.linear());
    Eigen::Quaterniond end_orientation(opt_result[i+1].result_contact.estimated_pose.linear());
    for (unsigned int j = 0; j < res.trajectory_->getWayPointCount(); j++) {
      double t = static_cast<double>(j) / res.trajectory_->getWayPointCount();
      Eigen::Quaterniond interpolated_orientation = start_orientation.slerp(t, end_orientation);
      Eigen::Affine3d interpolated_pose(interpolated_orientation);
      Eigen::Vector3d interpolated_position = start_position + t * position_diff;
      interpolated_pose.translation() = interpolated_position;
      robot_state::RobotStatePtr state = res.trajectory_->getWayPointPtr(j);
      updateMoveitRobotState(*state, interpolated_pose);
    }

    // Compute time parameterization and push to vector
    trajectory_tmp.clear();
    trajectory_tmp.append(*res.trajectory_, 0);
    time_parameterization.computeTimeStamps(trajectory_tmp);
    trajectories.push_back(trajectory_tmp);
  }

  publishTrajectory(trajectories);

  STOP_TIMING_AVG

  double duration = 0.0;
  for (const robot_trajectory::RobotTrajectory& trajectory: trajectories) {
    duration += trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount()-1);
  }
  ROS_INFO_STREAM("Trajectory duration: " << duration << " s.");
  return trajectories;
}

void WholeBodyPostureOptimization::publishTrajectory(const std::vector<robot_trajectory::RobotTrajectory> &robot_trajectory)
{
  if (robot_trajectory.empty()) {
    return;
  }
  moveit_msgs::DisplayTrajectory display_msg;
//  display_msg.model_id = "";

  moveit_msgs::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(robot_trajectory[0].getFirstWayPoint(), start_state_msg);
  display_msg.trajectory_start = start_state_msg;

  for (const robot_trajectory::RobotTrajectory& t: robot_trajectory) {
    moveit_msgs::RobotTrajectory trajectory_msg;
    t.getRobotTrajectoryMsg(trajectory_msg);
    display_msg.trajectory.push_back(trajectory_msg);
  }

  trajectory_pub_.publish(display_msg);
}

void WholeBodyPostureOptimization::printContactEstimationStatistics()
{
  double pct = static_cast<double>(contact_estimation_->getFailedEstimationCount()) / static_cast<double>(contact_estimation_->getTotalEstimationCount());
  ROS_INFO_STREAM("Failed contact estimations: [" << contact_estimation_->getFailedEstimationCount() << "/" << contact_estimation_->getTotalEstimationCount()
                  << "] (" << pct << " %)");
}

void WholeBodyPostureOptimization::printResultInfo(const std::vector<PostureOptimizationResult> &results)
{
  std::vector<std::string> names = {"initial_stability", "flipper_opt", "arm_opt"};
  std::vector<std::vector<double>> stabilities(names.size());
  for (const PostureOptimizationResult& result: results) {
    stabilities[0].push_back(result.initial_stability);
    stabilities[1].push_back(result.stability_flipper_opt);
    stabilities[2].push_back(result.stability_arm_opt);
  }

  const int column_width_name = 20;
  const int column_width_avg = 10;
  const int column_width_min = 10;
  std::stringstream ss;
  ss << "STABILITY INFOS:" << std::endl;
  ss << "|" << std::setw(column_width_name) << "Name " << "|" << std::setw(column_width_avg) << "Avg" << "|" << std::setw(column_width_min) << "Min" << "|" << std::endl;
  ss << line(column_width_name + column_width_avg + column_width_min + 5) << std::endl;
  for (unsigned int i = 0; i < stabilities.size(); i++) {
    std::string name = names[i];
    const std::vector<double>& s = stabilities[i];
    double min = *std::min_element(begin(s), end(s));
    double avg = std::accumulate(begin(s), end(s), 0.0)/s.size();
    ss << "|" << std::setw(column_width_name) << name << "|" << std::setw(column_width_avg) << avg << "|" << std::setw(column_width_min) << min << "|" << std::endl;
  }
  ROS_INFO_STREAM(ss.str());
}

void WholeBodyPostureOptimization::saveResultInfo(const std::vector<PostureOptimizationResult>& results, const std::vector<robot_trajectory::RobotTrajectory>& trajectories,
                                                  std::string save_path)
{
  std::ofstream file;
  file.open(save_path);

  file << "time pos_x pos_y flipper_cost arm_cost flipper_stability stability" << std::endl;
  double time_from_start = 0.0;
  for (unsigned int i = 0; i < results.size(); i++) {
    const PostureOptimizationResult& result = results[i];
    file << time_from_start << " ";
    file << result.requested_pose.translation().x() << " ";
    file << result.requested_pose.translation().y() << " ";
    file << result.flipper_cost << " ";
    file << result.arm_cost << " ";
    file << result.stability_flipper_opt << " ";
    file << result.stability_arm_opt << std::endl;
    if (i < trajectories.size()) {
      time_from_start += trajectories[i].getWayPointDurationFromStart(trajectories[i].getWayPointCount()-1);
    }
  }

  file.close();
}

}
