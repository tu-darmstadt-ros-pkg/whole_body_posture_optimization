#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <iomanip>

#include <pluginlib/class_loader.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>

#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>
#include <voxblox/io/layer_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <whole_body_posture_optimization/timing.h>
#include <whole_body_posture_optimization/common.h>

INIT_TIMING

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh,
                                                           const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                           const sdf_contact_estimation::SdfModelPtr& sdf_model)
  : nh_(nh), pose_predictor_(pose_predictor), sdf_model_(sdf_model), stability_limit_(0.0)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(nh_);

  ros::NodeHandle flipper_nh(nh, "flipper_optimization");
  flipper_optimization_ = std::make_shared<FlipperOptimization>(flipper_nh, robot_model_, pose_predictor_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_, sdf_model_);
//  ros::NodeHandle combined_nh(nh, "combined_optimization");
//  combined_optimization_ = std::make_shared<CombinedOptimization>(combined_nh, robot_model_, contact_estimation_);

  // Publishers
  trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 10, true);
  sdf_pub_ = nh_.advertise<voxblox_msgs::Layer>("planning_sdf", 10, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>("input_path", 10, true);
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
  nh.param<std::string>("trajectory_planner", trajectory_planner_name_, "ompl_interface/OMPLPlanner");

  return true;
}

std::shared_ptr<RobotModel> WholeBodyPostureOptimization::getRobotModel() const
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

std::vector<PostureOptimizationResult> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::Path& path,
                                                                                         const robot_state::RobotStateConstPtr& start_state)
{
  // Iterate over path poses and run optimization
  path_pub_.publish(path);
  std::vector<PostureOptimizationResult> results;
  if (path.poses.empty()) {
    ROS_WARN_STREAM("Can't find optimal postures, path is empty.");
    return results;
  }
  PostureOptimizationResult last_result;
  robot_state::RobotStatePtr previous_state;
  bool first = true;
  int counter = 0;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    ROS_INFO_STREAM("Pose: " << counter);
    counter++;
    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    if (first && start_state) {
      // We received a start state. The first pose of the path is the current pose,
      // so the state does not need to be optimized
      results.emplace_back();
      previous_state = std::make_shared<robot_state::RobotState>(*start_state);
      first = false;
    } else {
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), previous_state);
      robot_model_->updateJointAngles(last_result.result_joint_angles);
      previous_state = std::make_shared<robot_state::RobotState>(robot_model_->getMoveitRobotState());
      updateMoveitRobotState(*previous_state, last_result.result_pose);

      results.push_back(last_result);
      if (!last_result.success) {
        ROS_ERROR_STREAM("Posture optimization failed.");
        return results;
      }

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
  }

  return results;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Isometry3d& pose, const std::vector<double>& init_joint_angles,
                                                                           const robot_state::RobotStateConstPtr& previous_state)
{
  if (optimization_type_ == SPLIT) {
    return findOptimalPostureSplit(pose, init_joint_angles, previous_state);
  } else if (optimization_type_ == COMBINED) {
    ROS_ERROR_STREAM("Combined optimization currently not supported.");
//    return findOptimalPostureCombined(pose, init_joint_angles, previous_joint_angles, init_contact_result);
    return {};
  } else {
    ROS_ERROR_STREAM("Unknown optimization type");
    return {};
  }
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPostureSplit(const Eigen::Isometry3d& pose, const std::vector<double>& init_joint_angles,
                                                                                const robot_state::RobotStateConstPtr& previous_state)
{
  START_TIMING("findOptimalPostureSplit")
  ROS_INFO_STREAM("**********************************************");
  ROS_INFO_STREAM("Requested pose: " << poseToString(pose));
  ROS_INFO_STREAM("**********************************************");

  PostureOptimizationResult result;
  result.requested_pose = pose;
  result.init_joint_angles = init_joint_angles;

  Eigen::Isometry3d estimated_pose = pose; // Estimated pose of the robot at the requested position
  if (previous_state) {
    // Use last estimated pose to update estimate, if available
    estimated_pose = sdf_contact_estimation::updatePoseFromLastResult(pose, previous_state->getJointTransform("world_virtual_joint"), true, false);
  }

  // Re-run contact estimation at new position
//  sdf_contact_estimation::ContactEstimationResult updated_contact;
//  updated_contact false= flipper_optimization_->computeContactEstimation(*previous_joint_angles, estimated_pose, base_to_com);

  // Optimize
  ROS_INFO_STREAM("Estimated pose before optimization: " << poseToString(estimated_pose));
  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(estimated_pose, init_joint_angles, previous_state);
  result.result_pose = flipper_result.pose;
  result.support_polygon = flipper_result.support_polygon;
  result.result_joint_angles = flipper_result.joint_state;
  result.stability_flipper_opt = flipper_result.stability;
  result.flipper_cost = flipper_result.final_cost;

  // Optimize the arm
  ArmOptimizationResult arm_result = arm_optimization_->optimize(init_joint_angles, flipper_result.support_polygon.contact_hull_points,
                                                                 flipper_result.pose, flipper_result.joint_state);
  result.result_joint_angles = arm_result.joint_angles;
  result.result_com = arm_result.center_of_mass;
  result.stability_arm_opt = arm_result.stability;
  result.arm_cost = arm_result.final_cost;

  result.success = result.stability_arm_opt > 0;

  // Publish results
  robot_model_->updateRobotState(result.result_pose, result.result_joint_angles);
  robot_model_->publishRobotState();
  robot_model_->publishCollisionSpheres();

  ROS_INFO_STREAM("Stability: " << result.stability_arm_opt);

  STOP_TIMING_AVG

  return result;
}

//PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPostureCombined(const Eigen::Isometry3d& pose, const std::vector<double>& init_joint_angles,
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
//  Eigen::Isometry3d base_to_com(Eigen::Translation3d(result.init_com));

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

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result,
                                                                                       std::vector<size_t>& trajectory_start_indices,
                                                                                       const robot_state::RobotStateConstPtr& trajectory_start_state)
{
  START_TIMING("Trajectory planning")
  std::string group_with_world = complete_robot_with_world_group_;
  robot_trajectory::RobotTrajectory trajectory_with_world(robot_model_->getMoveitRobotModel(), group_with_world);
  std::string group = complete_robot_group_;
  robot_trajectory::RobotTrajectory trajectory(robot_model_->getMoveitRobotModel(), group);
  if (opt_result.size() < 2) {
    ROS_WARN_STREAM("Can't compute timed trajectory, path is empty.");
    return trajectory;
  }
  if (!opt_result.back().success) {
    ROS_WARN_STREAM("Posture optimization failed, can't compute trajectory");
    return trajectory;
  }

  const robot_state::JointModelGroup* joint_model_group = robot_model_->getMoveitRobotModel()->getJointModelGroup(group);

  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());

  // Load the trajectory planner
  pluginlib::ClassLoader<planning_interface::PlannerManager> planner_plugin_loader("moveit_core", "planning_interface::PlannerManager");
  std::vector<std::string> available_planners = planner_plugin_loader.getDeclaredClasses();
  if (std::find(available_planners.begin(), available_planners.end(), trajectory_planner_name_) == available_planners.end()) {
    std::stringstream ss;
    ROS_ERROR_STREAM("Can't find moveit trajectory planner with the name '" << trajectory_planner_name_ << "'. Available planners are: " << sdf_contact_estimation::vectorToString(available_planners));
    if (!available_planners.empty()) {
      trajectory_planner_name_ = available_planners[0];
      ROS_ERROR_STREAM("Using planner '" << trajectory_planner_name_ << "' instead.");
    } else {
      ROS_ERROR_STREAM("No planners found.");
      return trajectory;
    }
  }
  planning_interface::PlannerManagerPtr planner_instance(planner_plugin_loader.createUnmanagedInstance(trajectory_planner_name_));
  if (!planner_instance->initialize(robot_model_->getMoveitRobotModel(), nh_.getNamespace())) {
    ROS_FATAL_STREAM("Could not initialize planner instance");
    return trajectory;
  }
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");

  std::vector<Eigen::Isometry3d> full_world_poses;
  std::vector<size_t> trajectory_start_index;
  double total_distance = 0;
  double total_rotation = 0;
  for (unsigned int i = 0; i < opt_result.size()-1; i++) {
    planning_interface::MotionPlanRequest req;
    req.group_name = group;
    req.allowed_planning_time = 2.0;
    // set start state
    if (i == 0 && trajectory_start_state) {
      moveit::core::robotStateToRobotStateMsg(*trajectory_start_state, req.start_state);
      planning_scene->setCurrentState(*trajectory_start_state);
    } else {
      robot_model_->updateJointAngles(opt_result[i].result_joint_angles);
      robot_state::RobotState start_state = robot_model_->getMoveitRobotState();
      planning_scene->setCurrentState(start_state);
      moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
    }

    // set goal state
    robot_model_->updateJointAngles(opt_result[i+1].result_joint_angles);
    robot_state::RobotState goal_state = robot_model_->getMoveitRobotState();
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(goal_constraint);


    // compute trajectory from start to goal
    planning_interface::MotionPlanResponse res;
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    if (!context) {
      ROS_ERROR_STREAM("Failed to construct planning context. Error: " << res.error_code_);
      return trajectory_with_world;
    }
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      break;
    }

    // The moveit trajectory planner can't handle/interpolate the virtual world joint
    // So we do it ourselves and add the world virtual joint to all states
    Eigen::Isometry3d start_pose;
    if (i == 0 && trajectory_start_state) {
      start_pose = trajectory_start_state->getJointTransform("world_virtual_joint");
    } else {
      start_pose = opt_result[i].result_pose;
    }
    Eigen::Isometry3d end_pose = opt_result[i+1].result_pose;

    Eigen::Vector3d start_position = start_pose.translation();
    Eigen::Vector3d end_position = end_pose.translation();
//    Eigen::Vector3d start_position = opt_result[i].requested_pose.translation();
//    start_position.z() = opt_result[i].result_contact.estimated_pose.translation().z();
//    Eigen::Vector3d end_position = opt_result[i+1].requested_pose.translation();
//    end_position.z() = opt_result[i+1].result_contact.estimated_pose.translation().z();
    Eigen::Vector3d position_diff = end_position - start_position;
    Eigen::Quaterniond start_orientation(start_pose.linear());
    Eigen::Quaterniond end_orientation(end_pose.linear());

    std::vector<Eigen::Isometry3d> world_poses;
    world_poses.reserve(res.trajectory_->getWayPointCount());
    for (unsigned int j = 0; j < res.trajectory_->getWayPointCount()-1; j++) {
      double t = static_cast<double>(j) / (res.trajectory_->getWayPointCount()-1);
      Eigen::Quaterniond interpolated_orientation = start_orientation.slerp(t, end_orientation);
      Eigen::Isometry3d interpolated_pose(interpolated_orientation);
      Eigen::Vector3d interpolated_position = start_position + t * position_diff;
      interpolated_pose.translation() = interpolated_position;
      world_poses.push_back(interpolated_pose);

      double distance = (t * position_diff).norm();

      Eigen::Quaterniond rotation_diff = start_orientation.inverse() * interpolated_orientation;
      Eigen::Vector3d euler_angles = quatToEuler(rotation_diff);
      double yaw_distance = euler_angles(0);

      Eigen::Isometry3d odom_fake_pose = Eigen::Isometry3d::Identity();
      odom_fake_pose.translation().x() = total_distance + distance; // TODO this does not account for driving direction change
      odom_fake_pose.translation().z() = total_rotation + yaw_distance; // We use translation z for rotation. I consider this a work-around, not a hack!
      updateMoveitRobotState(*res.trajectory_->getWayPointPtr(j), odom_fake_pose);
    }
    total_distance += position_diff.norm();
    Eigen::Quaterniond rotation_diff = start_orientation.inverse() * end_orientation;
    Eigen::Vector3d euler_angles = quatToEuler(rotation_diff);
    double yaw_distance = euler_angles(0);
    total_rotation += yaw_distance;

    // Save start index of this trajectory part
    trajectory_start_indices.push_back(trajectory_with_world.getWayPointCount());
    // Append trajectory parts to full trajectory
    if (i != opt_result.size()-2) {
      // Append to full trajectory (minus the end point, which would be a duplicate of the next start)
      trajectory_with_world.append(*res.trajectory_, 0, 0, res.trajectory_->getWayPointCount()-1);
    } else {
      // Keep end point on last trajectory
//      updateMoveitRobotState(*res.trajectory_->getLastWayPointPtr(), opt_result[i+1].result_pose);
      Eigen::Isometry3d odom_fake_pose = Eigen::Isometry3d::Identity();
      odom_fake_pose.translation().x() = total_distance;
      odom_fake_pose.translation().z() = total_rotation;
      updateMoveitRobotState(*res.trajectory_->getLastWayPointPtr(), odom_fake_pose);
      world_poses.push_back(end_pose);
      trajectory_with_world.append(*res.trajectory_, 0);
    }
    full_world_poses.insert(full_world_poses.end(), world_poses.begin(), world_poses.end());
  }
  // Debug: print out trajectories
  //  ROS_INFO_STREAM(vecToString(trajectory_start_index));
//  ROS_INFO_STREAM(std::endl << robotTrajectoryToString(trajectory_with_world));

  // Perform time parameterization
  trajectory_processing::IterativeSplineParameterization time_parameterization(false);
  time_parameterization.computeTimeStamps(trajectory_with_world);

  // Write back world poses
  for (unsigned int i = 0; i < trajectory_with_world.getWayPointCount(); ++i) {
    updateMoveitRobotState(*trajectory_with_world.getWayPointPtr(i), full_world_poses[i]);
  }

  publishTrajectory(trajectory_with_world);

  STOP_TIMING_AVG

  ROS_INFO_STREAM("Trajectory duration: " << trajectory_with_world.getDuration() << " s.");
  // Planning group must not contain world, because otherwise moveit wants a controller for floating joints
  trajectory.append(trajectory_with_world, 0);
  return trajectory;
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result,
                                                                                       const robot_state::RobotStateConstPtr& start_state)
{
  std::vector<size_t> trajectory_start_indices;
  return computeTimedTrajectory(opt_result, trajectory_start_indices, start_state);
}

void WholeBodyPostureOptimization::publishTrajectory(const robot_trajectory::RobotTrajectory &robot_trajectory)
{
  moveit_msgs::DisplayTrajectory display_msg;
//  display_msg.model_id = "";

  moveit::core::robotStateToRobotStateMsg(robot_trajectory.getFirstWayPoint(), display_msg.trajectory_start);

  moveit_msgs::RobotTrajectory trajectory_msg;
  robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);
  display_msg.trajectory.push_back(trajectory_msg);

  trajectory_pub_.publish(display_msg);
}

void WholeBodyPostureOptimization::printResultInfo(const std::vector<PostureOptimizationResult> &results)
{
  if (results.empty()) {
    return;
  }
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
robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeOptimalTrajectory(
    const nav_msgs::Path& path, const moveit::core::RobotStateConstPtr& start_state)
{
  auto result = findOptimalPostures(path, start_state);
  auto trajectory = computeTimedTrajectory(result, start_state);
  return trajectory;
}

//void WholeBodyPostureOptimization::saveResultInfo(const std::vector<PostureOptimizationResult>& results, const std::vector<robot_trajectory::RobotTrajectory>& trajectories,
//                                                  std::string save_path)
//{
//  std::ofstream file;
//  file.open(save_path);
//
//  file << "time pos_x pos_y flipper_cost arm_cost flipper_stability stability" << std::endl;
//  double time_from_start = 0.0;
//  for (unsigned int i = 0; i < results.size(); i++) {
//    const PostureOptimizationResult& result = results[i];
//    file << time_from_start << " ";
//    file << result.requested_pose.translation().x() << " ";
//    file << result.requested_pose.translation().y() << " ";
//    file << result.flipper_cost << " ";
//    file << result.arm_cost << " ";
//    file << result.stability_flipper_opt << " ";
//    file << result.stability_arm_opt << std::endl;
//    if (i < trajectories.size()) {
//      time_from_start += trajectories[i].getDuration();
//    }
//  }
//
//  file.close();
//}

}
