#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <iomanip>
#include <ctime>

#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <voxblox/io/layer_io.h>

#include <whole_body_posture_optimization/timing.h>

INIT_TIMING

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
  : nh_(nh), stability_limit_(0.0)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(nh_);
  ros::NodeHandle contact_estimation_nh(nh, "contact_estimation");
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(contact_estimation_nh, robot_shape_gen);
  ros::NodeHandle flipper_nh(nh, "flipper_optimization");
  flipper_optimization_ = std::make_shared<FlipperOptimization>(flipper_nh, robot_model_, robot_shape_gen, contact_estimation_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_, contact_estimation_);

  trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 10, true);
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
  nh.param<std::string>("tsdf_save_folder", tsdf_save_folder_, "");

  return true;
}

void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf)
{
  contact_estimation_->loadTsdf(tsdf, 0.4, true);
}

void WholeBodyPostureOptimization::updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  contact_estimation_->loadCloud(cloud, true);
}

bool WholeBodyPostureOptimization::loadTsdfFromFile(std::string tsdf_file_path, float truncation_distance)
{
  return contact_estimation_->loadTsdfFromFile(tsdf_file_path, truncation_distance, true, true);
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
  // Save the current tsdf for later debugging
  if (tsdf_save_folder_ != "") {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    std::string time_stamp = oss.str();
    std::string save_path =  tsdf_save_folder_ + "/tsdf_map_" + time_stamp + ".tsdf";
    contact_estimation_->saveTsdfToFile(save_path);
  }

  // Iterate over path poses and run optimization
  std::vector<PostureOptimizationResult> results;
  PostureOptimizationResult last_result;
//  ros::Rate rate(5);
  bool first = true;
  int counter = 0;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    ROS_INFO_STREAM("Pose: " << counter);
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    auto start = std::chrono::high_resolution_clock::now();
    if (first) {
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), std::vector<double>());
      first = false;
    } else {
//      last_result = findOptimalPosture(pose, last_result.result_joint_angles, last_result.result_joint_angles, &last_result.result_contact);
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), last_result.result_joint_angles, &last_result.result_contact);
    }
    results.push_back(last_result);
//    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
//    rate.sleep();

    if (last_result.stability_arm_opt < stability_limit_) {
      ROS_WARN_STREAM("Robot instable (" << last_result.stability_arm_opt << " < " << stability_limit_ << "). Aborting!");
      break;
    }
    counter++;
  }

  return results;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_angles,
                                                                           const std::vector<double> &previous_joint_angles,
                                                                           const sdf_contact_estimation::ContactEstimationResult* init_contact_result)
{
  START_TIMING("findOptimalPosture");
  Eigen::Vector3d rpy = sdf_contact_estimation::rotToNormalizedRpy(pose.linear());
  Eigen::Vector3d xyz = pose.translation();
  ROS_INFO_STREAM("**********************************************");
  ROS_INFO("Requested pose: [%f, %f, %f, %f, %f, %f]", xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));
  ROS_INFO_STREAM("**********************************************");

  PostureOptimizationResult result;
  result.requested_pose = pose;
  result.init_joint_angles = init_joint_angles;

  // TODO we should be able to just take this from the previous time step. com shouldn't have changed
  if (!previous_joint_angles.empty()) {
    robot_model_->computeCenterOfMass(previous_joint_angles, result.init_com);
  } else {
    robot_model_->computeCenterOfMass(init_joint_angles, result.init_com);
  }
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
  base_to_com.translation() = result.init_com;

  sdf_contact_estimation::ContactEstimationResult updated_init_contact;
  if (init_contact_result != NULL) {
    result.init_contact = *init_contact_result;
//    updated_init_contact = flipper_optimization_->computeContactEstimation(previous_joint_angles, pose, base_to_com, result.init_contact);
    updated_init_contact = *init_contact_result;
  } else {
    if (!previous_joint_angles.empty()) {
      updated_init_contact = flipper_optimization_->computeContactEstimation(previous_joint_angles, pose, base_to_com);
    } else {
      updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_angles, pose, base_to_com);
    }
  }
  if (updated_init_contact.convex_hull_points.empty()) {
    ROS_WARN_STREAM("Updated init contact estimation failed.");
      updated_init_contact.estimated_pose = pose;
  } else {
    result.initial_stability = updated_init_contact.min_stability;
  }

  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(pose, base_to_com, init_joint_angles, previous_joint_angles,
                                                                             updated_init_contact);
  result.result_contact = flipper_result.contact_result;
  result.result_joint_angles = flipper_result.joint_state;
  result.stability_flipper_opt = flipper_result.contact_result.min_stability;



  ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points,
                                                                 flipper_result.contact_result.estimated_pose, previous_joint_angles);
  result.result_joint_angles = arm_result.joint_angles;
  result.result_com = arm_result.center_of_mass;
  result.stability_arm_opt = arm_result.stability;

  robot_model_->updateRobotState(result.result_contact.estimated_pose, result.result_joint_angles);
  robot_model_->publishRobotState();
  robot_model_->publishCollisionSpheres();

  STOP_TIMING_AVG;

  return result;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d &pose, const std::vector<double> &init_joint_state, const std::vector<double> &previous_joint_angles)
{
  return findOptimalPosture(pose, init_joint_state, previous_joint_angles, NULL);
}

std::vector<robot_trajectory::RobotTrajectory> WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result)
{
  auto start = std::chrono::high_resolution_clock::now();

  std::string group = "complete_robot_group"; //TODO parameter
  std::string group_with_world = "complete_robot_group_with_world";
  std::vector<robot_trajectory::RobotTrajectory> trajectories;
  const robot_state::JointModelGroup* joint_model_group = robot_model_->getMoveitRobotModel()->getJointModelGroup(group);

  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());

  pluginlib::ClassLoader<planning_interface::PlannerManager> planner_plugin_loader("moveit_core", "planning_interface::PlannerManager");
  std::vector<std::string> planner_names = planner_plugin_loader.getDeclaredClasses();
  std::string default_planner_name = planner_names[0];
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

  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  robot_trajectory::RobotTrajectory trajectory_tmp(robot_model_->getMoveitRobotModel(), group_with_world);
  for (unsigned int i = 0; i < opt_result.size()-1; i++) {
    // set start state
    // x and y are taken from original path to prevent reversing during execution
    Eigen::Affine3d robot_pose(opt_result[i].result_contact.estimated_pose);
    robot_pose.translation() = Eigen::Vector3d(opt_result[i].requested_pose.translation().x(),
                                               opt_result[i].requested_pose.translation().y(),
                                               opt_result[i].result_contact.estimated_pose.translation().z());
    robot_model_->updateRobotState(robot_pose, opt_result[i].result_joint_angles);
    robot_state::RobotState start_state = robot_model_->getMoveitRobotState();
    planning_scene->setCurrentState(start_state);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

    // set goal state
    robot_model_->updateRobotState(opt_result[i+1].result_contact.estimated_pose, opt_result[i+1].result_joint_angles);
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
//      return trajectories;
      break;
    }

    // The moveit trajectory planner can't handle/interpolate the virtual world joint
    // So we do it ourselves and override the respective states
    Eigen::Vector3d start_position = opt_result[i].result_contact.estimated_pose.translation();
    Eigen::Vector3d end_position = opt_result[i+1].result_contact.estimated_pose.translation();
    Eigen::Vector3d position_diff = end_position - start_position;
    Eigen::Quaterniond start_orientation(opt_result[i].result_contact.estimated_pose.rotation());
    Eigen::Quaterniond end_orientation(opt_result[i+1].result_contact.estimated_pose.rotation());
    for (unsigned int j = 0; j < res.trajectory_->getWayPointCount(); j++) {
      double t = static_cast<double>(j) / res.trajectory_->getWayPointCount();
      robot_state::RobotStatePtr state = res.trajectory_->getWayPointPtr(j);
      Eigen::Vector3d interpolated_position = start_position + t * position_diff;
      Eigen::Quaterniond interpolated_orientation = start_orientation.slerp(t, end_orientation);
      Eigen::Affine3d interpolated_pose(interpolated_orientation);
      interpolated_pose.translation() = interpolated_position;
      updateMoveitRobotState(*state, interpolated_pose);
    }

    // Compute time parameterization and push to vector
    trajectory_tmp.clear();
    trajectory_tmp.append(*res.trajectory_, 0);
    time_parameterization.computeTimeStamps(trajectory_tmp);
    trajectories.push_back(trajectory_tmp);
  }

  publishTrajectory(trajectories);

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Trajectory planning took " << elapsed.count() << " s.");

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

}
