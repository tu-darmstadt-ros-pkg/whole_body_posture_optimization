#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
  : nh_(nh)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(nh_);
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(robot_shape_gen);
  contact_estimation_->loadParametersFromNamespace(nh_);
  ros::NodeHandle flipper_nh(nh, "flipper_optimization");
  flipper_optimization_ = std::make_shared<FlipperOptimization>(flipper_nh, robot_model_, robot_shape_gen, contact_estimation_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_, contact_estimation_);

  trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 10, true);
}

bool WholeBodyPostureOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if (!nh.getParam("default_joint_state", default_joint_state_)) {
    ROS_ERROR_STREAM("No default joint state found.");
    return false;
  }

  return true;
}

void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf)
{
  tsdf_ = tsdf;
  contact_estimation_->loadTsdf(tsdf, true);
}

void WholeBodyPostureOptimization::updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  contact_estimation_->loadCloud(cloud, true);
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
  auto start_complete = std::chrono::high_resolution_clock::now();
  std::vector<PostureOptimizationResult> results;
  PostureOptimizationResult last_result;
//  ros::Rate rate(5);
  bool first = true;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    auto start = std::chrono::high_resolution_clock::now();
    if (first) {
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), getDefaultJointPositions());
      first = false;
    } else {
//      last_result = findOptimalPosture(pose, last_result.result_joint_angles, last_result.result_joint_angles, &last_result.result_contact);
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), last_result.result_joint_angles, &last_result.result_contact);
    }
    results.push_back(last_result);
    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
    ROS_INFO_STREAM("Complete optimization took " << elapsed.count()*1000.0 << " ms.");
//    rate.sleep();
  }
  std::chrono::duration<double> elapsed_complete = std::chrono::high_resolution_clock::now() - start_complete;
  ROS_INFO_STREAM("Path planning took " << elapsed_complete.count() << " s.");

  return results;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_angles,
                                                                           const std::vector<double> &previous_joint_angles,
                                                                           const sdf_contact_estimation::ContactEstimationResult* init_contact_result)
{
  Eigen::Vector3d rpy = sdf_contact_estimation::rotToNormalizedRpy(pose.linear());
  Eigen::Vector3d xyz = pose.translation();
  ROS_INFO_STREAM("**********************************************");
  ROS_INFO("Requested pose: [%f, %f, %f, %f, %f, %f]", xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));
  ROS_INFO_STREAM("**********************************************");

  PostureOptimizationResult result;
  result.init_joint_angles = init_joint_angles;

  // TODO we should be able to just take this from the previous time step. com shouldn't have changed
  robot_model_->computeCenterOfMass(init_joint_angles, result.init_com);
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
  base_to_com.translation() = result.init_com;

  sdf_contact_estimation::ContactEstimationResult updated_init_contact;
  if (init_contact_result != NULL) {
    result.init_contact = *init_contact_result;
    updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_angles, pose, base_to_com, result.init_contact);
  } else {
    updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_angles, pose, base_to_com);
  }

  if (updated_init_contact.convex_hull_points.empty()) {
    ROS_ERROR_STREAM("Contact estimation failed.");
    result.result_joint_angles = init_joint_angles;
    return result;
  }

  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(updated_init_contact.estimated_pose, base_to_com, init_joint_angles, previous_joint_angles,
                                                                             updated_init_contact);
  result.result_contact = flipper_result.contact_result;
  result.result_joint_angles = flipper_result.joint_state;



  ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points,
                                                                 flipper_result.contact_result.estimated_pose, previous_joint_angles);
//  ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points,
//                                                                 flipper_result.contact_result.estimated_pose, getDefaultJointPositions());
//  ArmOptimizationResult arm_result = arm_optimization_->optimize(init_joint_state, updated_init_contact.convex_hull_points, updated_init_contact.estimated_pose);
  result.result_joint_angles = arm_result.joint_angles;
  result.result_com = arm_result.center_of_mass;

  robot_model_->updateRobotState(result.result_contact.estimated_pose, result.result_joint_angles);
  robot_model_->publishRobotState();
  robot_model_->publishCollisionSpheres();

  return result;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d &pose, const std::vector<double> &init_joint_state, const std::vector<double> &previous_joint_angles)
{
  return findOptimalPosture(pose, init_joint_state, previous_joint_angles, NULL);
}

void WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result)
{
  std::string group = "complete_robot_group"; //TODO parameter
  std::string group_with_world = "complete_robot_group_with_world";
  const robot_state::JointModelGroup* joint_model_group = robot_model_->getMoveitRobotModel()->getJointModelGroup(group);

  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());

  pluginlib::ClassLoader<planning_interface::PlannerManager> planner_plugin_loader("moveit_core", "planning_interface::PlannerManager");
  std::vector<std::string> planner_names = planner_plugin_loader.getDeclaredClasses();
  std::string default_planner_name = planner_names[0];
  planning_interface::PlannerManagerPtr planner_instance(planner_plugin_loader.createUnmanagedInstance(default_planner_name));
  if (!planner_instance->initialize(robot_model_->getMoveitRobotModel(), nh_.getNamespace())) {
    ROS_FATAL_STREAM("Could not initialize planner instance");
    return;
  }
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");


  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = group;

  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  robot_trajectory::RobotTrajectory trajectory(robot_model_->getMoveitRobotModel(), group_with_world);

  for (unsigned int i = 0; i < opt_result.size()-1; i++) {
    // set start state
    robot_model_->updateRobotState(opt_result[i].result_contact.estimated_pose, opt_result[i].result_joint_angles);
    robot_state::RobotState start_state = robot_model_->getMoveitRobotState();
    planning_scene->setCurrentState(start_state);

    // set goal state
    robot_model_->updateRobotState(opt_result[i+1].result_contact.estimated_pose, opt_result[i+1].result_joint_angles);
    robot_state::RobotState goal_state = robot_model_->getMoveitRobotState();
    ROS_INFO_STREAM("Setting constraints");
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(goal_constraint);

    // compute trajectory from start to goal
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    if (!context) {
      ROS_ERROR_STREAM("Failed to construct planning context. Error: " << res.error_code_);
      return;
    }
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return ;
    }
    // append to trajectory
    trajectory.append(*res.trajectory_, 0.0);
  }
  time_parameterization.computeTimeStamps(trajectory);
  publishTrajectory(trajectory);
}

void WholeBodyPostureOptimization::publishTrajectory(const robot_trajectory::RobotTrajectory &robot_trajectory)
{
  moveit_msgs::DisplayTrajectory display_msg;
//  display_msg.model_id = "";

//  moveit_msgs::RobotState start_state_msg;
//  moveit::core::robotStateToRobotStateMsg(robot_trajectory.getFirstWayPoint(), start_state_msg);
//  display_msg.trajectory_start = start_state_msg;

  moveit_msgs::RobotTrajectory trajectory_msg;
  robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);
  display_msg.trajectory.push_back(trajectory_msg);

  trajectory_pub_.publish(display_msg);
}

}
