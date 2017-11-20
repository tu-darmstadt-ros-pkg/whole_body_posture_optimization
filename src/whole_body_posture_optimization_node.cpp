#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
  ros::NodeHandle nh;
  ros::NodeHandle param_nh("~");

  // Create robot shape
  sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape = std::make_shared<argo_tracker::ArgoTrackerShape>();
  whole_body_posture_optimization::WholeBodyPostureOptimization po(param_nh, tracker_shape);

  // create environment
  auto cloud = sdf_contact_estimation::generateStep();
  po.updateWorldModel(cloud);

  // create path
  nav_msgs::Path path;
//  double step = 0.1;
//  for (unsigned double x = -0.5; x < 0.5; x+=step) {
//    geometry_msgs::PoseStamped pose;
//    pose.header.frame_id = "world";

//    pose.pose.position.x = x;
//    pose.pose.position.y = 0;
//    pose.pose.position.z = -0.9;
//    pose.pose.orientation.w = 1.0;

//    path.poses.push_back(pose);
//  }
  // Try with one pose first
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";

  pose.pose.position.x = -0.054;
  pose.pose.position.y = 0;
  pose.pose.position.z = -0.96;
  pose.pose.orientation.w = 1.0;

  path.poses.push_back(pose);

  // Compute optimal postures
  po.findOptimalPostures(path);

  ros::spin();

  return 0;
}
