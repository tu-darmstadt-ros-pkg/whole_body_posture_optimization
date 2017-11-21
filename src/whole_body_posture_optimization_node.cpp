#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>

bool pose_updated_;
Eigen::Affine3d robot_pose_;

void poseCB(const geometry_msgs::PoseStamped& pose_msg) {
  tf::poseMsgToEigen(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
  ros::NodeHandle pnh("~");

  ros::Subscriber robot_pose_sub = pnh.subscribe("set_robot_pose", 10, &poseCB);

  // Create robot shape
  sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape = std::make_shared<argo_tracker::ArgoTrackerShape>();
  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, tracker_shape);

  // create environment
  auto cloud = sdf_contact_estimation::generateStep();
  cloud.header.frame_id = "world";
  po.updateWorldModel(cloud);

  // create path
//  nav_msgs::Path path;
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
  // Compute optimal postures
//  po.findOptimalPostures(path);

  // Try with one pose first  
  robot_pose_ = Eigen::Affine3d::Identity();
  robot_pose_.translation() = Eigen::Vector3d(-0.054, 0, -0.96);
  pose_updated_ = true;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    if (pose_updated_) {
      auto start = std::chrono::high_resolution_clock::now();
      po.findOptimalPosture(robot_pose_, std::vector<double>());
      std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
      ROS_INFO_STREAM("Flipper optimization took " << elapsed.count()*1000.0 << " ms.");
      pose_updated_ = false;
    }
    rate.sleep();
  }

  ros::spin();

  return 0;
}
