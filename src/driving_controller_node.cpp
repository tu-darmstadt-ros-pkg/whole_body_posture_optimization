#include <whole_body_posture_optimization/driving_controller.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "driving_controller_node");
  ros::NodeHandle pnh("~");

  whole_body_posture_optimization::DrivingController driving_controller(pnh);
  ROS_INFO_STREAM("Finished initializing driving controller");

  std::string tsdf_load_path;
  pnh.param<std::string>("tsdf_load_path", tsdf_load_path, "");
  if (tsdf_load_path != "") {
    driving_controller.loadTsdfFromPath(tsdf_load_path);
  }

  // Debug
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  double step = 0.02;
  for (double x = 0.0; x < 2; x+=step) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(x, 0, 0);
    tf::poseEigenToMsg(pose, pose_msg.pose);

    path.poses.push_back(pose_msg);
  }

  ros::Publisher path_pub = pnh.advertise<nav_msgs::Path>("path_command", 10, true);
  ros::Duration(1).sleep();
  ros::spinOnce();
  path_pub.publish(path);

  ros::spin();

  return 0;
}
