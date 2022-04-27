#include <whole_body_planning_server/whole_body_planning_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "driving_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  whole_body_planning_server::WholeBodyPlanningServer driving_controller(nh, pnh);
  ROS_INFO_STREAM("Finished initializing driving controller");

  std::string tsdf_load_path;
  pnh.param<std::string>("tsdf_load_path", tsdf_load_path, "");
  if (tsdf_load_path.empty()) {
    driving_controller.loadTsdfFromPath(tsdf_load_path);
  }

  // Debug
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";  
  double path_start, path_stop, path_step;
  pnh.param("path_start", path_start, 0.0);
  pnh.param("path_stop", path_stop, 1.5);
  pnh.param("path_step", path_step, 0.02);
  for (double x = path_start; x < path_stop; x += path_step) {
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
