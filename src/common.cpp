#include <whole_body_posture_optimization/common.h>

#include <ctime>

namespace whole_body_posture_optimization {
void createConvexHullMarkerArray(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& stabilities, visualization_msgs::MarkerArray &marker_array)
{
  for (unsigned i = 0; i < points.size(); i++) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.0;
    marker.scale.z = 0.00001;

    // calculate stability color
    double max_stability = 6.0;
    double clamped_stability = std::min(stabilities[i], max_stability);
    double stability_pct = clamped_stability/max_stability;
    marker.color.a = 1.0;
    marker.color.r = 1 - stability_pct;
    marker.color.g = stability_pct;
    marker.color.b = 0;

    marker.header.frame_id = "world";
    marker.ns = "support_polygon";
    marker.id = i;

    Eigen::Affine3d marker_pose = Eigen::Affine3d::Identity();
    tf::poseEigenToMsg(marker_pose, marker.pose);

    geometry_msgs::Point point_start;
    tf::pointEigenToMsg(points[i], point_start);
    marker.points.push_back(point_start);
    int ip1 = (i+1) % points.size(); // wrap around
    geometry_msgs::Point point_end;
    tf::pointEigenToMsg(points[ip1], point_end);
    marker.points.push_back(point_end);
    marker_array.markers.push_back(marker);
  }
}

void updateMoveitRobotState(moveit::core::RobotState &state, const Eigen::Affine3d &pose)
{
  sensor_msgs::JointState virtual_link_joint_states;
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_x");
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_y");
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_z");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_x");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_y");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_z");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_w");

  virtual_link_joint_states.position.resize(7);
  virtual_link_joint_states.position[0] = pose.translation().x();
  virtual_link_joint_states.position[1] = pose.translation().y();
  virtual_link_joint_states.position[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  virtual_link_joint_states.position[3] = quat.x();
  virtual_link_joint_states.position[4] = quat.y();
  virtual_link_joint_states.position[5] = quat.z();
  virtual_link_joint_states.position[6] = quat.w();
  moveit::core::jointStateToRobotState(virtual_link_joint_states, state);
}

std::string line(int length) {
  std::string l = "";
  for (unsigned int i = 0; i < length; i++) {
    l += "-";
  }
  return l;
}

std::string getTimestamp()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

}
