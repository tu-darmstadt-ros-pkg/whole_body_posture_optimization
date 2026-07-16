#ifndef WHOLE_BODY_PLANNING_SERVER_UTIL_H
#define WHOLE_BODY_PLANNING_SERVER_UTIL_H

#include <nav_msgs/msg/path.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace whole_body_planning_server {

nav_msgs::msg::Path slicePath(const nav_msgs::msg::Path& path, double distance);
nav_msgs::msg::Path slicePath(const nav_msgs::msg::Path& path, int start_index, int end_index);
nav_msgs::msg::Path slicePathDirectDistance(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Vector3& p0, double distance_limit, bool invert=false, int search_start_index=0);
template <typename Vector1, typename Vector2>
double euclideanDistance2D(const Vector1& p1, const Vector2& p2) {
  double x = p2.x - p1.x;
  double y = p2.y - p1.y;
  return std::sqrt(x * x + y * y);
}
void appendPath(nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2);

void enforceAccelerationBounds(const moveit::core::RobotStatePtr& robot_state_ptr);

std::string pathToString(const nav_msgs::msg::Path& path);
}

#endif  // WHOLE_BODY_PLANNING_SERVER_UTIL_H
