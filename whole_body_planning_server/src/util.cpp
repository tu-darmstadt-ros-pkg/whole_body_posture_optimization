#include <whole_body_planning_server/util.h>

namespace whole_body_planning_server {

nav_msgs::Path slicePath(const nav_msgs::Path& path, double distance)
{
  // Special case: always return original path if distance is <=0
  if (distance <= 0) {
    return path;
  }
  // Sum up distance and add poses
  double current_distance = 0.0;
  nav_msgs::Path path_slice;
  path_slice.header = path.header;
  for (const auto& pose: path.poses) {
    if (path_slice.poses.empty()) {
      path_slice.poses.push_back(pose);
    } else {
      double d = euclideanDistance2D(path_slice.poses.back().pose.position, pose.pose.position);
      current_distance += d;

      if (current_distance > distance) {
        break;
      }
      path_slice.poses.push_back(pose);
    }
  }
  return path_slice;
}

nav_msgs::Path slicePath(const nav_msgs::Path& path, int start_index, int end_index)
{
  nav_msgs::Path path_slice;
  path_slice.header = path.header;

  if (start_index >= path.poses.size() || end_index <= start_index) {
    return path_slice;
  }

  if (end_index > path.poses.size()) {
    end_index = path.poses.size();
  }

  if (start_index < 0) {
    start_index = 0;
  }

  path_slice.poses.assign(path.poses.begin() + start_index, path.poses.begin() + end_index);
  return path_slice;
}

nav_msgs::Path slicePathDirectDistance(const nav_msgs::Path& path, const geometry_msgs::Vector3& p0,
                                                                double distance_limit, bool invert, int search_start_index)
{
  int i;
  for (i = search_start_index; i < path.poses.size(); ++i) {
    // Distance in x-y plane
    double distance = euclideanDistance2D(p0, path.poses[i].pose.position);
    if (distance > distance_limit) break;
  }
  nav_msgs::Path path_slice;
  if (!invert) {
    path_slice = slicePath(path, 0, i);
  } else {
    path_slice = slicePath(path, i, path.poses.size());

  }
  return path_slice;
}

void enforceAccelerationBounds(const moveit::core::RobotStatePtr& robot_state_ptr)
{
  const std::vector<std::string>& variable_names = robot_state_ptr->getVariableNames();
  const moveit::core::RobotModelConstPtr& robot_model = robot_state_ptr->getRobotModel();
  for (int j = 0; j < robot_state_ptr->getVariableCount(); ++j) {
    const moveit::core::VariableBounds& bounds = robot_model->getVariableBounds(variable_names[j]);
    if (bounds.acceleration_bounded_) {
      double acceleration = robot_state_ptr->getVariableAcceleration(j);
      if (acceleration > bounds.max_acceleration_) {
        robot_state_ptr->setVariableAcceleration(j, bounds.max_acceleration_);
      } else if (acceleration < bounds.min_acceleration_) {
        robot_state_ptr->setVariableAcceleration(j, bounds.min_acceleration_);
      }
    }
  }
}

void appendPath(nav_msgs::Path& path1, const nav_msgs::Path& path2) {
  path1.poses.insert(path1.poses.end(), path2.poses.begin(), path2.poses.end());
}

double euclideanDistance2D(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  double x = p2.x - p1.x;
  double y = p2.y - p1.y;
  return std::sqrt(x * x + y * y);
}

std::string pathToString(const nav_msgs::Path& path)
{
  std::stringstream ss;
  for (unsigned int i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::Point& point = path.poses[i].pose.position;
    ss << std::endl << i << ": [" << point.x << ", " << point.y << ", " << point.z << "]";
  }
  return ss.str();
}

}
