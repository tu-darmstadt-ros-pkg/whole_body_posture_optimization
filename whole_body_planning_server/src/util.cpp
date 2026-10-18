#include <algorithm>
#include <iterator>

#include <tf2_eigen/tf2_eigen.hpp>
#include <whole_body_planning_server/util.h>

namespace whole_body_planning_server {

nav_msgs::msg::Path slicePath(const nav_msgs::msg::Path& path, double distance)
{
  // Special case: always return original path if distance is <=0
  if (distance <= 0) {
    return path;
  }
  // Sum up distance and add poses
  double current_distance = 0.0;
  nav_msgs::msg::Path path_slice;
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

nav_msgs::msg::Path slicePath(const nav_msgs::msg::Path& path, int start_index, int end_index)
{
  nav_msgs::msg::Path path_slice;
  path_slice.header = path.header;

  if (start_index >= static_cast<int>(path.poses.size()) || end_index <= start_index) {
    return path_slice;
  }

  if (end_index > static_cast<int>(path.poses.size())) {
    end_index = path.poses.size();
  }

  if (start_index < 0) {
    start_index = 0;
  }

  path_slice.poses.assign(path.poses.begin() + start_index, path.poses.begin() + end_index);
  return path_slice;
}

nav_msgs::msg::Path slicePathDirectDistance(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Vector3& p0,
                                                                double distance_limit, bool invert, int search_start_index)
{
  int i;
  for (i = search_start_index; i < static_cast<int>(path.poses.size()); ++i) {
    // Distance in x-y plane
    double distance = euclideanDistance2D(p0, path.poses[i].pose.position);
    if (distance > distance_limit) break;
  }
  nav_msgs::msg::Path path_slice;
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
  for (int j = 0; j < static_cast<int>(robot_state_ptr->getVariableCount()); ++j) {
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

void appendPath(nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2) {
  path1.poses.insert(path1.poses.end(), path2.poses.begin(), path2.poses.end());
}

rclcpp::Time startTimeAfterWait(const robot_trajectory::RobotTrajectory& trajectory, size_t waited_index, const rclcpp::Time& now) {
  return now - rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(waited_index));
}

void dropPosesNearStart(nav_msgs::msg::Path& path, double distance) {
  if (path.poses.size() < 3) {
    return;
  }
  const geometry_msgs::msg::Point start = path.poses.front().pose.position;
  auto first_distant = std::find_if(std::next(path.poses.begin()), std::prev(path.poses.end()),
                                    [&](const geometry_msgs::msg::PoseStamped& pose) {
    return euclideanDistance2D(start, pose.pose.position) >= distance;
  });
  path.poses.erase(std::next(path.poses.begin()), first_distant);
}

double euclideanDistance2D(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
  double x = p2.x - p1.x;
  double y = p2.y - p1.y;
  return std::sqrt(x * x + y * y);
}

std::string pathToString(const nav_msgs::msg::Path& path)
{
  std::stringstream ss;
  for (unsigned int i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::msg::Point& point = path.poses[i].pose.position;
    ss << std::endl << i << ": [" << point.x << ", " << point.y << ", " << point.z << "]";
  }
  return ss.str();
}

nav_msgs::msg::Path timedPath(const std::vector<Eigen::Isometry3d>& poses, const std::vector<double>& times,
                              const rclcpp::Time& start_time, const std::string& frame_id)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = start_time;
  Eigen::Isometry3d previous_pose;
  double previous_time = 0.0;
  for (size_t i = 0; i < poses.size(); ++i) {
    const Eigen::Isometry3d& pose = poses[i];
    if (i > 0) {
      if (times[i] <= previous_time) {
        continue;
      }
      double distance = (pose.translation().head<2>() - previous_pose.translation().head<2>()).norm();
      Eigen::Vector2d heading = pose.linear().col(0).head<2>();
      Eigen::Vector2d previous_heading = previous_pose.linear().col(0).head<2>();
      double turn = std::abs(std::atan2(previous_heading.x() * heading.y() - previous_heading.y() * heading.x(), previous_heading.dot(heading)));
      if (distance < 0.01 && turn < 0.02) {
        if (i + 1 < poses.size()) {
          continue;
        }
        if (path.poses.size() > 1) {
          path.poses.pop_back();
        }
      }
    }
    previous_pose = pose;
    previous_time = times[i];
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame_id;
    pose_msg.header.stamp = start_time + rclcpp::Duration::from_seconds(times[i]);
    pose_msg.pose = tf2::toMsg(pose);
    path.poses.push_back(pose_msg);
  }
  return path;
}

}
