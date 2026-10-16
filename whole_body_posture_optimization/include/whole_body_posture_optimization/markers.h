#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_MARKERS_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_MARKERS_H

#include <cstddef>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <sdf_contact_estimation/sdf/terrain_surface.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

/// Robot meshes per message are bounded, longer horizons get every n-th waypoint as a ghost
constexpr std::size_t kMaxGhosts = 20;

/// A single DELETEALL marker
visualization_msgs::msg::MarkerArray clearMarkers(const std::string& frame_id);

/// Markers of optimized waypoints, starting with DELETEALL. results holds only optimized waypoints, path the
/// reference poses, terrain the planner's terrain surface. Namespaces ghost_<k> (results[k]), planned_path,
/// reference_path, support_polygon, contacts, com, failed_pose, terrain_surface and terrain_no_surface.
visualization_msgs::msg::MarkerArray horizonMarkers(const nav_msgs::msg::Path& path,
                                                    const std::vector<PostureOptimizationResult>& results,
                                                    const sdf_contact_estimation::SurfaceBand& terrain,
                                                    const std::string& frame_id);

/// Namespace status: a red sphere above robot_position and a label of at most two words naming why the planner holds
std::vector<visualization_msgs::msg::Marker> statusMarkers(const Eigen::Vector3d& robot_position, const std::string& reason,
                                                           const std::string& frame_id);

/// Markers of a goal end: a DELETEALL, then statusMarkers when status names why the goal was aborted
visualization_msgs::msg::MarkerArray goalEndMarkers(const std::string& status, const Eigen::Vector3d& robot_position,
                                                    const std::string& frame_id);

/// Status label of a failed waypoint, "unstable" when a support polygon exists, "no posture" otherwise
std::string failureReason(const PostureOptimizationResult& result);

}

#endif
