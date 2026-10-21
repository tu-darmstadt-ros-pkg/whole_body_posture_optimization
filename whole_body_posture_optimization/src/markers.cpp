#include <whole_body_posture_optimization/markers.h>

#include <algorithm>

#include <tf2_eigen/tf2_eigen.hpp>

#include <whole_body_posture_optimization/util/common.h>

namespace whole_body_posture_optimization {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

// Same scale as the support polygon colours of hector_pose_prediction_ros
constexpr double kMaxStability = 6.0;

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA stabilityColor(double stability)
{
  const auto t = static_cast<float>(std::clamp(stability / kMaxStability, 0.0, 1.0));
  return rgba(1.0f - t, t, 0.0f, 1.0f);
}

Marker makeMarker(const std::string& frame_id, const std::string& ns, int id, int32_t type)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

}

MarkerArray clearMarkers(const std::string& frame_id)
{
  MarkerArray array;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.action = Marker::DELETEALL;
  array.markers.push_back(marker);
  return array;
}

MarkerArray horizonMarkers(const nav_msgs::msg::Path& path, const std::vector<PostureOptimizationResult>& results,
                           const sdf_contact_estimation::SurfaceBand& terrain, const std::string& frame_id)
{
  MarkerArray array = clearMarkers(frame_id);
  const std_msgs::msg::ColorRGBA red = rgba(1.0f, 0.0f, 0.0f, 1.0f);

  // rviz rejects an empty triangle list
  if (!terrain.triangles.empty()) {
    Marker surface = makeMarker(frame_id, "terrain_surface", 0, Marker::TRIANGLE_LIST);
    surface.scale.x = surface.scale.y = surface.scale.z = 1.0;
    surface.color = rgba(0.2f, 0.4f, 1.0f, 0.35f);
    for (const Eigen::Vector3d& p : terrain.triangles) {
      surface.points.push_back(tf2::toMsg(p));
    }
    array.markers.push_back(surface);
  }
  if (!terrain.no_surface.empty()) {
    Marker none = makeMarker(frame_id, "terrain_no_surface", 0, Marker::POINTS);
    none.scale.x = none.scale.y = 0.02;
    none.color = red;
    for (const Eigen::Vector3d& p : terrain.no_surface) {
      none.points.push_back(tf2::toMsg(p));
    }
    array.markers.push_back(none);
  }

  Marker reference = makeMarker(frame_id, "reference_path", 0, Marker::LINE_STRIP);
  reference.scale.x = 0.02;
  reference.color = rgba(0.5f, 0.5f, 0.5f, 0.8f);
  for (const auto& pose : path.poses) {
    reference.points.push_back(pose.pose.position);
  }
  if (reference.points.size() > 1) {
    array.markers.push_back(reference);
  }

  Marker planned = makeMarker(frame_id, "planned_path", 0, Marker::LINE_STRIP);
  planned.scale.x = 0.03;
  const std_msgs::msg::ColorRGBA orange = rgba(1.0f, 0.55f, 0.0f, 1.0f);
  planned.color = orange;

  const std::size_t ghost_stride = (results.size() + kMaxGhosts - 1) / kMaxGhosts;
  for (std::size_t k = 0; k < results.size(); ++k) {
    const PostureOptimizationResult& result = results[k];
    const int id = static_cast<int>(k);

    planned.points.push_back(tf2::toMsg(Eigen::Vector3d(result.result_pose.translation())));
    planned.colors.push_back(result.success ? orange : red);

    // The last waypoint always gets a ghost because it is the newest or the failed one
    if (k % ghost_stride == 0 || k + 1 == results.size()) {
      const std_msgs::msg::ColorRGBA ghost_color = result.success ? rgba(0.0f, 1.0f, 0.0f, 0.4f) : rgba(1.0f, 0.0f, 0.0f, 0.4f);
      moveit::core::RobotState state(*result.result_state);
      MarkerArray ghost;
      state.getRobotMarkers(ghost, state.getRobotModel()->getLinkModelNames(), ghost_color, "ghost_" + std::to_string(k),
                            rclcpp::Duration(0, 0));
      for (Marker& marker : ghost.markers) {
        marker.header.frame_id = frame_id;
        array.markers.push_back(marker);
      }
    }

    if (!result.success) {
      Marker failed = makeMarker(frame_id, "failed_pose", id, Marker::ARROW);
      failed.pose = tf2::toMsg(result.requested_pose);
      failed.scale.x = 0.4;
      failed.scale.y = failed.scale.z = 0.08;
      failed.color = red;
      array.markers.push_back(failed);
    }

    // The arm optimization computes the COM only for a polygon
    const auto& hull = result.support_polygon.contact_hull_points;
    if (hull.size() > 2) {
      const Eigen::Vector3d com = result.result_pose * result.result_com;
      const std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(hull, com);

      // A line list colours each edge on its own, a strip would blend neighbouring edges
      Marker polygon = makeMarker(frame_id, "support_polygon", id, Marker::LINE_LIST);
      polygon.scale.x = 0.02;
      polygon.color = rgba(1.0f, 1.0f, 1.0f, 1.0f);
      double ground_z = 0.0;
      for (std::size_t i = 0; i < hull.size(); ++i) {
        polygon.points.push_back(tf2::toMsg(Eigen::Vector3d(hull[i])));
        polygon.points.push_back(tf2::toMsg(Eigen::Vector3d(hull[(i + 1) % hull.size()])));
        polygon.colors.insert(polygon.colors.end(), 2, stabilityColor(stabilities[i]));
        ground_z += hull[i].z() / static_cast<double>(hull.size());
      }
      array.markers.push_back(polygon);

      Marker com_sphere = makeMarker(frame_id, "com", 2 * id, Marker::SPHERE);
      com_sphere.pose.position = tf2::toMsg(com);
      com_sphere.scale.x = com_sphere.scale.y = com_sphere.scale.z = 0.08;
      com_sphere.color = rgba(1.0f, 0.0f, 1.0f, 1.0f);
      array.markers.push_back(com_sphere);

      Marker com_line = makeMarker(frame_id, "com", 2 * id + 1, Marker::LINE_LIST);
      com_line.scale.x = 0.01;
      com_line.color = com_sphere.color;
      com_line.points.push_back(tf2::toMsg(com));
      com_line.points.push_back(tf2::toMsg(Eigen::Vector3d(com.x(), com.y(), ground_z)));
      array.markers.push_back(com_line);
    }

    Marker contacts = makeMarker(frame_id, "contacts", id, Marker::SPHERE_LIST);
    contacts.scale.x = contacts.scale.y = contacts.scale.z = 0.04;
    contacts.color = rgba(1.0f, 1.0f, 0.0f, 0.9f);
    for (const auto& contact : result.contact_information.contact_points) {
      contacts.points.push_back(tf2::toMsg(Eigen::Vector3d(contact.point)));
    }
    if (!contacts.points.empty()) {
      array.markers.push_back(contacts);
    }
  }

  if (planned.points.size() > 1) {
    array.markers.push_back(planned);
  }
  return array;
}

std::vector<Marker> statusMarkers(const Eigen::Vector3d& robot_position, const std::string& reason, const std::string& frame_id)
{
  const std_msgs::msg::ColorRGBA red = rgba(0.9f, 0.1f, 0.1f, 0.9f);
  Marker sphere = makeMarker(frame_id, "status", 0, Marker::SPHERE);
  sphere.pose.position = tf2::toMsg(Eigen::Vector3d(robot_position + Eigen::Vector3d(0.0, 0.0, 0.8)));
  sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15;
  sphere.color = red;
  Marker label = makeMarker(frame_id, "status", 1, Marker::TEXT_VIEW_FACING);
  label.pose.position = tf2::toMsg(Eigen::Vector3d(robot_position + Eigen::Vector3d(0.0, 0.0, 1.0)));
  label.scale.z = 0.1;
  label.color = red;
  label.text = reason;
  return {sphere, label};
}

MarkerArray goalEndMarkers(const std::string& status, const Eigen::Vector3d& robot_position, const std::string& frame_id)
{
  MarkerArray array = clearMarkers(frame_id);
  if (!status.empty()) {
    const auto status_markers = statusMarkers(robot_position, status, frame_id);
    array.markers.insert(array.markers.end(), status_markers.begin(), status_markers.end());
  }
  return array;
}

std::string failureReason(const PostureOptimizationResult& result)
{
  if (!result.failure_reason.empty()) {
    return result.failure_reason;
  }
  return result.support_polygon.contact_hull_points.size() > 2 ? "unstable" : "no posture";
}

}
