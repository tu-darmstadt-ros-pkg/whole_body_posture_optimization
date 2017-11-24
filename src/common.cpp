#include <whole_body_posture_optimization/common.h>

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

}
