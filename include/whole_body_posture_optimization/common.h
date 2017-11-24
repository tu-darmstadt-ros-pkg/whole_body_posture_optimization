#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__COMMON_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__COMMON_H

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>

namespace whole_body_posture_optimization {
void createConvexHullMarkerArray(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& stabilities, visualization_msgs::MarkerArray &marker_array);
}

#endif
