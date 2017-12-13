#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__COMMON_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__COMMON_H

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>

namespace whole_body_posture_optimization {

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3 = Eigen::Transform<T, 3, Eigen::Affine>;

template<typename T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template <typename T> T lorentzian(const T& x, const T& sigma) {
  return log(T(1) + pow(x, T(2))/(T(2) * pow(sigma, T(2))));
}

template <typename T> std::vector<T> computeForceAngleStabilities(const std::vector<Vector3<T>>& contact_points, const Vector3<T>& external_force, const Vector3<T>& com) {
  std::vector<T> stabilities;
  for (unsigned int i = 0; i < contact_points.size(); i++) {
    Vector3<T> contact_point_p1 = contact_points[(i+1) % contact_points.size()];

    Vector3<T> axis = contact_point_p1 - contact_points[i]; // a_i
    Matrix3<T> identity = Matrix3<T>::Identity(); // I
    Vector3<T> axis_norm = axis / axis.norm(); // \hat{a}_i
    Matrix3<T> p1 = identity - axis_norm * axis_norm.transpose(); // I - \hat{a}_i * \hat{a}_i^T
    Vector3<T> axis_normal = p1 * (contact_point_p1 - com); // l_i
    Vector3<T> axis_normal_norm = axis_normal / axis_normal.norm();
    Vector3<T> force_component = p1 * external_force; // f_i -> can be precomputed
    Vector3<T> force_component_norm = force_component / force_component.norm(); // \hat{f}_i
    Vector3<T> distance = -axis_normal + (axis_normal.dot(force_component_norm)) * force_component_norm; // d_i


    T t1 = force_component_norm.cross(axis_normal_norm).dot(axis_norm);
    T t2 = axis_normal_norm.dot(force_component_norm);
    T signed_angle = atan2(t1, t2);
//    T angle = acos(axis_normal_norm.dot(force_component_norm));
    T stability = signed_angle * distance.norm() * force_component.norm();
    stabilities.push_back(stability);
  }

  return stabilities;
}

template <typename T> std::vector<T> computeForceAngleStabilitiesWithGravity(const std::vector<Vector3<T>>& contact_points, const Vector3<T>& com) {
  return computeForceAngleStabilities(contact_points, Vector3<T>(T(0), T(0), T(-9.81)), com);
}

void createConvexHullMarkerArray(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& stabilities, visualization_msgs::MarkerArray &marker_array);
}

#endif
