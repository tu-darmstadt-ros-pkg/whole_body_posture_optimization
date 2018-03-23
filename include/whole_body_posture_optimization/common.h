#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__COMMON_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__COMMON_H

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/collision_distance_field/collision_distance_field_types.h>

#include <ceres_nlopt_wrapper/utils.h>

namespace whole_body_posture_optimization {

enum CollisionType {
  ENVIRONMENT,
  SELF_COLLISION,
  SELF_COLLISION_TSDF
};

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3 = Eigen::Transform<T, 3, Eigen::Affine>;

template<typename T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template<typename T>
std::string vecToString(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

template <typename T> T lorentzian(const T& x, const T& sigma) {
  return log(T(1) + pow(x, T(2))/(T(2) * pow(sigma, T(2))));
}

struct CollisionObject {
  CollisionObject(const collision_detection::CollisionSphere& _collision_sphere, std::string _link_name, double _padding)
    : collision_sphere(_collision_sphere), link_name(_link_name), padding(_padding) {}

  collision_detection::CollisionSphere collision_sphere;
  std::string link_name;
  double padding;
};

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

void updateMoveitRobotState(robot_state::RobotState& state, const Eigen::Affine3d& pose);
//void updateState(robot_state::RobotState &state, const std::vector<double>& angles);

std::string line(int length);

std::string getTimestamp();

}

#endif
