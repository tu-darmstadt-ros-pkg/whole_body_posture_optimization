#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_COMMON_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_COMMON_H

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <moveit/collision_distance_field/collision_distance_field_types.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <ceres_nlopt_wrapper/utils.h>
#include <ceres/cost_function.h>
#include <sdf_contact_estimation/util/utils.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include "types.h"

namespace whole_body_posture_optimization {

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

std::string poseToString(const Eigen::Isometry3d& pose);

template <typename T> T lorentzian(const T& x, const T& sigma) {
  return log(T(1) + (x*x)/(T(2) * sigma * sigma));
}

/// Mean of values weighted by exp(value / temperature). The maximum for temperature 0, the mean for an infinite one.
template <typename T> T softMax(const std::vector<T>& values, const T& temperature) {
  if (values.empty()) {
    return std::numeric_limits<T>::quiet_NaN();
  }
  const T max = *std::max_element(values.begin(), values.end());
  if (temperature <= T(0)) {
    return max;
  }
  T weighted_sum = 0;
  T weight_sum = 0;
  for (const T& value : values) {
    const T weight = std::exp((value - max) / temperature);
    weighted_sum += weight * value;
    weight_sum += weight;
  }
  return weighted_sum / weight_sum;
}

template <typename T> T gaussianDistribution(T x, T mean, T s) {
  T e2 = (x - mean) / s;
  T frac = (s * std::sqrt(T(2 * M_PI)));
  return  std::exp(T(-0.5) * e2 * e2) / frac;
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
  stabilities.reserve(contact_points.size());
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

void updateMoveitRobotState(moveit::core::RobotState& state, const Eigen::Isometry3d& pose);

std::vector<double> getVariablePositions(const moveit::core::RobotStateConstPtr& state, const std::vector<std::string>& variables);

std::string robotTrajectoryToString(const robot_trajectory::RobotTrajectory& trajectory);
std::string robotTrajectoryToOdomString(const robot_trajectory::RobotTrajectory& trajectory);

double computeTrajectoryDuration(const std::vector<robot_trajectory::RobotTrajectory>& trajectories);

std::string line(int length);

std::string getTimestamp();

bool checkGradient(const ceres::CostFunction* cost_function);

//! Computes an index mapping from subset index to list index
//! Returns a list with the same size as subset
template <typename T>
std::vector<std::size_t> getIndexMap(const std::vector<T>& list, const std::vector<T>& subset)
{
  std::vector<std::size_t> indices;
  for (const auto& s: subset) {
    auto it = std::find(list.begin(), list.end(), s);
    std::size_t idx;
    if (it != list.end()) {
      idx  = static_cast<std::size_t>(it - list.begin());
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_posture_optimization"), "Could not find object " << s << " in list");
      idx = 0;
    }
    indices.push_back(idx);
  }
  return indices;
}

template <typename T>
std::vector<T> mapListToSubset(const std::vector<T> &list, const std::vector<std::size_t>& subset_idx_to_list_idx)
{
  std::vector<T> subset(subset_idx_to_list_idx.size());
  for (unsigned int i = 0; i < subset.size(); ++i) {
    subset[i] = list[subset_idx_to_list_idx[i]];
  }
  return subset;
}

Eigen::Vector3d quatToEuler(const Eigen::Quaterniond& quaternion, const std::string& convention = "zyx");

std::string rosTimeToString(const rclcpp::Time& time);

/// Height of the highest upward facing surface in column (x, y) between z_top and z_bottom, NaN when there
/// is none, z_top lies in solid or the column reaches unknown space. The underside of a plate faces down
/// and is skipped. Throws std::invalid_argument for an SDF other than an ESDF.
double highestSurface(const sdf_contact_estimation::SdfModel& sdf, double x, double y, double z_top, double z_bottom);

}

#endif
