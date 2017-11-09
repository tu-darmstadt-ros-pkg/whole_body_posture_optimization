#ifndef STABILITY_ERROR
#define STABILITY_ERROR

#include <whole_body_posture_optimization/center_of_mass.h>
#include <ceres/ceres.h>

#include <Eigen/Eigen>

namespace  whole_body_posture_optimization {

template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3T = Eigen::Transform<T, 3, Eigen::Affine>;


class StabilityError {
public:
  StabilityError(const std::string& robot_description);
  template <typename T> bool operator()(const T* const parameters, T* cost) const {

  }

  static ceres::CostFunction* Create(const std::string& robot_description);

private:
  template <typename T> T computeForceAngleStability(const std::vector<Vector3T<T>>& contact_points) {

  }

  CenterOfMass com_calculation_;
};

}

#endif
