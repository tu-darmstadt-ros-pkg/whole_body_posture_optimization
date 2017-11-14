#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H

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
  StabilityError(const std::shared_ptr<CenterOfMass> &com_calculation);
  template <typename T> bool operator()(const T* const parameters, T* cost) const {

  }

  static ceres::CostFunction* Create(const std::shared_ptr<CenterOfMass> &com_calculation);

private:
  template <typename T> T computeForceAngleStability(const std::vector<Vector3T<T>>& contact_points) {

  }

  std::shared_ptr<CenterOfMass> com_calculation_;
};

}

#endif
