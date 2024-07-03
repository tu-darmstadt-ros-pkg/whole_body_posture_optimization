#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_TYPES_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_TYPES_H

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

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_TYPES_H
