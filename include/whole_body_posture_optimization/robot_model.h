#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__ROBOT_MODEL_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__ROBOT_MODEL_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#undef LOG // of course everyone names their logging macro the same
#include <differentiation/rbdl_utilsAD.h>
#include <urdfreader/urdfreader.h>

namespace rbdl = RigidBodyDynamics;

namespace whole_body_posture_optimization {

class RobotModel {
public:
  RobotModel(const std::string& robot_description);
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com, Eigen::MatrixXd &jacobian);
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com);

  std::vector<unsigned int> getBodyIndices(const std::vector<std::string>& bodies);
  unsigned int getNumJoints() const;

private:
  std::shared_ptr<rbdl::Model> model_ptr_;
  std::shared_ptr<ADModel> ad_model_ptr_;
  unsigned int num_joints;
};

}

#endif
