#ifndef CENTER_OF_MASS_H
#define CENTER_OF_MASS_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <differentiation/rbdl_utilsAD.h>
#include <urdfreader/urdfreader.h>

namespace rbdl = RigidBodyDynamics;

namespace whole_body_posture_optimization {

class CenterOfMass {
public:
  CenterOfMass(const std::string& robot_description);
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com, Eigen::MatrixXd &jacobian);
private:
  std::shared_ptr<rbdl::Model> model_ptr_;
  std::shared_ptr<ADModel> ad_model_ptr_;
};

}

#endif
