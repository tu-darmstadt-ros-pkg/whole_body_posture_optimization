#include <whole_body_posture_optimization/stability_error.h>

namespace whole_body_posture_optimization {

StabilityError::StabilityError(const std::string &robot_description)
  : com_calculation_(robot_description)
{

}

ceres::CostFunction* StabilityError::Create(const std::string &robot_description)
{
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<StabilityError, 1, 3>(
        new StabilityError(robot_description));

  return cost_function;
}

}
