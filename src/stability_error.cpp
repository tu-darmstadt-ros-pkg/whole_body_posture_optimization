#include <whole_body_posture_optimization/stability_error.h>

namespace whole_body_posture_optimization {

StabilityError::StabilityError(const std::shared_ptr<CenterOfMass>& com_calculation)
  : com_calculation_(com_calculation)
{

}

ceres::CostFunction* StabilityError::Create(const std::shared_ptr<CenterOfMass>& com_calculation)
{
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<StabilityError, 1, 3>(
        new StabilityError(com_calculation));

  return cost_function;
}

}
