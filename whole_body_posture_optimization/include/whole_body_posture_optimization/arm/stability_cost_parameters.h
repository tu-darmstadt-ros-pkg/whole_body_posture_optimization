#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_STABILITY_COST_PARAMETERS_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_STABILITY_COST_PARAMETERS_H

#include <whole_body_posture_optimization/optimization_parameters_base.h>

namespace whole_body_posture_optimization {

class StabilityCostParameters : public OptimizationParametersBase {
public:
  StabilityCostParameters();

  double exp_parameters[3];
  double movement_cost_weight;
  double lorentzian_sigma;
  double function_scale;
protected:
  bool loadParametersFromNamespaceImpl(const rclcpp::Node::SharedPtr& node) override;
private:
};

}



#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_STABILITY_COST_PARAMETERS_H
