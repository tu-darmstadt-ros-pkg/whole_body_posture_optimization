#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_PARAMETERS_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_PARAMETERS_H

#include <whole_body_posture_optimization/optimization_parameters_base.h>

namespace whole_body_posture_optimization {

class FlipperMultiObjectiveCostParameters : public OptimizationParametersBase {
public:
  FlipperMultiObjectiveCostParameters();
  double support_area_weight;
  double movement_cost_weight;
  double lorentzian_sigma;
  double track_contact_weight;
  double track_contact_nominal_value;
  double body_contact_weight;
  double turning_resistance_weight;
  double position_deviation_weight;
  double roll_cost_weight;
  double stability_weight;
  double stability_nominal_value;
  double stability_exp_parameters[3];
  double impact_weight;
  double num_offset_steps_x;
  double offset_step_size;
protected:
  bool loadParametersFromNamespaceImpl(const ros::NodeHandle& nh) override;
private:
};

}



#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_PARAMETERS_H
