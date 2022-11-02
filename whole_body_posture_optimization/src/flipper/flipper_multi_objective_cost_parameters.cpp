#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost_parameters.h>

namespace whole_body_posture_optimization {

FlipperMultiObjectiveCostParameters::FlipperMultiObjectiveCostParameters()
: OptimizationParametersBase(),
  support_area_weight(0.0),
  movement_cost_weight(0.0),
  lorentzian_sigma(0.0),
  track_contact_weight(0.0),
  track_contact_nominal_value(0.0),
  body_contact_weight(0.0),
  turning_resistance_weight(0.0),
  position_deviation_weight(0.0),
  roll_cost_weight(0.0),
  stability_weight(0.0),
  stability_nominal_value(0.0),
  stability_exp_parameters{0, 0, 0},
  num_offset_steps_x(0.0),
  offset_step_size(0.05)
{}

bool FlipperMultiObjectiveCostParameters::loadParametersFromNamespaceImpl(const ros::NodeHandle& nh) {
  registerParameter(nh, "support_area_weight", support_area_weight, 1.0, "Support area weight", 0.0, 100);
  registerParameter(nh, "movement_cost_weight", movement_cost_weight, 0.2, "Movement cost weight", 0.0, 100);
  registerParameter(nh, "lorentzian_sigma", lorentzian_sigma, 0.2, "Lorentzian sigma", 0.0, 10);
  registerParameter(nh, "track_contact_weight", track_contact_weight, 2.0, "Track contact weight", 0.0, 100);
  registerParameter(nh, "track_contact_nominal_value", track_contact_nominal_value, 10.0, "Track contact nominal value", 0.0, 10);
  registerParameter(nh, "impact_weight", impact_weight, 1.0, "Impact penalty weight", 0.0, 100);
  registerParameter(nh, "turning_resistance_weight", turning_resistance_weight, 1.0, "Turning resistance weight", 0.0, 100);
  registerParameter(nh, "body_contact_weight", body_contact_weight, 4.0, "Body contact weight", 0.0, 10);
  registerParameter(nh, "position_deviation_weight", position_deviation_weight, 10.0, "Position deviation weight", 0.0, 100);
  registerParameter(nh, "roll_cost_weight", roll_cost_weight, 0.0, "Roll cost weight", 0.0, 10);
  registerParameter(nh, "stability_weight", stability_weight, 1.0, "Stability cost weight", 0.0, 100);
  registerParameter(nh, "stability_nominal_value", stability_nominal_value, 0.0, "Stability nominal value", 0.0, 10);
  registerParameter(nh, "stability_exp_parameters_a", stability_exp_parameters[0], 1.6, "Stability EXP parameter (a)", 0.0, 100);
  registerParameter(nh, "stability_exp_parameters_b", stability_exp_parameters[1], -1.7, "Stability EXP parameter (b)", -100.0, 0);
  registerParameter(nh, "stability_exp_parameters_c", stability_exp_parameters[2], 0.0, "Stability EXP parameter (c)", 0.0, 10);
  registerParameter(nh, "num_offset_steps_x", num_offset_steps_x, 0.0, "Number of offset steps in robot x-direction", -10, 10);
  registerParameter(nh, "offset_step_size", offset_step_size, 0.0, "Length of offset steps", -1, 1);
  return true;
}

}
