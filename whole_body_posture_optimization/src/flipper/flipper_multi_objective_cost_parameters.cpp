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
  stability_exp_parameters{0, 0, 0}
{}

bool FlipperMultiObjectiveCostParameters::loadParametersFromNamespaceImpl(const ros::NodeHandle& nh) {
  //  nh.param("support_area_weight", parameters_.support_area_weight, 1.0);
  registerParameter(nh, "support_area_weight", support_area_weight, 1.0, "Support area weight", 0.0, 10);
  //    nh.param("movement_cost_weight", parameters_.movement_cost_weight, 0.2);
  registerParameter(nh, "movement_cost_weight", movement_cost_weight, 0.2, "Movement cost weight", 0.0, 10);
  //    nh.param("lorentzian_sigma", parameters_.lorentzian_sigma, 0.2);
  registerParameter(nh, "lorentzian_sigma", lorentzian_sigma, 0.2, "Lorentzian sigma", 0.0, 10);
  //    nh.param("track_contact_weight", parameters_.track_contact_weight, 2.0);
  registerParameter(nh, "track_contact_weight", track_contact_weight, 2.0, "Track contact weight", 0.0, 10);
  //    nh.param("track_contact_nominal_value", parameters_.track_contact_nominal_value, 0.0);
  registerParameter(nh, "track_contact_nominal_value", track_contact_nominal_value, 10.0, "Track contact nominal value", 0.0, 10);
  //    nh.param("turning_resistance_weight", parameters_.turning_resistance_weight, 1.0);
  registerParameter(nh, "turning_resistance_weight", turning_resistance_weight, 1.0, "Turning resistance weight", 0.0, 10);
  //    nh.param("body_contact_weight", parameters_.body_contact_weight, 4.0);
  registerParameter(nh, "body_contact_weight", body_contact_weight, 4.0, "Body contact weight", 0.0, 10);
  //    nh.param("position_deviation_weight", parameters_.position_deviation_weight, 10.0);
  registerParameter(nh, "position_deviation_weight", position_deviation_weight, 10.0, "Position deviation weight", 0.0, 10);
  //    nh.param("roll_cost_weight", parameters_.roll_cost_weight, 2.0);
  registerParameter(nh, "roll_cost_weight", roll_cost_weight, 0.0, "Roll cost weight", 0.0, 10);
  //    nh.param("stability_weight", parameters_.stability_weight, 1.0);
  registerParameter(nh, "stability_weight", stability_weight, 1.0, "Stability cost weight", 0.0, 10);
  //    nh.param("stability_nominal_value", parameters_.stability_nominal_value, 0.0);
  registerParameter(nh, "stability_nominal_value", stability_nominal_value, 0.0, "Stability nominal value", 0.0, 10);
  registerParameter(nh, "stability_exp_parameters_a", stability_exp_parameters[0], 1.6, "Support area weight", 0.0, 100);
  registerParameter(nh, "stability_exp_parameters_b", stability_exp_parameters[1], -1.7, "Support area weight", -100.0, 0);
  registerParameter(nh, "stability_exp_parameters_c", stability_exp_parameters[2], 0.0, "Support area weight", 0.0, 10);
  return false;
}

}
