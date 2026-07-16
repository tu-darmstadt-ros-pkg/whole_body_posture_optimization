#include <whole_body_posture_optimization/arm/stability_cost_parameters.h>

namespace whole_body_posture_optimization {

StabilityCostParameters::StabilityCostParameters()
: OptimizationParametersBase(),
  exp_parameters{0, 0, 0},
  movement_cost_weight(0.0),
  lorentzian_sigma(0.0),
  function_scale(0.9)
{}

bool StabilityCostParameters::loadParametersFromNamespaceImpl(const rclcpp::Node::SharedPtr& node)
{
  registerParameter(node, "movement_cost_weight", movement_cost_weight, 0.2, "Movement cost weight", 0.0, 10);
  registerParameter(node, "lorentzian_sigma", lorentzian_sigma, 0.2, "Lorentzian sigma", 0.0, 10);
  registerParameter(node, "exp_parameters_a", exp_parameters[0], 1.6, "Support area weight", 0.0, 100);
  registerParameter(node, "exp_parameters_b", exp_parameters[1], -1.7, "Support area weight", -100.0, 0);
  registerParameter(node, "exp_parameters_c", exp_parameters[2], 0.0, "Support area weight", 0.0, 10);
  registerParameter(node, "function_scale", function_scale, 1.0, "Function scale", 0.0, 10);
  return true;
}


}
