#include <whole_body_posture_optimization/optimization_parameters_base.h>

#include <whole_body_posture_optimization/util/parameter_utils.h>

namespace whole_body_posture_optimization {

OptimizationParametersBase::OptimizationParametersBase()
: verbose(false)
{}

bool OptimizationParametersBase::loadParametersFromNamespace(const rclcpp::Node::SharedPtr& node)
{
  prefix_ = param_utils::paramPrefix(*node);
  bool success = loadParametersFromNamespaceImpl(node);
  success &= loadBaseParametersFromNamespace(node);
  callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&OptimizationParametersBase::parametersCallback, this, std::placeholders::_1));
  return success;
}

void OptimizationParametersBase::registerParameter(const rclcpp::Node::SharedPtr& node, const std::string& name, double& value,
                                                   double default_value, const std::string& description, double min,
                                                   double max)
{
  const std::string param_name = prefix_ + name;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min;
  range.to_value = max;
  descriptor.floating_point_range.push_back(range);
  // Use the parameters interface directly: the rclcpp::Node facade applies the
  // sub-namespace of sub-nodes in get_parameter (but not in declare_parameter)
  auto parameters = node->get_node_parameters_interface();
  if (!parameters->has_parameter(param_name)) {
    try {
      value = parameters->declare_parameter(param_name, rclcpp::ParameterValue(default_value), descriptor).get<double>();
    } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Parameter '" << param_name << "' has the wrong type. Using default.");
      value = default_value;
    }
  } else {
    value = parameters->get_parameter(param_name).as_double();
  }
  config_[param_name] = &value;
}

std::string OptimizationParametersBase::toString() const
{
  std::stringstream ss;
  for (auto const& parameter: config_) {
    ss << parameter.first << " = " << *parameter.second << std::endl;
  }
  return ss.str();
}

bool OptimizationParametersBase::loadBaseParametersFromNamespace(const rclcpp::Node::SharedPtr& node)
{
  verbose_param_name_ = prefix_ + "verbose";
  verbose = param_utils::param(*node, verbose_param_name_, false);
  verbose_ptr_ = &verbose;
  return true;
}

rcl_interfaces::msg::SetParametersResult OptimizationParametersBase::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter& parameter: parameters) {
    auto it = config_.find(parameter.get_name());
    if (it != config_.end() && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      *(it->second) = parameter.as_double();
    } else if (verbose_ptr_ && parameter.get_name() == verbose_param_name_ &&
               parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      *verbose_ptr_ = parameter.as_bool();
    }
  }
  return result;
}

}
