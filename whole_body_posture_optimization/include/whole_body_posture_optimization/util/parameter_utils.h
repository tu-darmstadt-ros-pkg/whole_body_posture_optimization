#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_PARAMETER_UTILS_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_PARAMETER_UTILS_H

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <vector>

namespace whole_body_posture_optimization {
namespace param_utils {

// Note: All functions use the node parameters interface directly instead of the
// rclcpp::Node facade. The facade's get_parameter() applies the sub-namespace of
// sub-nodes (while declare_parameter() does not), which breaks the manual
// "<sub_ns>.<name>" prefix convention used throughout this stack.

/// Returns the parameter prefix ("<sub_ns>." or "") for a (sub-)node.
/// Parameters of sub-nodes are not namespaced automatically in ROS 2, so the
/// sub-namespace is prepended to the parameter names manually (same convention
/// as in sdf_contact_estimation).
std::string paramPrefix(const rclcpp::Node& node);

/// Mirrors ros::NodeHandle::param: declare if needed, return override or default.
template<typename T>
T param(rclcpp::Node& node, const std::string& name, const T& default_value)
{
  auto parameters = node.get_node_parameters_interface();
  if (!parameters->has_parameter(name)) {
    try {
      return parameters->declare_parameter(name, rclcpp::ParameterValue(default_value)).get<T>();
    } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Parameter '" << name << "' has the wrong type. Using default.");
      return default_value;
    }
  }
  rclcpp::Parameter parameter;
  if (parameters->get_parameter(name, parameter)) {
    try {
      return parameter.get_value<T>();
    } catch (const rclcpp::ParameterTypeException&) {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Parameter '" << name << "' has the wrong type. Using default.");
    }
  }
  return default_value;
}

/// Mirrors ros::NodeHandle::getParam: true only if the parameter was set by the user.
template<typename T>
bool getParam(rclcpp::Node& node, const std::string& name, T& value)
{
  auto parameters = node.get_node_parameters_interface();
  if (!parameters->has_parameter(name)) {
    const auto& overrides = parameters->get_parameter_overrides();
    if (overrides.find(name) == overrides.end()) {
      return false;
    }
    try {
      value = parameters->declare_parameter(name, rclcpp::ParameterValue(value)).get<T>();
      return true;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Parameter '" << name << "' has the wrong type.");
      return false;
    }
  }
  rclcpp::Parameter parameter;
  if (!parameters->get_parameter(name, parameter)) {
    return false;
  }
  try {
    value = parameter.get_value<T>();
  } catch (const rclcpp::ParameterTypeException&) {
    RCLCPP_ERROR_STREAM(node.get_logger(), "Parameter '" << name << "' has the wrong type.");
    return false;
  }
  return true;
}

/// Reads a string->double map from the parameter overrides, e.g. yaml
///   environment_collision_padding:
///     gripper_palm_link: 0.2
/// with name "environment_collision_padding" yields {gripper_palm_link: 0.2}.
std::map<std::string, double> getDoubleMap(rclcpp::Node& node, const std::string& name,
                                           const std::map<std::string, double>& default_value = {});

/// Reads a string->string-list map from the parameter overrides, e.g. yaml
///   self_collision_allowed_links:
///     arm_link_1: [arm_link_3]
/// with name "self_collision_allowed_links" yields {arm_link_1: [arm_link_3]}.
std::map<std::string, std::vector<std::string>> getStringListMap(rclcpp::Node& node, const std::string& name);

}
}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_PARAMETER_UTILS_H
