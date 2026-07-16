#include <whole_body_posture_optimization/util/parameter_utils.h>

namespace whole_body_posture_optimization {
namespace param_utils {

std::string paramPrefix(const rclcpp::Node& node)
{
  std::string sub_ns = node.get_sub_namespace();
  if (sub_ns.empty()) {
    return "";
  }
  std::replace(sub_ns.begin(), sub_ns.end(), '/', '.');
  return sub_ns + ".";
}

std::map<std::string, double> getDoubleMap(rclcpp::Node& node, const std::string& name,
                                           const std::map<std::string, double>& default_value)
{
  std::map<std::string, double> result = default_value;
  const std::string prefix = name + ".";
  const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& [key, value]: overrides) {
    if (key.rfind(prefix, 0) != 0) {
      continue;
    }
    std::string entry = key.substr(prefix.size());
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result[entry] = value.get<double>();
    } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      result[entry] = static_cast<double>(value.get<int64_t>());
    } else {
      RCLCPP_WARN_STREAM(node.get_logger(), "Ignoring parameter '" << key << "': not a double.");
    }
  }
  return result;
}

std::map<std::string, std::vector<std::string>> getStringListMap(rclcpp::Node& node, const std::string& name)
{
  std::map<std::string, std::vector<std::string>> result;
  const std::string prefix = name + ".";
  const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& [key, value]: overrides) {
    if (key.rfind(prefix, 0) != 0) {
      continue;
    }
    std::string entry = key.substr(prefix.size());
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      result[entry] = value.get<std::vector<std::string>>();
    } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      result[entry] = {value.get<std::string>()};
    } else {
      RCLCPP_WARN_STREAM(node.get_logger(), "Ignoring parameter '" << key << "': not a string list.");
    }
  }
  return result;
}

}
}
