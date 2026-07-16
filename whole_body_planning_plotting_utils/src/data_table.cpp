#include <whole_body_planning_plotting_utils/data_table.h>

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace whole_body_planning_plotting_utils {

DataTable::DataTable()
: enabled_(false) {}

void DataTable::initialize(const std::vector<std::string>& data_labels)
{
  if (!labels_.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_planning_plotting_utils"), "Table has already been initialized.");
    return;
  }
  labels_ = data_labels;
}

void DataTable::addData(const Eigen::VectorXd& data)
{
  if (labels_.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_planning_plotting_utils"), "Table has not been initialized yet");
    return;
  }
  if (static_cast<size_t>(data.size()) != labels_.size()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_planning_plotting_utils"), "Data has wrong size. Expected: " << labels_.size() << ", received: " << data.size());
    return;
  }
  Eigen::Index num_entries = data_.rows();
  data_.conservativeResize(num_entries + 1, labels_.size());
  data_.row(num_entries) = data.transpose();
}

bool DataTable::saveToCsv(const std::string& file_name) const
{
  if (data_.rows() == 0) {
    return true;
  }
  std::filesystem::path parent_directory = std::filesystem::path(file_name).parent_path();
  try {
    std::filesystem::create_directories(parent_directory);
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_planning_plotting_utils"), e.what());
    return false;
  }
  catch (const std::bad_alloc& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_planning_plotting_utils"), e.what());
    return false;
  }
  std::ofstream file;
  file.open(file_name, std::ios::out);

  // Write labels
  for (size_t i = 0; i < labels_.size(); ++i) {
    file << labels_[i];
    if (i < labels_.size() - 1) {
      file << ",";
    }
  }
  file << std::endl;

  // Write data
  for (Eigen::Index row = 0; row < data_.rows(); ++row) {
    for (Eigen::Index col = 0; col < data_.cols(); ++col) {
      file << data_(row, col);
      if (col < data_.cols() - 1) {
        file << ",";
      }
    }
    file << std::endl;
  }
  file.close();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("whole_body_planning_plotting_utils"), "Saved " << data_.rows() << " data points to '" << file_name << "'");
  return true;
}

size_t DataTable::getDataSize() const
{
  return labels_.size();
}

bool DataTable::enabled() const
{
  return enabled_;
}

void DataTable::enableLogging()
{
  enabled_ = true;
}

void DataTable::enableLogging(bool enabled)
{
  enabled_ = enabled;
}

void DataTable::clear()
{
  data_ = Eigen::MatrixXd();
}

}


