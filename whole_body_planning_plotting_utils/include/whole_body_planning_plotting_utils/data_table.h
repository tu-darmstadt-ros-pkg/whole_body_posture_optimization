#ifndef WHOLE_BODY_PLANNING_PLOTTING_UTILS_DATA_TABLE_H
#define WHOLE_BODY_PLANNING_PLOTTING_UTILS_DATA_TABLE_H

#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace whole_body_planning_plotting_utils {

class DataTable {
public:
  DataTable();
  void initialize(const std::vector<std::string>& data_labels);
  void enableLogging();
  bool enabled() const;
  void addData(const Eigen::VectorXd& data);
  bool saveToCsv(const std::string& file_name) const;
  size_t getDataSize() const;
private:
  bool enabled_;
  std::vector<std::string> labels_;
  Eigen::MatrixXd data_;
};

}
#endif  // WHOLE_BODY_PLANNING_PLOTTING_UTILS_DATA_TABLE_H
