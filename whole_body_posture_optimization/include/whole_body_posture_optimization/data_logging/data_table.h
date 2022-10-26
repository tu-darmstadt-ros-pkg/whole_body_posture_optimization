//
// Created by martin on 26.10.22.
//

#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_DATA_TABLE_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_DATA_TABLE_H

#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace whole_body_posture_optimization {
class DataTable {
public:
  void initialize(const std::vector<std::string>& data_labels);
  void addData(const Eigen::VectorXd& data);
  bool saveToCsv(const std::string& file_name) const;
private:
  std::vector<std::string> labels_;
  Eigen::MatrixXd data_;
};

}
#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_DATA_TABLE_H
