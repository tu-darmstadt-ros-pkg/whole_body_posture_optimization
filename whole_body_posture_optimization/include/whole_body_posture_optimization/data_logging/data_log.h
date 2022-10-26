#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_DATA_LOG_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_DATA_LOG_H

#include <whole_body_posture_optimization/data_logging/data_table.h>

#include <unordered_map>

namespace whole_body_posture_optimization {

class DataLog
{
public:
  static DataLog& getInstance();

  DataLog(DataLog const&) = delete;
  void operator=(DataLog const&) = delete;

  DataTable& getTable(const std::string& table_name);
  bool saveTablesToFolder(const std::string& folder_name);
private:
  DataLog() = default;
  std::unordered_map<std::string, DataTable> tables_;
};

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_DATA_LOG_H
