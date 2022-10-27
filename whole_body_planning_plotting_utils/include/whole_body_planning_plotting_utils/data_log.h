#ifndef WHOLE_BODY_PLANNING_PLOTTING_UTILS_DATA_LOG_H
#define WHOLE_BODY_PLANNING_PLOTTING_UTILS_DATA_LOG_H

#include <whole_body_planning_plotting_utils/data_table.h>

#include <unordered_map>

namespace whole_body_planning_plotting_utils {

std::string getTimestamp();

class DataLog
{
public:
  static DataLog& getInstance();

  DataLog(DataLog const&) = delete;
  void operator=(DataLog const&) = delete;

  DataTable& getTable(const std::string& table_name);
  bool saveTablesToFolder(const std::string& folder_name);

  void enableLogging();
  bool loggingEnabled() const;
private:
  DataLog();
  std::unordered_map<std::string, DataTable> tables_;
  bool logging_enabled_;
};

}

#endif  // WHOLE_BODY_PLANNING_PLOTTING_UTILS_DATA_LOG_H
