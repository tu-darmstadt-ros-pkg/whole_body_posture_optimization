#include <whole_body_planning_plotting_utils/data_log.h>

#include <ctime>
#include <iomanip>

namespace whole_body_planning_plotting_utils {

std::string getTimestamp()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

DataLog& DataLog::getInstance()
{
  static DataLog instance; // Instantiated on first use.
  return instance;
}
DataTable& DataLog::getTable(const std::string& table_name)
{
  return tables_[table_name];
}
bool DataLog::saveTablesToFolder(const std::string& folder_name)
{
  bool success = true;
  for (const auto& x: tables_) {
    std::string file_name = folder_name + "/" + x.first + "-" + getTimestamp() + ".csv";
    success = success && x.second.saveToCsv(file_name);
  }
  return success;
}

}