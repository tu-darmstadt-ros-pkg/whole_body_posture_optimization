//
// Created by martin on 26.10.22.
//

#include "data_log.h"

namespace whole_body_posture_optimization {

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
    std::string file_name = folder_name + "/" + x.first + ".csv";
    success = success && x.second.saveToCsv(file_name);
  }
  return success;
}

}