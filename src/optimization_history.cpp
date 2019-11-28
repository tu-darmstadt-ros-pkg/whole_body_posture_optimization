#include <whole_body_posture_optimization/optimization_history.h>

namespace whole_body_posture_optimization {

HistoryWriter::HistoryWriter(OptimizationHistory& history)
  : history_(history) {}

HistoryWriter::~HistoryWriter() {}

ObjectiveWriter::ObjectiveWriter(OptimizationHistory& history)
  : HistoryWriter(history) {}

ObjectiveWriter::~ObjectiveWriter() {}

void ObjectiveWriter::addEntry(const std::vector<double>& x, double fvalue)
{
  OptimizationResult result;
  result.parameters = x;
  result.function_value = fvalue;
  result.constraints.resize(history_.getNumConstraints());
  history_.addEntry(result);
}

ConstraintWriter::ConstraintWriter(OptimizationHistory& history, size_t idx)
  : HistoryWriter(history), idx_(idx) {}

ConstraintWriter::~ConstraintWriter() {}

void ConstraintWriter::addEntry(const std::vector<double>& x, double fvalue)
{
  // Check if entries are available
  if (history_.getHistoryLength() == 0) {
    return;
  }
  // Get last entry
  OptimizationResult& result = history_.getEntry(history_.getHistoryLength()-1);
  // Check if the parameter is correct
  if (is_equal(x, result.parameters)) {
    result.constraints[idx_] = fvalue;
  }
}

HistoryWriter* OptimizationHistory::setObjective()
{
  objective_writer_ = std::make_shared<ObjectiveWriter>(*this);
  return static_cast<HistoryWriter*>(objective_writer_.get());
}

HistoryWriter* OptimizationHistory::addConstraint()
{
  ConstraintWriter writer(*this, constraint_writer_.size());
  constraint_writer_.push_back(writer);
  return static_cast<HistoryWriter*>(&constraint_writer_[constraint_writer_.size()-1]);
}

size_t OptimizationHistory::getNumConstraints() const
{
  return constraint_writer_.size();
}

void OptimizationHistory::addEntry(const OptimizationResult& result)
{
  history_.push_back(result);
}

OptimizationResult& OptimizationHistory::getEntry(const size_t& idx)
{
  return history_[idx];
}

size_t OptimizationHistory::getHistoryLength() const
{
  return history_.size();
}

const OptimizationResult* OptimizationHistory::getBestResult() const
{
  int best_idx = -1;
  double min_fvalue = std::nan("");

  for (unsigned int i = 0; i < getHistoryLength(); ++i) {
    const OptimizationResult& result = history_[i];
    if (result.isValid()) {
      if (best_idx == -1 || result.function_value < min_fvalue) {
        best_idx = static_cast<int>(i);
        min_fvalue = result.function_value;
      }
    }
  }

  if (best_idx == -1) {
    return nullptr;
  } else {
    return &history_[best_idx];
  }
}

bool OptimizationResult::isValid() const
{
  for (const double& constraint_value: constraints) {
    if (constraint_value > 0) {
      return false;
    }
  }
  return true;
}

}
