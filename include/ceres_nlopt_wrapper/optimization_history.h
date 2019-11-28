#ifndef CERES_NLOPT_WRAPPER_OPTIMIZATION_HISTORY_H
#define CERES_NLOPT_WRAPPER_OPTIMIZATION_HISTORY_H

#include <ros/ros.h>

namespace ceres_nlopt_wrapper {

// Predeclare
class OptimizationHistory;

class HistoryWriter {
public:
  HistoryWriter(OptimizationHistory& history);
  virtual ~HistoryWriter();
  virtual void addEntry(const std::vector<double>& x, double fvalue) = 0;
protected:
  OptimizationHistory& history_;
};

class ObjectiveWriter : public HistoryWriter {
public:
  ObjectiveWriter(OptimizationHistory& history);
  virtual ~ObjectiveWriter() override;
  virtual void addEntry(const std::vector<double> &x, double fvalue) override;
};

class ConstraintWriter : public HistoryWriter {
public:
  ConstraintWriter(OptimizationHistory& history, size_t idx);
  virtual ~ConstraintWriter() override;
  virtual void addEntry(const std::vector<double> &x, double fvalue) override;
private:
  size_t idx_;
};

struct OptimizationResult {
  std::vector<double> parameters;
  double function_value;
  std::vector<double> constraints;

  bool isValid(double tolerance) const;
};

class OptimizationHistory {
public:
  std::vector<OptimizationResult> history_;

  HistoryWriter* setObjective();
  HistoryWriter* addConstraint();
  size_t getNumConstraints() const;

  void addEntry(const OptimizationResult& result);
  OptimizationResult& getEntry(const size_t& idx);
  size_t getHistoryLength() const;
  std::string toString() const;

  const OptimizationResult* getBestResult() const;
private:
  std::shared_ptr<ObjectiveWriter> objective_writer_;
  std::vector<std::shared_ptr<ConstraintWriter>> constraint_writer_;
};

}

#endif
