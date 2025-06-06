#pragma once
#include <string>
#include <vector>

#include "model/action_model.h"
namespace planner {
class GuidanceMap {
 public:
  GuidanceMap() = default;
  explicit GuidanceMap(const Environment &env);
  std::string Load(const std::string &path);
  double GetWeight(const int &location, const int &orientation,
                   const Action &action) const;

 private:
  std::vector<double> map_weights_;
  const Environment &env_;
  int n_orientation_;
};
};  // namespace planner
