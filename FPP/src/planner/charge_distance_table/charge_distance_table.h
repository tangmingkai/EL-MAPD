#pragma once
#include <string>
#include <utility>
#include <vector>

#include "env/environment.h"
#include "planner/heuristic/heuristic_table.h"
namespace planner {
class HeuristicTable;

class ChargeDistanceTable {
 public:
  explicit ChargeDistanceTable(const Environment &env);
  void Preprocess(
    const std::string &suffix,
    const HeuristicTable &unloaded_energy_heuristic_table);
  const std::vector<std::pair<double, int>> &Get(
      const int &loc) const;
  const std::vector<std::pair<double, int>> &Get(
      const int &loc, const int &orient) const;
 private:
  std::vector<std::vector<std::pair<double, int>>> charge_distance_table_;
  std::vector<std::vector<std::pair<double, int>>>
      charge_orient_distance_table_;
  const Environment &env_;
};
}