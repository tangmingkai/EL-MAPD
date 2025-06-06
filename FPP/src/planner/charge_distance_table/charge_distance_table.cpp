#include "planner/charge_distance_table/charge_distance_table.h"
#include <algorithm>

namespace planner {
ChargeDistanceTable::ChargeDistanceTable(const Environment &env) : env_(env) {}

void ChargeDistanceTable::Preprocess(
    const std::string &suffix,
    const HeuristicTable &unloaded_main_heuristic_table) {
  charge_distance_table_.resize(env_.rows* env_.cols);
  #pragma omp parallel for schedule(dynamic, 1)
  for (int loc_idx = 0; loc_idx < env_.rows* env_.cols; ++loc_idx) {
    charge_distance_table_[loc_idx].clear();
    for (int charge_id = 0; charge_id < env_.charges_locs.size(); charge_id++) {
      double main_heuristic = unloaded_main_heuristic_table.Get(
          loc_idx, env_.charges_locs[charge_id]);
      charge_distance_table_[loc_idx].push_back({main_heuristic, charge_id});
    }
    std::sort(charge_distance_table_[loc_idx].begin(),
              charge_distance_table_[loc_idx].end());
  }

  charge_orient_distance_table_.resize(env_.rows* env_.cols*4);
  #pragma omp parallel for schedule(dynamic, 1)
  for (int id = 0; id < env_.rows* env_.cols*4; ++id) {
    charge_orient_distance_table_[id].clear();
    int loc_idx = id / 4;
    int orient_idx = id % 4;
    for (int charge_id = 0; charge_id < env_.charges_locs.size(); charge_id++) {
      double main_heuristic = unloaded_main_heuristic_table.Get(
          loc_idx, orient_idx, env_.charges_locs[charge_id]);
      charge_orient_distance_table_[id].push_back({main_heuristic, charge_id});
    }
    std::sort(charge_orient_distance_table_[id].begin(),
              charge_orient_distance_table_[id].end());
  }
}

const std::vector<std::pair<double, int>> &
ChargeDistanceTable::Get(const int &loc) const {
  return charge_distance_table_[loc];
}
const std::vector<std::pair<double, int>> &
ChargeDistanceTable::Get(const int &loc, const int &orient) const {
  return charge_orient_distance_table_[loc * 4 + orient];
}
}  // namespace planner
