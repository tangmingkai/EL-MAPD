#pragma once
#include <vector>
#include <utility>
#include "env/environment.h"
#include "model/action_model.h"
#include "planner/guidance_map/guidance_map.h"
#include "planner/heuristic/heuristic_table.h"
#include "planner/path_plan.h"
#include "planner/charge_distance_table/charge_distance_table.h"
#include "planner/cost_calculator/cost_type.h"
namespace planner {
class HeuristicTable;
class ChargeDistanceTable;


class CostCalculator {
 public:
  explicit CostCalculator(const Environment& env, const uint &seed);
  void Init(const double &energy_weight, const ActionModel &action_model,
            const GuidanceMap &guidance_map,
            const HeuristicTable &unloaded_main_heuristic_table,
            const HeuristicTable &loaded_main_heuristic_table,
            const HeuristicTable &unloaded_energy_heuristic_table,
            const HeuristicTable &loaded_energy_heuristic_table,
            const ChargeDistanceTable &charge_distance_table);
  double GetActionCostByType(const CostType &cost_type, const bool &is_loaded,
                             const int &location, const int &orientation,
                             const Action &action) const;
  double GetActionMainCost(const bool &is_loaded, const int &location,
                           const int &orientation, const Action &action) const;
  double GetActionPureEnergy(const bool &is_loaded, const int &location,
                             const int &orientation,
                             const Action &action) const;
  double GetRemainMainCost(const PathPlan &plan) const;
  void CalculateAccHeuristicByType(
    const CostType &cost_type,
    const std::vector<int> &goals,
    std::vector<double> *acc_heuristic) const;
  double GetMinimumEnergy(const std::vector<int> &goals) const;

  const HeuristicTable& unloaded_main_heuristic_table() const {
    return *unloaded_main_heuristic_table_;
  }
  const HeuristicTable& loaded_main_heuristic_table() const {
    return *loaded_main_heuristic_table_;
  }
  const HeuristicTable& unloaded_energy_heuristic_table() const {
    return *unloaded_energy_heuristic_table_;
  }
  const HeuristicTable& loaded_energy_heuristic_table() const {
    return *loaded_energy_heuristic_table_;
  }
  const ChargeDistanceTable &charge_distance_table() const {
    return *charge_distance_table_;
  }
  double GetMainHeuristic(int stage,
     const int &location,
     const int &orientation,
     const std::vector<int> &goals,
     const std::vector<double> &acc_main_heuristic) const;
  double GetMinimumEneryComsumption(int stage,
     const int &location,
     const int &orientation,
     const std::vector<int> &goals,
     const std::vector<double> &acc_main_heuristic) const;
  double GetUnloadedMainHeuristic(const int &loc1, const int &loc2) const;
  int SelectChargeID(
      const std::vector<int> &charge_point_allocated_agent_id,
      const int &current_agent_id, const int &loc) const;
  int SelectChargeID(
      const std::vector<int> &charge_point_allocated_agent_id,
      const int &current_agent_id, const int &loc, const int &orient) const;
  std::vector<int> GetAllMinCostChargeID(
      const std::vector<int> &charge_point_allocated_agent_id,
      const int &current_agent_id, const int &loc) const;
  std::vector<int> GetAllMinCostChargeID(
      const std::vector<int> &charge_point_allocated_agent_id,
      const int &current_agent_id, const int &loc, const int &orient) const;

 private:
  int SelectChargeID(
      const std::vector<int> &charge_point_allocated_agent_id,
      const int &current_agent_id,
      const std::vector<std::pair<double, int>> &distance_table) const;
  std::vector<int> GetAllMinCostChargeID(
      const std::vector<int> &charge_point_allocated_agent_id,
      const int &current_agent_id,
      const std::vector<std::pair<double, int>> &distance_table) const;
  void CalculateAccHeursitic(
    const CostType &cost_type,
    const std::vector<int> &goals,
    const HeuristicTable &unloaded_heuristic_table,
    const HeuristicTable &loaded_heuristic_table,
    std::vector<double> *acc_heurisitc) const;
  double energy_weight_;
  const ActionModel* action_model_;
  const GuidanceMap* guidance_map_;
  const HeuristicTable* unloaded_main_heuristic_table_;
  const HeuristicTable* loaded_main_heuristic_table_;
  const HeuristicTable* unloaded_energy_heuristic_table_;
  const HeuristicTable* loaded_energy_heuristic_table_;
  const ChargeDistanceTable* charge_distance_table_;

  const Environment& env_;
  mutable uint seed_;
  // const HeuristicTable* weighted_time_heuristic_table_;
};
}  // namespace planner
