#pragma once

#include <algorithm>
#include <boost/heap/d_ary_heap.hpp>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/agent.h"
#include "env/environment.h"
#include "planner/cat/cat.h"
#include "planner/cost_calculator/cost_calculator.h"
#include "planner/heuristic/heuristic_table.h"
#include "planner/multi_label_a_star/l_state.h"
#include "planner/multi_label_a_star/mlastar_node.h"
#include "planner/path_plan.h"
#include "common/time_limiter.h"

namespace planner {
class MLAStar {
 public:
  MLAStar(const Environment &env, const ActionModel &action_model,
          const CostCalculator &cost_calculator);
  // The last goal might be change.
  bool RechargeAndFallbackPlanningLayer(
      const TimeLimiter &time_limiter, const LState &start_state,
      const CAT &cat, const std::vector<int> &goals, const AgentStatus &status,
      const std::vector<int> &ignore_agent_ids, const double &recharge_threshold,
      PathPlan *plan) const;

 private:
  bool EnergyConstraintedPathPlanningLayer(
      const TimeLimiter &time_limiter, const LState &start_state,
      const CAT &cat, const std::vector<int> &goals,
      const std::vector<int> &ignore_agent_ids, PathPlan *paths) const;

  void CalculateAccHeuristic(
      const std::vector<int> &goals,
      const GuidanceMap &guidemap,
      const HeuristicTable &unloaded_heuristic_table,
      const HeuristicTable &loaded_heuristic_table,
      std::vector<int> *acc_heurisitc) const;
  double GetMainHeuristic(const LState &state,
                                 const std::vector<int> &goals) const;
  double GetMinimumEneryComsumption(const LState &state,
                                 const std::vector<int> &goals) const;
  void ConstructByFallback(const State &start_state,
                           const std::vector<int> &goals, PathPlan *plan) const;
  void AddPathAndAction(const int &start_loc, const int &start_ori,
                        const int &end_loc, const int &end_ori,
                        const HeuristicTable &heuristic_table,
                        PathPlan *plan) const;
  const Environment& env_;

  // const GuidanceMap& guidance_map_;
  // const GuidanceMap& uniform_map_;
  const ActionModel& action_model_;
  // const ActionModel& dummy_action_model_;
  // const HeuristicTableSet& heuristic_table_set_;
  // std::vector<double> acc_main_heuristic_;
  // std::vector<double> acc_energy_heuristic_;
  // std::vector<int> acc_weighted_time_heuristic_;
  const CostCalculator& cost_calculator_;

};
};  // namespace planner
