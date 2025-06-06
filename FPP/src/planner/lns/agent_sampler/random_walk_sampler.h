#pragma once
#include <unordered_set>
#include <vector>
#include <set>
#include <unordered_map>
#include "planner/lns/agent_sampler/base_agent_sampler.h"
#include "planner/cost_calculator/cost_calculator.h"
#include "env/environment.h"
#include "model/action_model.h"
#include "planner/guidance_map/guidance_map.h"
#include "planner/heuristic/heuristic_table.h"
namespace planner {
namespace lns {
class RandomWalkSampler : public BaseAgentSampler {
 public:
  RandomWalkSampler(const Environment &env,
                    const ActionModel &action_model,
                    const CostCalculator &cost_calculator,
                    const int &neighbor_size,
                    const int &thread_num,
                    const double &recharge_threshold,
                    const uint &seed);
  void Reset() override;
  void HaveUpdate(const Neighbor &neighbor) override;
  bool Sample(const TimeLimiter &time_limiter,
              const std::vector<PathPlan> &path_plans, const int &thread_id,
              const CAT &cat, Neighbor *neighbor) override;
  virtual std::string name() const { return "RandomWalkSampler";}
 
 private:
  int FindMostGapAgent(const std::vector<PathPlan> &path_plans,
                                        const int &thread_id);
  double GetGap(const PathPlan &plan, const int &i);
  int GetHeuristicValue(const int &stage, const int &location,
                        const int &orientation, const PathPlan &plan,
                        const ActionModel &action_model,
                        const GuidanceMap &guidancemap,
                        const HeuristicTable &loaded_heuristic_table,
                        const HeuristicTable &unloaded_heuristic_table) const;
  void RandomWalk(
    const std::vector<PathPlan> path_plans,
    const PathPlan &plan, const int &start_id, const int &neighbor_size,
    const CAT &cat, std::unordered_set<int> *conflicting_agents) const;
  double GetPartialPathCost(const PathPlan &plan, const int &time_step) const;
  double GetActionCost(const bool &is_loaded, const int &location,
                       const int &orientation, const Action &action) const;
  const ActionModel &action_model_;
  const CostCalculator &cost_calculator_;
  const Environment &env_;
  mutable uint seed_;
  int neighbor_size_;
  int thread_num_;
  double energy_weight_;
  CAT empty_cat_;
  std::vector<std::unordered_set<int>> tabu_list_list_;
  std::unordered_map<int, double> main_heuristic_cache_;
};
}  // namespace lns
}  // namespace planner
