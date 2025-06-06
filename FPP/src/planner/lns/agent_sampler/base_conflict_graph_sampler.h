#pragma once
#include <unordered_set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <set>
#include "planner/lns/agent_sampler/base_agent_sampler.h"
#include "planner/cost_calculator/cost_calculator.h"
#include "env/environment.h"
#include "model/action_model.h"
namespace planner {
namespace lns {
class BaseConflictGraphSampler : public BaseAgentSampler {
 public:
  BaseConflictGraphSampler(const Environment &env,
                           const ActionModel &action_model,
                           const CostCalculator &cost_calculator,
                           const int &neighbor_size, const int &thread_num,
                           const double &recharge_threshold, const uint &seed);
  virtual void Reset() override;
  virtual void HaveUpdate(const Neighbor &neighbor) override;
  bool Sample(const TimeLimiter &time_limiter,
              const std::vector<PathPlan> &path_plans, const int &thread_id,
              const CAT &cat, Neighbor *neighbor) override;
  virtual std::string name() const = 0;

 protected:
  int FindMostGapAgent(
    const std::vector<PathPlan> &path_plans, const int &thread_id);
  virtual void ConstructConflictGraph(
                              const TimeLimiter &time_limiter,
                              const std::vector<PathPlan> &path_plans,
                              std::vector<std::vector<int>> *graph) = 0;
  double GetGap(const PathPlan &plan, const int &i);
  const Environment &env_;
  const ActionModel &action_model_;
  const CostCalculator &cost_calculator_;
  int neighbor_size_;
  int thread_num_;
  uint seed_;
  bool is_init_;
  std::vector<PathPlan> optimal_path_plans_;
  std::vector<std::vector<int>> graph_;
  std::unordered_map<int, int> vertex_occupied_;
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_occupied_;
  std::vector<std::unordered_set<int>> tabu_list_list_;
  std::unordered_map<int, double> main_heuristic_cache_;
};
}  // namespace lns
}  // namespace planner
