#pragma once
#include <unordered_set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <set>
#include "planner/lns/agent_sampler/base_conflict_graph_sampler.h"
#include "planner/cost_calculator/cost_calculator.h"
#include "env/environment.h"
#include "model/action_model.h"
namespace planner {
namespace lns {
class OptimalPathSampler : public BaseConflictGraphSampler {
 public:
  OptimalPathSampler(const Environment &env, const ActionModel &action_model,
                     const CostCalculator &cost_calculator,
                     const int &neighbor_size, const int &thread_num,
                     const double &recharge_threshold,
                     const uint &seed);
  virtual std::string name() const { return "OptimalPathSampler";}

 private:
  void ConstructConflictGraph(const TimeLimiter &time_limiter,
                              const std::vector<PathPlan> &path_plans,
                              std::vector<std::vector<int>> *graph) override;
  std::vector<int> empty_charge_point_allocated_agent_id_;
  double recharge_threshold_;
};
}  // namespace lns
}  // namespace planner
