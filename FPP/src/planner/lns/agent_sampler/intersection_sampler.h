#pragma once
#include <unordered_set>
#include <vector>
#include <set>
#include "planner/lns/agent_sampler/base_agent_sampler.h"
#include "planner/cost_calculator/cost_calculator.h"
#include "env/environment.h"
#include "model/action_model.h"
namespace planner {
namespace lns {
class IntersectionSampler : public BaseAgentSampler {
 public:
  IntersectionSampler(const Environment &env, const ActionModel &action_model,
                      const CostCalculator &cost_calculator,
                      const int &neighbor_size, const int &thread_num,
                      const double &recharge_threshold,
                      const uint &seed);
  void Reset() override;
  void HaveUpdate(const Neighbor &neighbor) override;
  bool Sample(const TimeLimiter &time_limiter,
              const std::vector<PathPlan> &path_plans, const int &thread_id,
              const CAT &cat, Neighbor *neighbor) override;
  virtual std::string name() const { return "IntersectionSampler";}

 private:
  std::vector<int> intersection_locs_;
  std::vector<bool> is_intersections_;
  std::vector<int> good_start_intersections_locs_;
  const Environment &env_;
  uint seed_;
  int neighbor_size_;
  std::mt19937 mt_;
  bool has_update_;
};
}  // namespace lns
}  // namespace planner
