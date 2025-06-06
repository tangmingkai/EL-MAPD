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
class RandomAgentSampler : public BaseAgentSampler {
 public:
  RandomAgentSampler(const Environment &env, const ActionModel &action_model,
                     const CostCalculator &cost_calculator,
                     const int &neighbor_size, const int &thread_num,
                     const double &recharge_threshold,
                     const uint &seed);
  void Reset() override;
  bool Sample(const TimeLimiter &time_limiter,
              const std::vector<PathPlan> &path_plans, const int &thread_id,
              const CAT &cat, Neighbor *neighbor) override;
  virtual std::string name() const { return "RandomAgentSampler";}

 private:
  int neighbor_size_;
  uint seed_;
};
}  // namespace lns
}  // namespace planner
