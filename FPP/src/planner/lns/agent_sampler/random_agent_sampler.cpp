#include "planner/lns/agent_sampler/random_agent_sampler.h"

#include <queue>

namespace planner {
namespace lns {
RandomAgentSampler::RandomAgentSampler(const Environment &env,
                                       const ActionModel &action_model,
                                       const CostCalculator &cost_calculator,
                                       const int &neighbor_size,
                                       const int &thread_num,
                                       const double &recharge_threshold,
                                       const uint &seed)
    : neighbor_size_(neighbor_size), seed_(seed) {}
void RandomAgentSampler::Reset() { }
bool RandomAgentSampler::Sample(const TimeLimiter &time_limiter,
                                const std::vector<PathPlan> &path_plans,
                                const int &thread_id, const CAT &cat,
                                Neighbor *neighbor) {
  std::unordered_set<int> neighbors_set;
  int count = 0;
  while (neighbors_set.size() < neighbor_size_ && count < neighbor_size_+20) {
    int id = rand_r(&seed_) % path_plans.size();
    if (!path_plans[id].IsIdle())
      neighbors_set.insert(id);
    count++;
  }
  neighbor->agent_ids.assign(neighbors_set.begin(), neighbors_set.end());
  return true;
}
}  // namespace lns
}  // namespace planner
