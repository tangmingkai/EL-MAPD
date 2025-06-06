#pragma once
#include <memory>
#include <vector>

#include "planner/lns/neighbor.h"
#include "planner/cat/cat.h"
#include "common/time_limiter.h"
namespace planner {
namespace lns {
class BaseAgentSampler {
 public:
  virtual void Reset() {}
  virtual void HaveUpdate(const Neighbor &neighbor) {}
  virtual bool Sample(const TimeLimiter &time_limiter,
                      const std::vector<PathPlan> &path_plans,
                      const int &thread_id, const CAT &cat,
                      Neighbor *neighbor) = 0;
  virtual std::string name() const { return "ChargeAgentSampler";}
};
using BaseAgentSamplerPtr = std::shared_ptr<BaseAgentSampler>;
}  // namespace lns
}  // namespace planner
