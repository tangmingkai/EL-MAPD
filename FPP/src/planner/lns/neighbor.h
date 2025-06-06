#pragma once
#include <utility>
#include <vector>

#include "planner/path_plan.h"
namespace planner {
namespace lns {
struct Neighbor {
  std::vector<int> agent_ids;
  std::vector<PathPlan> path_plans;
  // double sum_of_costs;
  int sampler_id;
  bool succ = false;
  void Reset() {
    succ = false;
    agent_ids.clear();
    path_plans.clear();
    sampler_id = -1;
  }
};
};  // namespace lns
};  // namespace planner
