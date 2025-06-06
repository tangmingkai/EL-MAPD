#include "planner/lns/agent_sampler/charge_agent_sampler.h"

#include "planner/utils/utils.h"
#include <queue>
namespace planner {
namespace lns {

ChargeAgentSampler::ChargeAgentSampler(
    const Environment &env, const ActionModel &action_model,
    const CostCalculator &cost_calculator, const int &neighbor_size,
    const int &thread_num, const double &recharge_threshold, const uint &seed)
    : BaseConflictGraphSampler(env, action_model, cost_calculator,
                               neighbor_size, thread_num, recharge_threshold,
                               seed),
      mt_(seed) {
  empty_charge_point_allocated_agent_id_.resize(env.charges_locs.size(), -1);
}

void ChargeAgentSampler::ConstructConflictGraph(
    const TimeLimiter &time_limiter,
    const std::vector<PathPlan> &path_plans,
    std::vector<std::vector<int>> *graph) {
  std::vector<std::unordered_set<int>> g(path_plans.size());
  std::vector<std::vector<int>> charge_agents(env_.charges_locs.size());
  for (int i = 0; i < path_plans.size(); i++) {
    auto &path_plan = path_plans[i];
    int charge_id = path_plan.charge_point_id;
    charge_agents[path_plan.charge_point_id].emplace_back(i);
    std::vector<int> min_cost_charge_ids;
    if (path_plan.IsPassedDelivery()) {
      auto l_state = path_plan.GetCurrentLState();
      min_cost_charge_ids = cost_calculator_.GetAllMinCostChargeID(
          empty_charge_point_allocated_agent_id_, i, l_state.state.location,
          l_state.state.orientation);
    } else {
      min_cost_charge_ids = cost_calculator_.GetAllMinCostChargeID(
          empty_charge_point_allocated_agent_id_, i, path_plan.DeliveryLoc());
    }
    for (auto charge_id : min_cost_charge_ids) {
      charge_agents[charge_id].emplace_back(i);
    }
  }
  for (auto &agents : charge_agents) {
    if (agents.size() <= 1) continue;
    std::shuffle(agents.begin(), agents.end(), mt_);
    for (int i = 1; i < agents.size(); i++) {
      if (agents[i] == agents[i - 1]) continue;
      g[agents[i - 1]].insert(agents[i]);
      g[agents[i]].insert(agents[i - 1]);
    }
  }
  graph->resize(path_plans.size());
  for (int i = 0; i < path_plans.size(); i++) {
    // std::cout << "g[i].size():" <<i << " " << g[i].size() << std::endl;
    if (!g[i].empty())
      (*graph)[i].assign(g[i].begin(), g[i].end());
  }
}


}  // namespace lns
}  // namespace planner
