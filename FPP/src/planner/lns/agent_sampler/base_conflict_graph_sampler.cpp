#include "planner/lns/agent_sampler/base_conflict_graph_sampler.h"

#include "planner/utils/utils.h"
#include <queue>
namespace planner {
namespace lns {

BaseConflictGraphSampler::BaseConflictGraphSampler(
    const Environment &env, const ActionModel &action_model,
    const CostCalculator &cost_calculator, const int &neighbor_size,
    const int &thread_num, const double &recharge_threshold, const uint &seed)
    : env_(env),
      action_model_(action_model),
      cost_calculator_(cost_calculator),
      neighbor_size_(neighbor_size),
      thread_num_(thread_num),
      seed_(seed) {
  tabu_list_list_.resize(thread_num_);
}

void BaseConflictGraphSampler::Reset() {
  is_init_ = false;
  main_heuristic_cache_.clear();
}
void BaseConflictGraphSampler::HaveUpdate(const Neighbor &neighbor) {
  is_init_ = false;
  for (auto &agent_id : neighbor.agent_ids) {
    auto iter = main_heuristic_cache_.find(agent_id);
    if (iter != main_heuristic_cache_.end()) main_heuristic_cache_.erase(iter);
  }
}
bool BaseConflictGraphSampler::Sample(
                                const TimeLimiter &time_limiter,
                                const std::vector<PathPlan> &path_plans,
                                const int &thread_id, const CAT &cat,
                                Neighbor *neighbor) {
  if (is_init_ == false) {
    ConstructConflictGraph(time_limiter, path_plans, &graph_);
    if (time_limiter.Timeout()) return false;
    is_init_ = true;
  }
  int a = FindMostGapAgent(path_plans, thread_id);
  if (a == -1) return false;
  if (time_limiter.Timeout()) return false;
  std::unordered_set<int> neighbors_set;
  std::unordered_set<int> closed_set;
  if (neighbors_set.size() < neighbor_size_) {
    neighbors_set.insert(a);
    std::queue<int> open;
    open.push(a);
    if (!path_plans[a].IsIdle()) neighbors_set.insert(a);
    while (!open.empty() && neighbors_set.size() < neighbor_size_) {
      int curr = open.front();
      closed_set.insert(a);
      open.pop();
      for (auto &x : graph_[a]) {
        if (neighbors_set.find(x) != neighbors_set.end()) continue;
        if (!path_plans[x].IsIdle()) neighbors_set.insert(x);
        if (neighbors_set.size() >= neighbor_size_) break;
        open.push(x);
      }
    }
  }
  neighbor->agent_ids.assign(neighbors_set.begin(), neighbors_set.end());
  return neighbors_set.size() >= 2;
}

int BaseConflictGraphSampler::FindMostGapAgent(
    const std::vector<PathPlan> &path_plans, const int &thread_id) {
  int a = -1;
  double max_gap = -1;
  auto &tabu_list = tabu_list_list_[thread_id];
  while (true) {
    for (int i = 0; i < path_plans.size(); i++) {
      // : currently we just use index to split threads
      if (i % thread_num_ != thread_id) continue;
      if (tabu_list.find(i) != tabu_list.end()) continue;
      double gap = GetGap(path_plans[i], i);
      // std::cout << std::endl;
      // std::cout << "path_plan[i].gap" << i << " " << gap << " "
      //           << path_plans[i].main_cost << " "
      //           << path_plans[i].cache_initial_heuristic << std::endl;
      if (gap < -1e-6) {
        std::cout << "Error. gap<0;"
                  << "Gap = " << gap << " " << path_plans[i].main_cost
                  << std::endl;
        exit(-1);
      }
      if (max_gap < gap) {
        a = i;
        max_gap = gap;
      }
    }
    // std::cout << "max_gap:" << max_gap << std::endl;
    if (max_gap < 1e-6) {
      if (tabu_list.empty())
        return -1;
      tabu_list.clear();
      continue;
    }
    // std::cout << "max_gap:" << max_gap << " " << a << std::endl;
    tabu_list.insert(a);
    return a;
  }
}

double BaseConflictGraphSampler::GetGap(const PathPlan &plan, const int &i) {
  if (plan.goals.size() == 0) return 0;
  int stage = plan.GetCurrentStage();
  auto &current_state = plan.path[plan.current_id];
  auto iter = main_heuristic_cache_.find(i);
  double current_heurstic;
  if (iter == main_heuristic_cache_.end()) {
    if (plan.cache_initial_heuristic >= 0)
      current_heurstic = plan.cache_initial_heuristic;
    else
      current_heurstic = cost_calculator_.GetMainHeuristic(
        stage, current_state.location, current_state.orientation,
        plan.goals, plan.acc_main_heuristic);
    main_heuristic_cache_[i] = current_heurstic;
  } else {
    current_heurstic = iter->second;
  }
  // std::cout << "current:" <<i << " " << plan.main_cost << " " << current_heurstic << std::endl;
  return plan.main_cost - current_heurstic;
}
}  // namespace lns
}  // namespace planner
