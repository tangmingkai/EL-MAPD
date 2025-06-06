#include "planner/lns/agent_sampler/random_walk_sampler.h"

#include <algorithm>
#include <set>
#include <utility>

#include "planner/multi_label_a_star/l_state.h"
#include "planner/multi_label_a_star/mlastar_node.h"
#include "planner/utils/utils.h"
namespace planner {
namespace lns {

RandomWalkSampler::RandomWalkSampler(
    const Environment &env,
    const ActionModel &action_model,
    const CostCalculator &cost_calculator,
    const int &neighbor_size, const int &thread_num,
    const double &recharge_threshold,
    const unsigned int &seed)
    : env_(env),
      action_model_(action_model),
      cost_calculator_(cost_calculator),
      neighbor_size_(neighbor_size),
      thread_num_(thread_num),
      seed_(seed) {
  tabu_list_list_.resize(thread_num_);
}

void RandomWalkSampler::Reset() {
  main_heuristic_cache_.clear();
}
void RandomWalkSampler::HaveUpdate(const Neighbor &neighbor) {
  for (auto &agent_id : neighbor.agent_ids) {
    auto iter = main_heuristic_cache_.find(agent_id);
    if (iter != main_heuristic_cache_.end()) main_heuristic_cache_.erase(iter);
  }
}
bool RandomWalkSampler::Sample(const TimeLimiter &time_limiter,
                               const std::vector<PathPlan> &path_plans,
                               const int &thread_id,
                               const CAT &cat,
                               Neighbor *neighbor) {
  if (neighbor_size_ >= path_plans.size()) {
    neighbor->agent_ids.clear();
    neighbor->agent_ids.resize(path_plans.size());
    for (int i = 0; i < path_plans.size(); i++) neighbor->agent_ids[i] = i;
    return true;
  }
  int a = FindMostGapAgent(path_plans, thread_id);
  if (a < 0) return false;

  std::unordered_set<int> neighbors_set;
  neighbors_set.insert(a);
  RandomWalk(path_plans, path_plans[a], 0, neighbor_size_, cat, &neighbors_set);
  int count = 0;
  while (neighbors_set.size() < neighbor_size_ && count < 10) {
    int start_id = rand_r(&seed_) %
                       (path_plans[a].path.size() - path_plans[a].current_id) +
                   path_plans[a].current_id;
    RandomWalk(path_plans, path_plans[a], start_id, neighbor_size_, cat,
               &neighbors_set);
    count++;
    // select the next agent randomly
    int idx = rand_r(&seed_) % neighbors_set.size();
    int i = 0;
    for (auto n : neighbors_set) {
      if (i == idx) {
        a = i;
        break;
      }
      i++;
    }
  }
  if (neighbors_set.size() < 2) return false;
  neighbor->agent_ids.assign(neighbors_set.begin(), neighbors_set.end());
  return true;
}

int RandomWalkSampler::FindMostGapAgent(const std::vector<PathPlan> &path_plans,
                                        const int &thread_id) {
  int a = -1;
  double max_gap = -1;
  auto &tabu_list = tabu_list_list_[thread_id];
  while (true) {
    for (int i = 0; i < path_plans.size(); i++) {
      // : currently we just use index to split threads
      if (i % thread_num_ != thread_id) continue;
      if (tabu_list.find(i) != tabu_list.end()) continue;
      double gap = GetGap(path_plans[i], i);
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
    if (max_gap < 1e-6) {
      if (tabu_list.empty())
        return -1;
      tabu_list.clear();
      continue;
    }
    tabu_list.insert(a);
    return a;
  }
}
double RandomWalkSampler::GetGap(const PathPlan &plan, const int &i) {
  // if (plan.goals.size() == 0) return 0;
  // int stage = plan.GetCurrentStage();
  // auto &current_state = plan.path[plan.current_id];
  // double current_heurstic = cost_calculator_.GetMainHeuristic(
  //   stage, current_state.location, current_state.orientation,
  //    plan.goals, plan.acc_main_heuristic);
  // return plan.main_cost - current_heurstic;
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
  return plan.main_cost - current_heurstic;
}

void RandomWalkSampler::RandomWalk(
    const std::vector<PathPlan> path_plans,
    const PathPlan &plan, const int &start_id, const int &neighbor_size,
    const CAT &cat, std::unordered_set<int> *conflicting_agents) const {
  auto &path = plan.path;
  double partial_path_cost = GetPartialPathCost(plan, start_id);
  auto current_l_state = LState(plan.GetStage(start_id), plan.path[start_id]);
  auto is_loaded = plan.GetIsLoaded(start_id);
  MLAStarNode current(current_l_state, partial_path_cost, 0);
  std::vector<std::pair<MLAStarNode, Action>> vaild_next_nodes;
  std::vector<int> current_conflict_agents;
  for (int t = start_id; t < plan.path.size(); ++t) {
    Utils::GetVaildNeighbors(
        env_, cost_calculator_, action_model_, current, empty_cat_, plan.goals,
        plan.acc_main_heuristic,
        plan.acc_energy_heuristic, {}, &vaild_next_nodes);
    while (!vaild_next_nodes.empty()) {
      int step = rand_r(&seed_) % vaild_next_nodes.size();
      auto iter = vaild_next_nodes.begin() + step;

      auto &next_node = iter->first;
      auto &next_state = next_node.state.state;
      auto &action = iter->second;

      double action_cost = cost_calculator_.GetActionMainCost(
          next_state.is_loaded, next_state.location, next_state.orientation,
          action);
      double next_h_val = cost_calculator_.GetMainHeuristic(
          next_node.state.label, next_state.location, next_state.orientation,
          plan.goals, plan.acc_main_heuristic);
      if (partial_path_cost + action_cost + next_h_val <
          plan.main_cost + 1e-6) {
        cat.GetConflictingAgents(current.state.state, next_state,
                                 &current_conflict_agents);
        for (auto &id : current_conflict_agents) {
          if (!path_plans[id].IsIdle()) conflicting_agents->insert(id);
        }
        current = next_node;
        partial_path_cost += action_cost;
        break;
      }
      vaild_next_nodes.erase(iter);
    }
    if (vaild_next_nodes.empty() || conflicting_agents->size() >= neighbor_size)
      break;
  }
}

double RandomWalkSampler::GetPartialPathCost(const PathPlan &plan,
                                             const int &id) const {
  double cost = 0;
  bool is_loaded = false;
  for (int i = 0; i < id; i++) {
    cost += cost_calculator_.GetActionMainCost(is_loaded, plan.path[i].location,
                                               plan.path[i].orientation,
                                               plan.actions[i]);
    if (plan.actions[i] == Action::P)
      is_loaded = true;
    else if (plan.actions[i] == Action::D)
      is_loaded = false;
  }
  return cost;
}

}  // namespace lns
}  // namespace planner
