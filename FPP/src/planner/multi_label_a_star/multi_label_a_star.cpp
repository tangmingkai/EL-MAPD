#include "planner/multi_label_a_star/multi_label_a_star.h"
#include "planner/utils/utils.h"
#include <unordered_set>
namespace planner {

MLAStar::MLAStar(const Environment &env,
                 const ActionModel &action_model,
                 const CostCalculator &cost_calculator)
    : env_(env),
      action_model_(action_model),
      cost_calculator_(cost_calculator) {}

bool MLAStar::RechargeAndFallbackPlanningLayer(
    const TimeLimiter &time_limiter, const LState &start_state, const CAT &cat,
    const std::vector<int> &goals, const AgentStatus &status,
    const std::vector<int> &ignore_agent_ids, const double &recharge_threshold,
    PathPlan *plan) const {
  plan->Clear();
  if (!cat.IsVaild(start_state.state, ignore_agent_ids)) {
    std::cout << "Why the current state is invaild" << std::endl;
    exit(-1);
    return false;
  }
  if ((start_state.state.is_loaded && start_state.label !=1) ||
      (!start_state.state.is_loaded && start_state.label ==1)) {
    std::cout << "Why is_loaded and label mismatch: "
              << start_state.state.is_loaded << " " << start_state.label
              << std::endl;
    exit(-1);
  }
  cost_calculator_.CalculateAccHeuristicByType(CostType::Main, goals,
                                               &plan->acc_main_heuristic);
  cost_calculator_.CalculateAccHeuristicByType(CostType::Energy, goals,
                                               &plan->acc_energy_heuristic);
  std::vector<State> charge_path;
  std::vector<Action> charge_actions;
  auto current_state = start_state;
  double minimum_energy_comsumption =
      cost_calculator_.GetMinimumEneryComsumption(
          0, current_state.state.location, current_state.state.orientation,
          goals, plan->acc_energy_heuristic);
  int max_constrainted_time_step = cat.GetMaxConstraintedTimeStep();
  plan->is_fallback = false;
  if (!env_.charges[current_state.state.location]) {
    if (current_state.state.energy + 1e-6 < minimum_energy_comsumption) {
      return false;
    }
  } else {
    double require_energy =
        std::max(minimum_energy_comsumption + 1e-6, recharge_threshold);
    if (current_state.state.energy + 1e-6 < require_energy) {
      int charge_num = static_cast<int>(
          std::ceil((require_energy - current_state.state.energy) /
                    action_model_.charge_energy_per_timestep()));
      for (int i = 0; i < charge_num; i++) {
        charge_path.emplace_back(current_state.state);
        charge_actions.emplace_back(Action::E);
        current_state.state =
          action_model_.ResultState(current_state.state, Action::E);
        if (!cat.IsVaild(current_state.state, ignore_agent_ids)) {
          return false;
        }
      }
    }
  }
  while (true) {
    bool res = false;
    if (current_state.state.energy > minimum_energy_comsumption + 1e-6)
      res = EnergyConstraintedPathPlanningLayer(
          time_limiter, current_state, cat, goals, ignore_agent_ids, plan);
    if (res == false &&
        action_model_.full_energy() - current_state.state.energy < 1e-6 &&
        current_state.state.timestep >= max_constrainted_time_step &&
        env_.charges[current_state.state.location] &&
        current_state.label == 0 && goals.size() == 3) {
      // Fallback Path
      Utils::ConstructOptimalPathAndAction(
          cost_calculator_, action_model_, current_state, goals,
          cost_calculator_.unloaded_energy_heuristic_table(),
          cost_calculator_.loaded_energy_heuristic_table(), true, plan);
      res = true;
      plan->is_fallback = true;
    }
    if (res == true) {
      plan->status = status;
      plan->goals = goals;
      plan->current_id = 0;
      plan->path.insert(plan->path.begin(), charge_path.begin(),
                        charge_path.end());
      plan->charge_point_id = env_.loc2charge_id[plan->path.back().location];
      plan->actions.insert(plan->actions.begin(), charge_actions.begin(),
                           charge_actions.end());
      plan->break_points[0] = plan->break_points[1] = -1;
      for (int i = 0; i < plan->actions.size(); i++) {
        if (plan->actions[i] == Action::P)
          plan->break_points[0] = i;
        else if (plan->actions[i] == Action::D)
          plan->break_points[1] = i;
      }
      plan->main_cost = cost_calculator_.GetRemainMainCost(*plan);
      if (!env_.charges[current_state.state.location] ||
          start_state.label != 0) {
        plan->cache_initial_heuristic = -1;
      } else {
        if (plan->is_fallback) {
          plan->cache_initial_heuristic = plan->main_cost;
        } else {
          double charge_cost =
              charge_actions.size() *
              cost_calculator_.GetActionMainCost(
                  start_state.state.is_loaded, start_state.state.location,
                  start_state.state.orientation, Action::E);
          double remain_heuristic = cost_calculator_.GetMainHeuristic(
              start_state.label, start_state.state.location,
              start_state.state.orientation, goals, plan->acc_main_heuristic);
          plan->cache_initial_heuristic = charge_cost + remain_heuristic;
        }
      }
      int stage = plan->GetCurrentStage();
      auto &current_state = plan->path[plan->current_id];
      double current_heurstic;
      current_heurstic = cost_calculator_.GetMainHeuristic(
          stage, current_state.location, current_state.orientation,
          plan->goals, plan->acc_main_heuristic);
      return true;
    }
    if (!env_.charges[current_state.state.location]) {
      break;
    }
    if (time_limiter.Timeout()) return false;
    charge_path.emplace_back(current_state.state);
    charge_actions.emplace_back(Action::E);
    current_state.state =
        action_model_.ResultState(current_state.state, Action::E);
    if (!cat.IsVaild(current_state.state, ignore_agent_ids)) {
      return false;
    }
  }
  return false;
}

bool MLAStar::EnergyConstraintedPathPlanningLayer(
    const TimeLimiter &time_limiter, const LState &start_state, const CAT &cat,
    const std::vector<int> &goals, const std::vector<int> &ignore_agent_ids,
    PathPlan *plan) const {
  auto now_state = start_state;
  using OpenList = boost::heap::d_ary_heap<MLAStarNode, boost::heap::arity<2>,
                                           boost::heap::mutable_<true>>;
  OpenList open_list;
  std::unordered_map<LState, OpenList::handle_type, std::hash<LState>>
      state_to_heap;
  std::unordered_set<LState, std::hash<LState>> closed_set;
  std::unordered_map<LState, std::pair<LState, Action>, std::hash<LState>>
      came_from;
  auto handle =
      open_list.push(MLAStarNode(now_state, 0, 0));
  state_to_heap.insert(std::make_pair(now_state, handle));
  std::vector<std::pair<MLAStarNode, Action>> vaild_next_nodes;
  vaild_next_nodes.reserve(10);
  while (!open_list.empty()) {
    auto current = open_list.top();
    if (current.state.state.location == goals.back() &&
        current.state.label == goals.size() - 1) {
      auto iter = came_from.find(current.state);
      plan->path.clear();
      plan->actions.clear();
      plan->path.push_back(current.state.state);
      while (iter != came_from.end()) {
        plan->path.push_back(iter->second.first.state);
        plan->actions.push_back(iter->second.second);
        iter = came_from.find(iter->second.first);
      }
      std::reverse(plan->path.begin(), plan->path.end());
      std::reverse(plan->actions.begin(), plan->actions.end());
      return true;
    }
    if (time_limiter.Timeout()) return false;
    open_list.pop();
    state_to_heap.erase(current.state);
    closed_set.insert(current.state);

    Utils::GetVaildNeighbors(env_, cost_calculator_, action_model_, current,
                             cat, goals, plan->acc_main_heuristic,
                             plan->acc_energy_heuristic, ignore_agent_ids,
                             &vaild_next_nodes);

    for (auto &neighbor : vaild_next_nodes) {
      auto &next_node = neighbor.first;
      auto &action = neighbor.second;
      if (closed_set.find(next_node.state) == closed_set.end()) {
        auto iter = state_to_heap.find(next_node.state);
        if (iter == state_to_heap.end()) {
          auto handle = open_list.push(next_node);
          state_to_heap[next_node.state] = handle;
        } else {
          auto handle = iter->second;
          if ((next_node.f > (*handle).f) ||
              ((next_node.f == (*handle).f) &&
               (next_node.state.state.energy >= (*handle).state.state.energy)))
            continue;
          (*handle).g = next_node.g;
          (*handle).f = next_node.f;
          (*handle).state = next_node.state;
          open_list.update(handle);
        }
        came_from[next_node.state] = {current.state, action};
      }
    }
  }
  return false;
}
};  // namespace planner
