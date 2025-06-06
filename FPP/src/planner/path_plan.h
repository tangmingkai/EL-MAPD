#pragma once
#include <vector>
#include <algorithm>
#include <tuple>
#include "common/state.h"
#include "planner/multi_label_a_star/l_state.h"
#include "model/action.h"
namespace planner {
enum class AgentStatus {
  // Has a task but does not have enough energy. Must go to charge
  ToCharge = 0,
  // Can perform a task
  OnTask = 1,
};
struct PathPlan {
  PathPlan() {Clear();}
  void Clear() {
    status = AgentStatus::OnTask;
    current_id = 0;
    break_points[0] = break_points[1] = -1;
    charge_point_id = -1;
    path.clear();
    actions.clear();
    goals.clear();
    acc_main_heuristic.clear();
    acc_energy_heuristic.clear();
    main_cost = 0;
    cache_initial_heuristic = -1;
    is_fallback = false;
  }
  int GetCurrentStage() const {
    return GetStage(current_id);
  }
  int GetStage(int id) const {
    if (id > break_points[1])
      return 2;
    else if (id > break_points[0])
      return 1;
    else
      return 0;
  }
  int GetIsLoaded(int id) const {
    return GetStage(id) == 1;
  }
  bool GetCurrentIsLoaded() const {
    return GetCurrentStage() == 1;
  }
  LState GetCurrentLState() const {
    return LState(GetCurrentStage(), path[current_id]);
  }
  bool IsIdle() const { return goals.size() == 0; }
  bool IsReachFinalState() const {
    return current_id + 1 >= path.size();
  }
  bool IsPassedDelivery() const { return current_id > break_points[1]; }
  int DeliveryLoc() const { return goals[1];}
  int PickupLoc() const { return goals[0];}

  std::tuple<int, int, int> goal_tuple() const {
    if (GetCurrentStage() == 2) return {goals[2], -1, -1};
    if (GetCurrentStage() == 1) return {goals[1], goals[2], -1};
    return {goals[0], goals[1], goals[2]};
  }

  AgentStatus status;
  int current_id;

  // paths[breakpoint[0]] -> pickup
  // paths[breakpoint[1]] -> delivery
  int break_points[2];
  int charge_point_id;
  std::vector<State> path;
  std::vector<Action> actions;
  std::vector<int> goals;

  // cost for lns;
  double main_cost;
  std::vector<double> acc_main_heuristic;
  std::vector<double> acc_energy_heuristic;
  double cache_initial_heuristic;
  bool is_fallback;
};
};  // namespace planner
