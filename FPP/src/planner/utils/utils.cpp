#include "planner/utils/utils.h"

namespace planner {

void Utils::GetVaildNeighbors(
    const Environment &env,
    const CostCalculator &cost_calculator,
    const ActionModel &action_model,
    const MLAStarNode &current, const CAT &cat, const std::vector<int> &goals,
    const std::vector<double> &acc_main_heuristic,
    const std::vector<double> &acc_energy_heuristic,
    const std::vector<int> &ignore_agent_ids,
    std::vector<std::pair<MLAStarNode, Action>> *vaild_next_nodes) {
  vaild_next_nodes->clear();
  if (current.state.state.location == goals[current.state.label]) {
    int next_label = current.state.label + 1;
    auto action = current.state.state.is_loaded ? Action::D : Action::P;
    auto next_state = action_model.ResultState(current.state.state, action);
    auto next_l_state = LState(next_label, next_state);
    double minimum_energy_comsumption =
        cost_calculator.GetMinimumEneryComsumption(
            current.state.label, current.state.state.location,
            current.state.state.orientation, goals, acc_energy_heuristic);
    if (next_state.energy > minimum_energy_comsumption + 1e-6
       && cat.IsVaild(current.state.state, next_state, ignore_agent_ids)) {
      double action_cost = cost_calculator.GetActionMainCost(
                          current.state.state.is_loaded,
                          current.state.state.location,
                          current.state.state.orientation, action);
      double heuristic = cost_calculator.GetMainHeuristic(
          next_l_state.label, next_l_state.state.location,
          next_l_state.state.orientation, goals,
          acc_main_heuristic);
      double next_g = current.g + action_cost;
      double next_f = next_g + heuristic;
      vaild_next_nodes->push_back(
          {MLAStarNode(next_l_state, next_g, next_f), action});
    }
  }

  auto &normal_actions = action_model.normal_actions();
  for (auto &action : normal_actions) {
    auto next_state = action_model.ResultState(current.state.state, action);
    if (next_state.timestep == -1) continue;
    if (!cat.IsVaild(current.state.state, next_state, ignore_agent_ids))
      continue;
    if (action == Action::FW &&
        next_state.location != goals[current.state.label] &&
        env.charges[next_state.location])
      continue;
    auto next_l_state = LState(current.state.label, next_state);
    double minimum_energy_comsumption =
        cost_calculator.GetMinimumEneryComsumption(
            next_l_state.label, next_l_state.state.location,
            next_l_state.state.orientation, goals, acc_energy_heuristic);
    if (next_state.energy + 1e-6 <= minimum_energy_comsumption)
      continue;
    double action_cost = cost_calculator.GetActionMainCost(
                          current.state.state.is_loaded,
                          current.state.state.location,
                          current.state.state.orientation, action);
    double heuristic = cost_calculator.GetMainHeuristic(
          next_l_state.label, next_l_state.state.location,
          next_l_state.state.orientation, goals,
          acc_main_heuristic);
    double next_g = current.g + action_cost;
    double next_f = next_g + heuristic;
    vaild_next_nodes->push_back(
        {MLAStarNode(next_l_state, next_g, next_f), action});
  }
}

void Utils::DeleteOldPathPlan(
    const PathPlan& old_plan, const int& agent_id,
    CAT* cat, std::vector<int>* charge_point_allocated_agent_id) {
  if (old_plan.path.empty()) return;
  for (int i = old_plan.current_id; i + 1 < old_plan.path.size(); i++) {
    cat->Delete(agent_id, old_plan.path[i]);
    cat->Delete(agent_id, old_plan.path[i], old_plan.path[i + 1]);
  }
  cat->Delete(agent_id, old_plan.path.back());
  cat->DeleteTimeStep(old_plan.path.back().timestep);
  if ((*charge_point_allocated_agent_id)[old_plan.charge_point_id] == -1) {
    std::cout << "Release empty charge point" << std::endl;
    exit(-1);
  }
  (*charge_point_allocated_agent_id)[old_plan.charge_point_id] = -1;
}

void Utils::AddNewPathPlan(const PathPlan& new_plan,
                                 const int& agent_id,
    CAT* cat, std::vector<int>* charge_point_allocated_agent_id) {
  if (new_plan.path.empty()) return;
  for (int i = new_plan.current_id; i + 1 < new_plan.path.size(); i++) {
    cat->Add(agent_id, new_plan.path[i]);
    cat->Add(agent_id, new_plan.path[i], new_plan.path[i + 1]);
  }
  cat->Add(agent_id, new_plan.path.back());
  cat->AddTimeStep(new_plan.path.back().timestep);
  if ((*charge_point_allocated_agent_id)[new_plan.charge_point_id] != -1) {
    std::cout << "Conflict at charge point" << std::endl;
    exit(-1);
  }
  (*charge_point_allocated_agent_id)[new_plan.charge_point_id] = agent_id;
}

void Utils::ConstructPureRechargePlan(const State &current_state,
                                      const int &charge_point_id,
                                      const ActionModel &action_model,
                                      PathPlan *plan) {
  plan->status = AgentStatus::ToCharge;
  plan->goals.clear();
  plan->path.clear();
  plan->path.push_back(current_state);
  plan->path.push_back(action_model.ResultState(current_state, Action::E));
  plan->actions.clear();
  plan->actions.push_back(Action::E);
  plan->break_points[0] = -1;
  plan->break_points[1] = -1;
  plan->goals = std::vector<int>{-1, -1, current_state.location};
  plan->current_id = 0;
  plan->charge_point_id = charge_point_id;
}

double Utils::GetActionCostByType(const CostType &cost_type,
                                  const bool &is_loaded, const int &location,
                                  const int &orientation, const Action &action,
                                  const ActionModel &action_model,
                                  const GuidanceMap &guidance_map,
                                  const double &energy_weight) {
  if (cost_type == CostType::Main) {
    return GetActionMainCost(is_loaded, location, orientation, action,
                             action_model, guidance_map, energy_weight);
  } else if (cost_type == CostType::Energy) {
    return GetActionPureEnergy(is_loaded, location, orientation, action,
                               action_model);
  } else {
    std::cout << "GetActionCostByType not implement" << std::endl;
    exit(-1);
  }
}
double Utils::GetActionMainCost(const bool &is_loaded, const int &location,
                                const int &orientation, const Action &action,
                                const ActionModel &action_model,
                                const GuidanceMap &guidance_map,
                                const double &energy_weight) {
  double energy = action_model.PureEnergyComsumption(is_loaded, action);
  double weight = guidance_map.GetWeight(location, orientation, action);
  return weight *
         (energy / action_model.active_unloaded_comsumption() * energy_weight +
          1 * (1 - energy_weight));
}

double Utils::GetActionPureEnergy(const bool &is_loaded, const int &location,
                                  const int &orientation, const Action &action,
                                  const ActionModel &action_model) {
  double energy = action_model.PureEnergyComsumption(is_loaded, action);
  return energy;
}

void Utils::ConstructOptimalPathAndAction(
    const CostCalculator &cost_calculator, const ActionModel &action_model,
    const LState &start_state, const std::vector<int> &goals,
    const HeuristicTable &unloaded_heuristic_table,
    const HeuristicTable &loaded_heuristic_table,
    const bool &enable_charge, PathPlan *plan) {
  double min_cost = MAX_HEURISTIC;
  int min_charge1_orient = -1, min_charge2_orient = -1, min_pickup_orient = -1,
      min_delivery_orient = -1;
  int charge1_orient_start = 0;
  int charge1_orient_end = 4;
  int pickup_orient_start = 0;
  int pickup_orient_end = 4;
  int delivery_orient_state = 0;
  int delivery_orient_end = 4;

  if (enable_charge == false) {
    charge1_orient_start = start_state.state.orientation;
    charge1_orient_end = start_state.state.orientation+1;
  }
  if (start_state.label >= 1) {
    pickup_orient_end = 1;
  }
  if (start_state.label >= 2) {
    delivery_orient_end = 1;
  }
  for (int charge1_orient = charge1_orient_start;
       charge1_orient < charge1_orient_end; charge1_orient++) {
    for (int pickup_orient = 0;
        pickup_orient < pickup_orient_end; pickup_orient++) {
      double cost1 = start_state.label >= 1 ? 0
                                            : unloaded_heuristic_table.Get(
                                                  start_state.state.location,
                                                  charge1_orient,
                                                  goals[0], pickup_orient);
      if (cost1 >= MAX_HEURISTIC) continue;
      for (int delivery_orient = 0;
          delivery_orient < delivery_orient_end; delivery_orient++) {
        double cost2 = start_state.label >= 2 ? 0
                                              : loaded_heuristic_table.Get(
                                                    goals[0], pickup_orient,
                                                    goals[1], delivery_orient);
        if (cost2 >= MAX_HEURISTIC) continue;
        for (int charge2_orient = 0; charge2_orient < 4; charge2_orient++) {
          double cost3 = unloaded_heuristic_table.Get(
              goals[1], delivery_orient, start_state.state.location,
              charge2_orient);
          if (cost3 >= MAX_HEURISTIC) continue;
          if (min_cost > cost1 + cost2 + cost3) {
            min_cost = cost1 + cost2 + cost3;
            min_charge1_orient = charge1_orient;
            min_charge2_orient = charge2_orient;
            min_pickup_orient = pickup_orient;
            min_delivery_orient = delivery_orient;
          }
        }
      }
    }
  }
  plan->actions.clear();
  plan->path.clear();
  auto current_state = start_state.state;
  if (enable_charge) {
    AppendOptimalPathAndAction(current_state.location,
                               current_state.orientation,
                               current_state.location, min_charge1_orient,
                               unloaded_heuristic_table, action_model, plan);
    plan->path[0].energy = start_state.state.energy;
    for (int i = 0; i < plan->actions.size(); i++) {
      plan->path[i + 1].energy =
          action_model.NextEnergy(plan->path[i].location, false,
                                  plan->path[i].energy, plan->actions[i]);
    }
    while (plan->path.back().energy + 1e-6 < action_model.full_energy()) {
      plan->actions.emplace_back(Action::E);
      plan->path.emplace_back(
          action_model.ResultState(plan->path.back(), Action::E));
    }
    current_state = plan->path.back();
    plan->path.pop_back();
  }
  if (start_state.label <= 0) {
    AppendOptimalPathAndAction(
        current_state.location, current_state.orientation, goals[0],
        min_pickup_orient, unloaded_heuristic_table, action_model, plan);
    current_state = plan->path.back();
    plan->actions.emplace_back(Action::P);
  }
  if (start_state.label <= 1) {
    AppendOptimalPathAndAction(
        current_state.location, current_state.orientation, goals[1],
        min_delivery_orient, loaded_heuristic_table, action_model, plan);
    current_state = plan->path.back();
    plan->actions.emplace_back(Action::D);
  }
  AppendOptimalPathAndAction(current_state.location, current_state.orientation,
                             start_state.state.location, min_charge2_orient,
                             unloaded_heuristic_table, action_model, plan);
  bool is_loaded = start_state.state.is_loaded;
  plan->path[0] = start_state.state;
  for (int i = 0; i < plan->actions.size(); i++) {
    if (plan->actions[i] == Action::P) {
      is_loaded = true;
    } else if (plan->actions[i] == Action::D) {
      is_loaded = false;
    }
    plan->path[i + 1].energy =
        action_model.NextEnergy(plan->path[i].location, is_loaded,
                                 plan->path[i].energy, plan->actions[i]);
    plan->path[i + 1].is_loaded = is_loaded;
    plan->path[i + 1].timestep = plan->path[i].timestep + 1;
  }
}
void Utils::AppendOptimalPathAndAction(const int &start_loc,
                                       const int &start_ori, const int &end_loc,
                                       const int &end_ori,
                                       const HeuristicTable &heuristic_table,
                                       const ActionModel &action_model,
                                       PathPlan *plan) {
  std::vector<State> path;
  std::vector<Action> actions;
  int current_loc = end_loc;
  int current_ori = end_ori;
  while (current_loc != -1) {
    State state;
    state.location = current_loc;
    state.orientation = current_ori;
    path.emplace_back(state);
    std::tie(current_loc, current_ori) =
        heuristic_table.GetFrom(start_loc, start_ori, current_loc, current_ori);
  }
  std::reverse(path.begin(), path.end());
  plan->path.insert(plan->path.end(), path.begin(), path.end());
  for (int i = 1; i < path.size(); i++) {
    plan->actions.emplace_back(
        action_model.GetNormalAction(path[i - 1], path[i]));
  }
}

}  // namespace planner
