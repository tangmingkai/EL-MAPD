#include "planner/cost_calculator/cost_calculator.h"
#include "planner/utils/utils.h"
#include <algorithm>

namespace planner {

CostCalculator::CostCalculator(const Environment &env, const uint &seed)
    : env_(env), seed_(seed) {}
void CostCalculator::Init(const double &energy_weight,
                          const ActionModel &action_model,
                          const GuidanceMap &guidance_map,
                          const HeuristicTable &unloaded_main_heuristic_table,
                          const HeuristicTable &loaded_main_heuristic_table,
                          const HeuristicTable &unloaded_energy_heuristic_table,
                          const HeuristicTable &loaded_energy_heuristic_table,
                          const ChargeDistanceTable &charge_distance_table) {
  energy_weight_ = energy_weight;
  action_model_ = &action_model;
  guidance_map_ = &guidance_map;
  unloaded_main_heuristic_table_ = &unloaded_main_heuristic_table;
  loaded_main_heuristic_table_ = &loaded_main_heuristic_table;
  unloaded_energy_heuristic_table_ = &unloaded_energy_heuristic_table;
  loaded_energy_heuristic_table_ = &loaded_energy_heuristic_table;
  charge_distance_table_ = &charge_distance_table;
}
double CostCalculator::GetActionCostByType(const CostType &cost_type,
                                           const bool &is_loaded,
                                           const int &location,
                                           const int &orientation,
                                           const Action &action) const {
  if (cost_type == CostType::Main) {
    return GetActionMainCost(is_loaded, location, orientation, action);
  } else if (cost_type == CostType::Energy) {
    return GetActionPureEnergy(is_loaded, location, orientation, action);
  } else {
    std::cout << "GetActionCostByType not implement" << std::endl;
    exit(-1);
  }
}
double CostCalculator::GetActionMainCost(const bool &is_loaded,
                                         const int &location,
                                         const int &orientation,
                                         const Action &action) const {
  return Utils::GetActionMainCost(is_loaded, location, orientation,
                                            action, *action_model_,
                                            *guidance_map_, energy_weight_);
}

double CostCalculator::GetActionPureEnergy(const bool &is_loaded,
                                           const int &location,
                                           const int &orientation,
                                           const Action &action) const {
  return Utils::GetActionPureEnergy(is_loaded, location, orientation,
                                            action, *action_model_);
}

void CostCalculator::CalculateAccHeuristicByType(
    const CostType &cost_type, const std::vector<int> &goals,
    std::vector<double> *acc_heuristic) const {
  if (cost_type == CostType::Main) {
    CalculateAccHeursitic(cost_type, goals, *unloaded_main_heuristic_table_,
                          *loaded_main_heuristic_table_, acc_heuristic);
  } else if (cost_type == CostType::Energy) {
    CalculateAccHeursitic(cost_type, goals, *unloaded_energy_heuristic_table_,
                          *loaded_energy_heuristic_table_, acc_heuristic);
  } else {
    std::cout << "CalculateAccHeuristicByType not implement" << std::endl;
    exit(-1);
  }
}

void CostCalculator::CalculateAccHeursitic(
    const CostType &cost_type, const std::vector<int> &goals,
    const HeuristicTable &unloaded_heuristic_table,
    const HeuristicTable &loaded_heuristic_table,
    std::vector<double> *acc_heuristic) const {
  acc_heuristic->clear();
  acc_heuristic->resize(goals.size() * 4, 0);
  for (int i = static_cast<int>(goals.size()) - 2; i >= 0; i--) {
    for (int j = 0; j < 4; j++) {
      (*acc_heuristic)[i * 4 + j] = MAX_HEURISTIC;
      for (int k = 0; k < 4; k++) {
        int current_segment_heuristic = 0;
        bool is_loaded = (i == static_cast<int>(goals.size()) - 3);
        Action current_switch_action;
        if (is_loaded) {
          current_segment_heuristic =
              loaded_heuristic_table.Get(goals[i], j, goals[i + 1], k);
          current_switch_action = Action::P;
        } else {
          current_segment_heuristic =
              unloaded_heuristic_table.Get(goals[i], j, goals[i + 1], k);
          current_switch_action = Action::D;
        }
        double switch_cost = GetActionCostByType(cost_type, is_loaded, goals[i],
                                                 j, current_switch_action);
        (*acc_heuristic)[i * 4 + j] =
            std::min((*acc_heuristic)[i * 4 + j],
                     (*acc_heuristic)[(i + 1) * 4 + k] +
                         current_segment_heuristic + switch_cost);
      }
    }
  }
}
double CostCalculator::GetUnloadedMainHeuristic(const int &loc1,
                                                const int &loc2) const {
  return unloaded_main_heuristic_table_->Get(loc1, loc2);
}
double CostCalculator::GetMainHeuristic(
    int stage, const int &location, const int &orientation,
    const std::vector<int> &goals,
    const std::vector<double> &acc_main_heuristic) const {
  if (stage >= goals.size()) return 0;
  double res = MAX_HEURISTIC;
  for (int current_goal_orient = 0; current_goal_orient < 4;
       current_goal_orient++) {
    double current_segment_heuristic = 0;
    bool is_loaded = stage == 1;
    if (is_loaded)
      current_segment_heuristic = loaded_main_heuristic_table_->Get(
          location, orientation, goals[stage], current_goal_orient);
    else
      current_segment_heuristic = unloaded_main_heuristic_table_->Get(
          location, orientation, goals[stage], current_goal_orient);
    res = std::min(res, acc_main_heuristic[stage * 4 + current_goal_orient] +
                            current_segment_heuristic);
  }
  // std::cout << "MainHeuristic:" << stage << " " << location << " "
  //           << orientation << " -> " << res << std::endl;
  return res;
}

double CostCalculator::GetMinimumEneryComsumption(
    int stage, const int &location, const int &orientation,
    const std::vector<int> &goals,
    const std::vector<double> &acc_energy_heuristic) const {
  if (stage >= goals.size()) return 0;
  double res = MAX_HEURISTIC;
  for (int current_goal_orient = 0; current_goal_orient < 4;
       current_goal_orient++) {
    double current_segment_heuristic = 0;
    bool is_loaded = stage == 1;
    if (is_loaded)
      current_segment_heuristic = loaded_energy_heuristic_table_->Get(
          location, orientation, goals[stage], current_goal_orient);
    else
      current_segment_heuristic = unloaded_energy_heuristic_table_->Get(
          location, orientation, goals[stage], current_goal_orient);
    res = std::min(res, acc_energy_heuristic[stage * 4 + current_goal_orient] +
                            current_segment_heuristic);
  }
  return res;
}
double CostCalculator::GetRemainMainCost(const PathPlan &plan) const {
  double res = 0;
  for (int i = plan.current_id; i < plan.actions.size(); i++) {
    // std::cout << plan.path[i].is_loaded
    //           << env_.Str(plan.path[i].location, plan.path[i].orientation)
    //           << " " << plan.actions[i]
    //           << GetActionMainCost(plan.path[i].is_loaded,
    //                                plan.path[i].location,
    //                                plan.path[i].orientation, plan.actions[i])
    //           << std::endl;
    res += GetActionMainCost(plan.path[i].is_loaded, plan.path[i].location,
                             plan.path[i].orientation, plan.actions[i]);
  }
  return res;
}

int CostCalculator::SelectChargeID(
    const std::vector<int> &charge_point_allocated_agent_id,
    const int &current_agent_id, const int &loc) const {
  return SelectChargeID(charge_point_allocated_agent_id,
                                       current_agent_id,
                                       charge_distance_table_->Get(loc));
}
int CostCalculator::SelectChargeID(
    const std::vector<int> &charge_point_allocated_agent_id,
    const int &current_agent_id, const int &loc, const int &orient) const {
  return SelectChargeID(
      charge_point_allocated_agent_id, current_agent_id,
      charge_distance_table_->Get(loc, orient));
}

std::vector<int>  CostCalculator::GetAllMinCostChargeID(
    const std::vector<int> &charge_point_allocated_agent_id,
    const int &current_agent_id, const int &loc) const {
  return GetAllMinCostChargeID(charge_point_allocated_agent_id,
                                       current_agent_id,
                                       charge_distance_table_->Get(loc));
}
std::vector<int>  CostCalculator::GetAllMinCostChargeID(
    const std::vector<int> &charge_point_allocated_agent_id,
    const int &current_agent_id, const int &loc, const int &orient) const {
  return GetAllMinCostChargeID(
      charge_point_allocated_agent_id, current_agent_id,
      charge_distance_table_->Get(loc, orient));
}

int CostCalculator::SelectChargeID(
    const std::vector<int> &charge_point_allocated_agent_id,
    const int &current_agent_id,
    const std::vector<std::pair<double, int>> &distance_table) const {
  auto current_ids = GetAllMinCostChargeID(
      charge_point_allocated_agent_id, current_agent_id, distance_table);
  if (!current_ids.empty()) {
    int id = rand_r(&seed_) % current_ids.size();
    return current_ids[id];
  }
  return -1;
}

std::vector<int> CostCalculator::GetAllMinCostChargeID(
    const std::vector<int> &charge_point_allocated_agent_id,
    const int &current_agent_id,
    const std::vector<std::pair<double, int>> &distance_table) const {
  std::vector<int> current_ids;
  for (int i = 0; i < distance_table.size(); i++) {
    int charge_id = distance_table[i].second;
    if (charge_point_allocated_agent_id[charge_id] == -1 ||
        charge_point_allocated_agent_id[charge_id] == current_agent_id) {
      current_ids.emplace_back(charge_id);
    }
    if (i + 1 < distance_table.size() &&
        distance_table[i].first + 1e-6 < distance_table[i + 1].first &&
        !current_ids.empty()) {
      break;
    }
  }
  return current_ids;
}

}  // namespace planner
