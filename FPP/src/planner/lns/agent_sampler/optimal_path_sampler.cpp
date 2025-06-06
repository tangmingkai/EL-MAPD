#include "planner/lns/agent_sampler/optimal_path_sampler.h"

#include "planner/utils/utils.h"
#include <queue>
#include <algorithm>
namespace planner {
namespace lns {

OptimalPathSampler::OptimalPathSampler(
    const Environment &env, const ActionModel &action_model,
    const CostCalculator &cost_calculator, const int &neighbor_size,
    const int &thread_num, const double &recharge_threshold, const uint &seed)
    : BaseConflictGraphSampler(env, action_model, cost_calculator,
                               neighbor_size, thread_num, recharge_threshold,
                               seed),
      recharge_threshold_(recharge_threshold) {
  empty_charge_point_allocated_agent_id_.resize(env.charges_locs.size(), -1);
}

void OptimalPathSampler::ConstructConflictGraph(
    const TimeLimiter &time_limiter,
    const std::vector<PathPlan> &path_plans,
    std::vector<std::vector<int>> *graph) {
  std::vector<std::unordered_set<int>> g(path_plans.size());
  std::vector<PathPlan> optimal_path_plans(path_plans.size());
  for (int i = 0; i < path_plans.size(); i++) {
    auto &path_plan = path_plans[i];
    auto &optimal_path_plan = optimal_path_plans[i];
    auto l_state = path_plan.GetCurrentLState();
    auto goals = path_plan.goals;
    int charge_point_id = -1;
    if (path_plan.IsPassedDelivery())
      charge_point_id = cost_calculator_.SelectChargeID(
          empty_charge_point_allocated_agent_id_, i, l_state.state.location,
          l_state.state.orientation);
    else
      charge_point_id = cost_calculator_.SelectChargeID(
          empty_charge_point_allocated_agent_id_, i, path_plan.DeliveryLoc());
    goals[2] = env_.charges_locs[charge_point_id];
    Utils::ConstructOptimalPathAndAction(
        cost_calculator_, action_model_, path_plans[i].GetCurrentLState(),
        goals, cost_calculator_.unloaded_main_heuristic_table(),
        cost_calculator_.loaded_main_heuristic_table(), false,
        &(optimal_path_plan));

    // for (int i = 0; i < optimal_path_plan.actions.size(); i++) {
    //   auto current_state = optimal_path_plan.path[i];
    //   current_state.energy = action_model_.full_energy();
    //   auto next_state = action_model_.ResultState(current_state,
    //                                               optimal_path_plan.actions[i]);
    //   if (next_state.location != optimal_path_plan.path[i + 1].location ||
    //       next_state.orientation != optimal_path_plan.path[i + 1].orientation) {
    //     for (int i = 0; i < optimal_path_plan.path.size(); i++) {
    //       std::cout << optimal_path_plan.path[i].timestep << " "
    //                 << env_.Str(optimal_path_plan.path[i].location,
    //                             optimal_path_plan.path[i].orientation)
    //                 << " " << optimal_path_plan.path[i].energy << " ";
    //       if (i < optimal_path_plan.actions.size())
    //         std::cout << optimal_path_plan.actions[i];
    //       std::cout << std::endl;
    //     }
    //     std::cout << std::endl;
    //     std::cout << env_.Str(next_state.location,
    //                             next_state.orientation) << std::endl;
    //     std::cout << "path and action is mismatch:"
    //               << "; timestep: " << optimal_path_plan.path[i].timestep
    //               << std::endl;
    //     exit(-1);
    //   }
    // }
    // std::cout << "optimal path1" << std::endl;
    // for (int i = 0; i < optimal_path_plan.path.size(); i++) {
    //   std::cout << optimal_path_plan.path[i].timestep << " "
    //             << env_.Str(optimal_path_plan.path[i].location,
    //                         optimal_path_plan.path[i].orientation)
    //             << " " << optimal_path_plan.path[i].energy << " ";
    //   if (i < optimal_path_plan.actions.size())
    //     std::cout << optimal_path_plan.actions[i];
    //   std::cout << std::endl;
    // }
    // std::cout << std::endl;
    if (env_.charges[l_state.state.location]) {
      double energy_requirement = 0;
      for (int i = 0; i < optimal_path_plan.path.size(); i++) {
        energy_requirement += action_model_.PureEnergyComsumption(
            optimal_path_plan.path[i].is_loaded, optimal_path_plan.actions[i]);
      }
      energy_requirement = std::max(energy_requirement, recharge_threshold_);
      if (energy_requirement < action_model_.full_energy()) {
        auto dummy_start_state = l_state.state;
        std::vector<Action> charge_actions;
        while (dummy_start_state.energy + 1e-6 < energy_requirement) {
          dummy_start_state =
                action_model_.ResultState(dummy_start_state, Action::E);
          charge_actions.emplace_back(Action::E);
          if (dummy_start_state.energy + 1e-6 >= action_model_.full_energy())
            break;
        }

        optimal_path_plan.actions.insert(optimal_path_plan.actions.begin(),
                                         charge_actions.begin(),
                                         charge_actions.end());
        optimal_path_plan.path.resize(optimal_path_plan.actions.size() + 1);
        optimal_path_plan.path[0] = l_state.state;
        for (int i = 0; i < optimal_path_plan.actions.size(); i++) {
          optimal_path_plan.path[i + 1] = action_model_.ResultState(
              optimal_path_plan.path[i], optimal_path_plan.actions[i]);
        }
        // std::cout << "optimal path2" << std::endl;
        // for (int i = 0; i < optimal_path_plan.path.size(); i++) {
        //   std::cout << optimal_path_plan.path[i].timestep << " "
        //             << env_.Str(optimal_path_plan.path[i].location,
        //                         optimal_path_plan.path[i].orientation)
        //             << " " << optimal_path_plan.path[i].energy << " ";
        //   if (i < optimal_path_plan.actions.size())
        //     std::cout << optimal_path_plan.actions[i];
        //   std::cout << std::endl;
        // }
        // std::cout << std::endl;
      } else {
        Utils::ConstructOptimalPathAndAction(
            cost_calculator_, action_model_, l_state, goals,
            cost_calculator_.unloaded_energy_heuristic_table(),
            cost_calculator_.loaded_energy_heuristic_table(), true,
            &optimal_path_plan);
      }
    }
  }
  if (time_limiter.Timeout()) return;
  int current_id = 0;
  int next_loc;
  int current_loc;
  bool has_not_finished = true;
  while (has_not_finished) {
    if (time_limiter.Timeout()) return;
    has_not_finished = false;
    vertex_occupied_.clear();
    edge_occupied_.clear();
    for (int i = 0; i < optimal_path_plans.size(); i++) {
      if (current_id < optimal_path_plans[i].path.size()) {
        current_loc = optimal_path_plans[i].path[current_id].location;
        has_not_finished = true;
      } else {
        current_loc = optimal_path_plans[i].path.back().location;
      }

      if (current_id + 1 < optimal_path_plans[i].path.size()) {
        next_loc = optimal_path_plans[i].path[current_id + 1].location;
      } else {
        next_loc = optimal_path_plans[i].path.back().location;
      }

      {
        auto iter = vertex_occupied_.find(current_loc);
        if (iter != vertex_occupied_.end()) {
          g[iter->second].insert(i);
          g[i].insert(iter->second);
          iter->second = i;
        } else {
          vertex_occupied_[current_loc] = i;
        }
      }
      {
        auto iter = edge_occupied_.find({current_loc, next_loc});
        if (iter != edge_occupied_.end()) {
          g[iter->second].insert(i);
          g[i].insert(iter->second);
        }
        edge_occupied_[{next_loc, current_loc}] = i;
      }
    }

    for (int i = 0; i < path_plans.size(); i++) {
      if (current_id < path_plans[i].path.size()) {
        current_loc = path_plans[i].path[current_id].location;
        has_not_finished = true;
      } else {
        current_loc = path_plans[i].path.back().location;
      }

      if (current_id + 1 < path_plans[i].path.size()) {
        next_loc = path_plans[i].path[current_id + 1].location;
      } else {
        next_loc = path_plans[i].path.back().location;
      }

      {
        auto iter = vertex_occupied_.find(current_loc);
        if (iter != vertex_occupied_.end()) {
          g[iter->second].insert(i);
          g[i].insert(iter->second);
          iter->second = i;
        } else {
          vertex_occupied_[current_loc] = i;
        }
      }
      {
        auto iter = edge_occupied_.find({current_loc, next_loc});
        if (iter != edge_occupied_.end()) {
          g[iter->second].insert(i);
          g[i].insert(iter->second);
        }
        edge_occupied_[{next_loc, current_loc}] = i;
      }
    }
    current_id++;
  }
  graph->resize(path_plans.size());
  for (int i = 0; i < path_plans.size(); i++) {
    (*graph)[i].assign(g[i].begin(), g[i].end());
  }
}


}  // namespace lns
}  // namespace planner
