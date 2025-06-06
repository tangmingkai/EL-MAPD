#include "planner/fallback_priority_planning.h"

#include <algorithm>
#include <random>

#include "boost/filesystem.hpp"
#include "boost/format.hpp"
#include "common/common.h"
#include "common/time_limiter.h"
#include "planner/utils/utils.h"

namespace planner {

FallbackPriorityPlanning::FallbackPriorityPlanning(
    const std::string& map_weights_path, const Environment& env,
    const ActionModel& action_model, const double& energy_weight,
    const double& recharge_threshold_factor, const int& lns_mode,
    const uint& seed)
    : map_weights_path_(map_weights_path),
      env_(env),
      action_model_(action_model),
      energy_weight_(energy_weight),
      recharge_threshold_factor_(recharge_threshold_factor),
      lns_mode_(lns_mode),
      mt_(seed),
      seed_(seed) {}

void FallbackPriorityPlanning::Initialize(const double& preprocess_time_limit) {
  cout << "planner initialization begins" << endl;
  recharge_threshold_ = action_model_.full_energy() * recharge_threshold_factor_;
  guidance_map_ = std::make_shared<GuidanceMap>(env_);
  std::string suffix = guidance_map_->Load(map_weights_path_);
  unloaded_main_heuristic_table_ =
      std::make_shared<HeuristicTable>(env_, *guidance_map_, action_model_,
                                       false, energy_weight_, CostType::Main);
  unloaded_main_heuristic_table_->Preprocess(suffix);
  loaded_main_heuristic_table_ =
      std::make_shared<HeuristicTable>(env_, *guidance_map_, action_model_,
                                       true, energy_weight_, CostType::Main);
  loaded_main_heuristic_table_->Preprocess(suffix);
  unloaded_energy_heuristic_table_ =
      std::make_shared<HeuristicTable>(env_, *guidance_map_, action_model_,
                                       false, energy_weight_, CostType::Energy);
  unloaded_energy_heuristic_table_->Preprocess(suffix);
  loaded_energy_heuristic_table_ =
      std::make_shared<HeuristicTable>(env_, *guidance_map_, action_model_,
                                       true, energy_weight_, CostType::Energy);
  loaded_energy_heuristic_table_->Preprocess(suffix);

  charge_distance_table_ = std::make_shared<ChargeDistanceTable>(env_);
  charge_distance_table_->Preprocess(
      suffix, *unloaded_main_heuristic_table_);

  cost_calculator_ = std::make_shared<CostCalculator>(env_, seed_ * 7);
  cost_calculator_->Init(
      energy_weight_, action_model_, *guidance_map_,
      *unloaded_main_heuristic_table_, *loaded_main_heuristic_table_,
      *unloaded_energy_heuristic_table_, *loaded_energy_heuristic_table_,
      *charge_distance_table_);
  // dummy_action_model_ =
  //     std::make_shared<ActionModel>(env_, 1, 1, 1, 1, 2147483647);
  // weighted_time_heuristic_table_ = std::make_shared<HeuristicTable>(
  //     env_, *dummy_action_model_, false, *guidance_map_);
  // weighted_time_heuristic_table_->Preprocess(suffix);

  // heurisc_table_set_ = std::make_shared<HeuristicTableSet>(
  //     *weighted_unloaded_heuristic_table_, *weighted_loaded_heuristic_table_,
  //     *uniform_unloaded_heuristic_table_, *uniform_loaded_heuristic_table_,
  //     *weighted_time_heuristic_table_);

  single_agent_solver_ =
      std::make_shared<MLAStar>(env_, action_model_, *cost_calculator_);
  charge_point_allocated_agent_id_.clear();
  charge_point_allocated_agent_id_.resize(env_.charges_locs.size(), -1);
  int neighbor_size = 8;
  int thread_num = omp_get_max_threads();
  double reaction_factor = 0.01;
  double decay_factor = 0.01;

  lns_ = std::make_shared<lns::LNSGlobalManager>(
      env_, action_model_, *cost_calculator_, *single_agent_solver_,
      neighbor_size, thread_num, reaction_factor, decay_factor,
      recharge_threshold_, lns_mode_, seed_ * 7 + 2);
  std::cout << "planner initialization ends" << endl;
}

double FallbackPriorityPlanning::GetMinimumEnergyRequirement() const {
  std::cout << "GetMinimumEnergyRequirement" << std::endl;
  const auto& charge_locs = env_.charges_locs;
  const auto& task_locs = env_.task_locs;
  double max_cost = 0;
  long long ctr = 0;
  int step = 1000000;
  long long total =
      1LL * charge_locs.size() * task_locs.size() * task_locs.size();
  auto start = std::chrono::steady_clock::now();
// #pragma omp parallel for collapse(3)
  for (int charge_id = 0; charge_id < charge_locs.size(); charge_id++) {
    for (int pickup_id = 0; pickup_id < task_locs.size(); pickup_id++) {
      for (int delivery_id = 0; delivery_id < task_locs.size(); delivery_id++) {
        double min_cost = MAX_HEURISTIC;
        for (int charge1_orient = 0; charge1_orient < 4; charge1_orient++) {
          for (int pickup_orient = 0; pickup_orient < 4; pickup_orient++) {
            double cost1 = unloaded_energy_heuristic_table_->Get(
                charge_locs[charge_id], charge1_orient, task_locs[pickup_id],
                pickup_orient);
            double cost1_5 =
                  action_model_.PureEnergyComsumption(false, Action::P) *
                  guidance_map_->GetWeight(task_locs[pickup_id], pickup_orient,
                                           Action::P);
            if (cost1 >= MAX_HEURISTIC) continue;
            for (int delivery_orient = 0; delivery_orient < 4;
                 delivery_orient++) {
              double cost2 = loaded_energy_heuristic_table_->Get(
                  task_locs[pickup_id], pickup_orient, task_locs[delivery_id],
                  delivery_orient);
              double cost2_5 =
                  action_model_.PureEnergyComsumption(true, Action::D) *
                  guidance_map_->GetWeight(task_locs[delivery_id],
                                           delivery_orient, Action::D);
              for (int charge2_orient = 0; charge2_orient < 4;
                   charge2_orient++) {
                double cost3 = unloaded_energy_heuristic_table_->Get(
                    task_locs[delivery_id], delivery_orient,
                    charge_locs[charge_id], charge2_orient);
                if (cost2 >= MAX_HEURISTIC || cost3 >= MAX_HEURISTIC) continue;
                min_cost = std::min(min_cost,
                                    cost1 + cost1_5 + cost2 + cost2_5 + cost3);
              }
            }
          }
        }
#pragma omp critical  // 确保对 max_cost 的访问是安全的
        {
          max_cost = std::max(max_cost, min_cost);
          std::cout << env_.Str(charge_locs[charge_id]) << " "
                    << env_.Str(task_locs[pickup_id]) << " "
                    << env_.Str(task_locs[delivery_id]) << " "
                    << min_cost << std::endl;
          ctr++;
          if (ctr % step == 0) {
            auto end = std::chrono::steady_clock::now();
            double elapse = std::chrono::duration<double>(end - start).count();
            double estimated_total = elapse / ctr * total;
            cout << "CheckWellFormed:"
                 << static_cast<int>(10000.0 * ctr / total) / 100.0 << "%"
                 << " completed in " << elapse
                 << "s. estimated time to finish all: "
                 << estimated_total - elapse
                 << "s.  estimated total time: " << (estimated_total) << "s."
                 << endl;
          }
        }
      }
    }
  }
  max_cost = std::max(max_cost, action_model_.active_unloaded_comsumption()*2);
  std::cout << "GetMaximumCost Finished" << std::endl;
  std::cout << "max_cost:" << max_cost << std::endl;
  exit(-1);
  return max_cost;
}

void FallbackPriorityPlanning::NextTimeStep(const std::vector<Agent>& agents,
                               std::vector<int>* replan_agent_ids) {
  replan_agent_ids->clear();
  for (int i = 0; i < agents.size(); i++) {
    auto& path_plan = path_plans_[i];
    if (!path_plan.IsReachFinalState()) {
      cat_.Delete(i, path_plan.path[path_plan.current_id]);
      cat_.Delete(i, path_plan.path[path_plan.current_id],
                  path_plan.path[path_plan.current_id + 1]);
      double current_cost = cost_calculator_->GetActionMainCost(
          path_plan.GetCurrentIsLoaded(),
          path_plan.path[path_plan.current_id].location,
          path_plan.path[path_plan.current_id].orientation,
          path_plan.actions[path_plan.current_id]);
      path_plan.main_cost -= current_cost;

      if (path_plan.cache_initial_heuristic >= 0) {
        if (path_plan.is_fallback ||
            path_plan.actions[path_plan.current_id] == Action::E)
          path_plan.cache_initial_heuristic -= current_cost;
        else
          path_plan.cache_initial_heuristic = -1;
      }
      path_plan.current_id++;
      bool is_check_moving_as_expected = true;
      if (is_check_moving_as_expected) {
        auto &state = path_plan.path[path_plan.current_id];
        if (agents[i].state.location != state.location ||
            agents[i].state.orientation != state.orientation ||
            agents[i].state.timestep != state.timestep ||
            agents[i].state.is_loaded != state.is_loaded ||
            fabs(agents[i].state.energy - state.energy) > 1e-6) {
              std::cout << "Why not the same" << std::endl;
              std::cout << i << " " << agents[i].state.energy << " "
                        << state.energy << std::endl;
              exit(-1);
            }
      }
    }

    if (path_plan.status == AgentStatus::ToCharge) {
      if (path_plan.IsReachFinalState()) {
        replan_agent_ids->push_back(i);
      }
    } else {
      if (path_plan.IsPassedDelivery()) {
        replan_agent_ids->push_back(i);
      }
    }
  }
}

void FallbackPriorityPlanning::MainWorkflowLayer(
    double cutoff_time, const std::vector<Agent>& agents, const int &time_step,
    std::vector<Action>* actions,
    std::vector<std::tuple<int, int, int, int, int>>* agent_goal_infos) {
  TimeLimiter time_limiter(cutoff_time);
  std::vector<int> replan_agent_ids;
  if (path_plans_.empty()) {
    current_agent_goals_.resize(agents.size());
    for (auto& current_agent_goal : current_agent_goals_) {
      current_agent_goal = {-1, -1, -1};
    }
    path_plans_.resize(agents.size());
    for (auto& plan : path_plans_) {
      plan.Clear();
    }
    for (auto& agent : agents) {
      int charge_id = env_.loc2charge_id[agent.state.location];
      Utils::ConstructPureRechargePlan(agent.state, charge_id, action_model_,
                                       &path_plans_[agent.id]);
      Utils::AddNewPathPlan(path_plans_[agent.id], agent.id, &cat_,
                              &charge_point_allocated_agent_id_);
      replan_agent_ids.emplace_back(agent.id);
    }
  } else {
    NextTimeStep(agents, &replan_agent_ids);
  }
  std::cout << "Time Step:" << time_step
            << "  replan agent size:" << replan_agent_ids.size() << std::endl;
  RandomizedReplanningLayer(time_limiter, agents, &replan_agent_ids);
  std::cout << "Replan finished" << std::endl;
  double time1 = time_limiter.GetElapse();
  if (lns_mode_!= -1)
    lns_->ParallelALNSOptimizationLayer(time_limiter, &path_plans_, &cat_,
                                        &charge_point_allocated_agent_id_);
  double time2 = time_limiter.GetElapse();

  GetNextActions(actions);
  UpdateAgentGoalInfo(time_step, agent_goal_infos);
  replan_times_.emplace_back(time1);
  lns_times_.emplace_back(time2-time1);
  std::cout << "Finish Plan" << std::endl;
  return;
}

void FallbackPriorityPlanning::RandomizedReplanningLayer(
    const TimeLimiter& time_limiter, const std::vector<Agent>& agents,
    std::vector<int>* new_task_agent_ids) {
  std::shuffle(new_task_agent_ids->begin(), new_task_agent_ids->end(),
                      mt_);

  for (int new_task_agent_id = 0;
       new_task_agent_id < new_task_agent_ids->size(); new_task_agent_id++) {
    int id = (*new_task_agent_ids)[new_task_agent_id];
    if (agents[id].task.stage == TaskStage::Idle) {
      if (path_plans_[id].IsReachFinalState()) {
        PathPlan plan;
        Utils::ConstructPureRechargePlan(path_plans_[id].path.back(),
                                         path_plans_[id].charge_point_id,
                                         action_model_, &plan);
        Utils::DeleteOldPathPlan(path_plans_[id], id, &cat_,
                                 &charge_point_allocated_agent_id_);
        Utils::AddNewPathPlan(plan, id, &cat_,
                              &charge_point_allocated_agent_id_);
        path_plans_[id] = plan;
      }
      continue;
    }
    int charge_point_id = cost_calculator_->SelectChargeID(
        charge_point_allocated_agent_id_, id, agents[id].task.delivery_loc);
    if (charge_point_id < 0) {
      std::cout << "Why not enough charge point" << std::endl;
      exit(-1);
    }
    std::vector<int> goals{agents[id].task.pickup_loc,
                           agents[id].task.delivery_loc,
                           env_.charges_locs[charge_point_id]};
    std::vector<std::vector<State>> paths;
    PathPlan plan;
    bool succ = single_agent_solver_->RechargeAndFallbackPlanningLayer(
        time_limiter, LState(0, agents[id].state), cat_, goals,
        AgentStatus::OnTask, {id}, recharge_threshold_, &plan);
    if (!succ) {
      if (time_limiter.Timeout())
        path_plans_[id].status = AgentStatus::OnTask;
      else
        path_plans_[id].status = AgentStatus::ToCharge;
      if (path_plans_[id].IsReachFinalState()) {
        PathPlan plan;
        Utils::ConstructPureRechargePlan(path_plans_[id].path.back(),
                                         path_plans_[id].charge_point_id,
                                         action_model_, &plan);
        Utils::DeleteOldPathPlan(path_plans_[id], id, &cat_,
                                 &charge_point_allocated_agent_id_);
        Utils::AddNewPathPlan(plan, id, &cat_,
                              &charge_point_allocated_agent_id_);
        path_plans_[id] = plan;
      }
    } else {
      Utils::DeleteOldPathPlan(path_plans_[id], id, &cat_,
                               &charge_point_allocated_agent_id_);
      Utils::AddNewPathPlan(plan, id, &cat_, &charge_point_allocated_agent_id_);
      path_plans_[id] = plan;
    }
  }
}

void FallbackPriorityPlanning::GetNextActions(
    std::vector<Action>* actions) const {
  actions->clear();
  for (int i = 0; i < path_plans_.size(); i++) {
    auto& plan = path_plans_[i];
    // std::cout << i << " " << plan.actions[plan.current_id] << " "
    //           << plan.path.size() << " " << plan.is_fallback << std::endl;
    actions->push_back(plan.actions[plan.current_id]);
  }
}

void FallbackPriorityPlanning::UpdateAgentGoalInfo(const int &time_step,
    std::vector<std::tuple<int, int, int, int, int>>* agent_goal_infos) {
  for (int i = 0; i < path_plans_.size(); i++) {
    auto goal_tuple = path_plans_[i].goal_tuple();
    if (goal_tuple != current_agent_goals_[i]) {
      agent_goal_infos->push_back({time_step, i, std::get<0>(goal_tuple),
                                      std::get<1>(goal_tuple),
                                      std::get<2>(goal_tuple)});
      current_agent_goals_[i] = goal_tuple;
    }
  }
}

}  // namespace planner
