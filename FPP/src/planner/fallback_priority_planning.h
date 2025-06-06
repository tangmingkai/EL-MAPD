#pragma once

#include <memory>
#include <vector>
#include <string>
#include <list>
#include <utility>
#include <tuple>
#include "common/common.h"
#include "env/environment.h"
#include "model/action_model.h"
#include "planner/heuristic/heuristic_table.h"
#include "planner/charge_distance_table/charge_distance_table.h"
#include "planner/guidance_map/guidance_map.h"
#include "planner/cat/cat.h"
#include "planner/path_plan.h"
#include "planner/multi_label_a_star/multi_label_a_star.h"
#include "planner/cost_calculator/cost_calculator.h"
#include "planner/lns/lns_global_manager.h"

namespace planner {
class FallbackPriorityPlanning {
 public:
  FallbackPriorityPlanning(const std::string& map_weights_path,
                           const Environment& env,
                           const ActionModel& action_model,
                           const double& energy_weight,
                           const double& recharge_threshold_factor,
                           const int& lns_mode, const uint& seed);
  virtual ~FallbackPriorityPlanning() {}

  virtual void Initialize(const double &preprocess_time_limit);

  // return next states for all agents
  virtual void MainWorkflowLayer(
      double time_limit, const std::vector<Agent>& agents, const int& time_step,
      std::vector<Action>* actions,
      std::vector<std::tuple<int, int, int, int, int>>* agent_goal_infos);

  double GetMinimumEnergyRequirement() const;
  const std::vector<double> &replan_times() const { return replan_times_;}
  const std::vector<double> &lns_times() const { return lns_times_;}
  double GetAverageReplanTime() const {
    return std::accumulate(replan_times_.begin(), replan_times_.end(), 0.0) /
           replan_times_.size();
  }
  double GetAverageLNSTime() const {
    return std::accumulate(lns_times_.begin(), lns_times_.end(), 0.0) /
           lns_times_.size();
  }
  void GetLnsInfo(
      std::vector<tuple<string, int, int>>* global_lns_infos) const {
    return lns_->GetLnsInfo(global_lns_infos);
  }

 private:
  void NextTimeStep(const std::vector<Agent>& agents,
      std::vector<int>* next_task_agent_ids);
  void RandomizedReplanningLayer(
      const TimeLimiter &time_limiter, const std::vector<Agent>& agents,
      std::vector<int>* new_task_agent_ids);
  void GetNextActions(std::vector<Action>* actions) const;
  void ConstructPureRechargePlan(const State& current_state,
                                 const int& charge_point_id,
                                 PathPlan* plan) const;
  void UpdateAgentGoalInfo(const int &time_step,
      std::vector<std::tuple<int, int, int, int, int>>* agent_goal_infos);
  std::vector<PathPlan> path_plans_;
  std::string map_weights_path_;
  const Environment &env_;
  const ActionModel &action_model_;
  double energy_weight_;
  double recharge_threshold_factor_;
  double recharge_threshold_;
  int lns_mode_;
  double cutoff_time_;
  CAT cat_;
  std::shared_ptr<GuidanceMap> guidance_map_;
  std::shared_ptr<GuidanceMap> uniform_map_;
  std::shared_ptr<HeuristicTable> unloaded_main_heuristic_table_;
  std::shared_ptr<HeuristicTable> loaded_main_heuristic_table_;
  std::shared_ptr<HeuristicTable> unloaded_energy_heuristic_table_;
  std::shared_ptr<HeuristicTable> loaded_energy_heuristic_table_;
  std::shared_ptr<CostCalculator> cost_calculator_;

  std::shared_ptr<HeuristicTable> weighted_time_heuristic_table_;
  std::shared_ptr<MLAStar> single_agent_solver_;
  std::shared_ptr<ActionModel> dummy_action_model_;
  std::vector<int> charge_point_allocated_agent_id_;
  std::shared_ptr<ChargeDistanceTable> charge_distance_table_;

  std::shared_ptr<lns::LNSGlobalManager> lns_;

  std::vector<tuple<int, int, int>> current_agent_goals_;
  std::mt19937 mt_;
  uint seed_;

  std::vector<double> replan_times_;
  std::vector<double> lns_times_;
};
};
