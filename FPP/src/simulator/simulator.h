#pragma once
// #include "BasicSystem.h"
#include <pthread.h>

#include <future>
#include <tuple>
#include <vector>
#include <list>
#include <string>

#include "common/logger.h"
#include "common/tasks.h"
#include "env/environment.h"
#include "model/action_model.h"
#include "planner/fallback_priority_planning.h"

class Simulator {
 public:
  Simulator(const Environment& env,
            const ActionModel& action_model,
            const std::vector<Task> &tasks,
            const std::vector<State> &start_states,
            const double &preprocess_time_limit,
            const double &planning_time_limit,
            const double &cutoff_time_limit,
            const double &p_idle,
            const std::string &input_json_file,
            const std::string &map_weights_path,
            const double &energy_weight,
            const double &recharge_threshold_factor,
            const int &lns_mode,
            const uint &seed,
            const bool &is_check_well_formed);

  virtual ~Simulator();
  void Simulate(int simulation_timestep);
  std::vector<Action> Plan();
  std::vector<Action> PlanWrapper();

  void SaveResults(const string& fileName) const;

 private:
  const Environment& env_;
  const ActionModel& action_model_;
  const std::vector<Task> &tasks_proto_;

  std::vector<Task> all_assigned_tasks_;
  std::vector<int> task_counter_;
  int task_id_;

  planner::FallbackPriorityPlanning* planner_;
  std::future<std::vector<Action>> planner_future_;
  std::thread planner_thread_;
  bool planner_thread_started_ = false;
  int time_step_;

  double preprocess_time_limit_;
  double planning_time_limit_;
  double cutoff_time_limit_;

  std::vector<std::vector<State>> paths_;

  std::vector<Agent> agents_;
  std::vector<State> start_states_;

  std::vector<std::list<Action>> actual_movements_;

  // finish_timestep, agent_id, task_id, target_id
  std::vector<std::tuple<int, int, int, int>> events_;
  // timestep, agent_id, [goal_loctions] (1<=size<=3)
  std::vector<std::tuple<int, int, int, int, int>> agent_goal_infos_;

  // for evaluation
  vector<int> solution_costs_;
  int num_of_task_finish_;
  vector<double> planner_times_;
  double preprocess_time_;
  double p_idle_;
  std::string input_json_file_;

  std::string map_weights_path_;
  double energy_weight_;
  double recharge_threshold_factor_;
  int lns_mode_;
  uint seed_;

  double total_energy_consumption_;
  double total_charge_energy_;
  double total_planning_tim_;
  double minimum_energy_;

  bool is_check_well_formed_;
  bool succ_;

  void Initialize();
  bool PlannerInitialize();
  void UpdateTasks();

  int MoveOneStep(vector<Action>* actions);
  bool ValidMoves(const vector<Agent>& agents,
                  const vector<Action>& action) const;
  bool CheckWellFormed();

  void log_preprocessing(bool succ);
  void log_event_assigned(int agent_id, int task_id, int timestep);
  void log_event_finished(int agent_id, int task_id, int timestep);
};

