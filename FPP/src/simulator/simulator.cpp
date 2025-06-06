#include "simulator/simulator.h"

#include <boost/tokenizer.hpp>
#include <cmath>
#include <functional>
#include <utility>

#include "common/logger.h"
#include "common/common.h"

using json = nlohmann::ordered_json;

Simulator::Simulator(
            const Environment& env,
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
            const bool &is_check_well_formed):
      env_(env),
      action_model_(action_model),
      tasks_proto_(tasks),
      preprocess_time_limit_(preprocess_time_limit),
      planning_time_limit_(planning_time_limit),
      cutoff_time_limit_(cutoff_time_limit),
      p_idle_(p_idle),
      input_json_file_(input_json_file),
      map_weights_path_(map_weights_path),
      energy_weight_(energy_weight),
      recharge_threshold_factor_(recharge_threshold_factor),
      lns_mode_(lns_mode),
      seed_(seed),
      is_check_well_formed_(is_check_well_formed) {
    assert(cutoff_time_limit_ < planning_time_limit_);
    total_energy_consumption_ = 0;
    total_charge_energy_ = 0;
    all_assigned_tasks_.clear();
    start_states_ = start_states;
    agents_.clear();
    for (int i = 0; i < start_states.size(); i++) {
      agents_.push_back(Agent(i, start_states[i]));
    }
    task_counter_.clear();
    task_counter_.resize(agents_.size(), 0);
    task_id_ = 0;
    num_of_task_finish_ = 0;
    for (size_t i = 0; i < agents_.size(); i++) {
      if (agents_[i].state.location >= env.obstacles.size()) {
        std::cout << "error: agent " << i
                  << "'s start location is out of map ("
                  << agents_[i].state.location << ")" << endl;
      }
      if (env.obstacles[agents_[i].state.location] == 1) {
        std::cout << "error: agent " << i << "'s start location is an obstacle("
             << agents_[i].state.location << ")" << endl;
        exit(-1);
      }
    }
    planner_ = new planner::FallbackPriorityPlanning(
        map_weights_path, env_, action_model_, energy_weight_,
        recharge_threshold_factor_, lns_mode_, seed_);
    minimum_energy_ = -1;
  }

Simulator::~Simulator() {
  // safely exit: wait for join the thread then delete planner and exit
    if (planner_thread_started_) {
      planner_thread_.join();
    }
    if (planner_ != nullptr) {
      delete planner_;
    }
}
void Simulator::Simulate(int simulation_timestep) {
  succ_ = false;
  Initialize();
  if (is_check_well_formed_ && !CheckWellFormed()) {
      std::cout << "It is not a well formed instance" << std::endl;
      exit(-1);
  }
  std::cout << "It is a well-formed instance" << std::endl;
  for (; time_step_ < simulation_timestep;) {
    auto start = std::chrono::steady_clock::now();
    std::vector<Action> actions = Plan();
    auto end = std::chrono::steady_clock::now();

    time_step_ += 1;
    for (int i = 0; i < agents_.size(); i++) {
      if (agents_[i].task.stage != TaskStage::Idle) solution_costs_[i]++;
    }
    std::cout << "running_time:"
              << std::chrono::duration<double>(end - start).count()
              << std::endl;
    int finished_tasks_this_timestep = MoveOneStep(&actions);
    num_of_task_finish_ += finished_tasks_this_timestep;
    auto diff = end - start;
    planner_times_.push_back(std::chrono::duration<double>(diff).count());
    if (time_step_ >= simulation_timestep)
      break;
    UpdateTasks();
    bool complete_all = false;
    for (auto& agent : agents_) {
      if (agent.task.stage == TaskStage::Idle) {
        complete_all = true;
      } else {
        complete_all = false;
        break;
      }
    }
    if (complete_all) {
      break;
    }
  }
  succ_ = true;
}

void Simulator::Initialize() {
  paths_.clear();
  paths_.resize(agents_.size());
  for (int i = 0; i < agents_.size(); i++) {
    paths_[i].emplace_back(agents_[i].state);
  }
  events_.clear();
  agent_goal_infos_.clear();
  time_step_ = 0;

  auto start = std::chrono::steady_clock::now();
  bool planner_initialize_success = PlannerInitialize();
  auto end = std::chrono::steady_clock::now();
  preprocess_time_ = std::chrono::duration<double>(end - start).count();
  log_preprocessing(planner_initialize_success);
  if (!planner_initialize_success) {
    std::cout << "Initialize Unsuccess" << std::endl;
    exit(-1);
  }
  // initialize_goal_locations();
  UpdateTasks();
  actual_movements_.clear();
  actual_movements_.resize(agents_.size());
  solution_costs_.clear();
  solution_costs_.resize(agents_.size(), 0);
}

bool Simulator::PlannerInitialize() {
  std::packaged_task<void(const double &)> init_task(
      std::bind(&planner::FallbackPriorityPlanning::Initialize, planner_,
                std::placeholders::_1));
  auto init_future = init_task.get_future();
  auto init_thread = std::thread(std::move(init_task), preprocess_time_limit_);
  if (init_future.wait_for(std::chrono::milliseconds(static_cast<int>(
          preprocess_time_limit_ * 1000))) == std::future_status::ready) {
    init_thread.join();
    return true;
  }

  init_thread.detach();
  return false;
}

int Simulator::MoveOneStep(vector<Action>* actions) {
  if (agents_.size() != actions->size()) {
    std::cout << agents_.size() << " " << actions->size() << std::endl;
    std::cout << "plan TLE" << std::endl;
    exit(-1);
  }

  int finished_tasks_this_timestep = 0;
  if (!ValidMoves(agents_, *actions)) {
    std::cout << "plan inVaild" << std::endl;
    exit(-1);
  }
  for (int i = 0; i < agents_.size(); i++) {
    auto next_state =
        action_model_.ResultState(agents_[i].state, (*actions)[i]);
    total_energy_consumption_ += action_model_.PureEnergyComsumption(
        agents_[i].state.is_loaded, (*actions)[i]);
    if ((*actions)[i] == Action::E)
      total_charge_energy_ += next_state.energy - agents_[i].state.energy +
                              action_model_.idle_comsumption();
    agents_[i].state = next_state;
    if (agents_[i].task.stage != TaskStage::Idle) {
      if (agents_[i].task.stage == TaskStage::ToPickup &&
          (*actions)[i] == Action::P) {
        agents_[i].task.stage = TaskStage::ToDelivery;
        events_.push_back(
            make_tuple(time_step_, i, agents_[i].task.task_id, 1));
      }
      if (agents_[i].task.stage == TaskStage::ToDelivery &&
          (*actions)[i] == Action::D) {
        agents_[i].task.stage = TaskStage::Idle;
        finished_tasks_this_timestep++;
        events_.push_back(
            make_tuple(time_step_, i, agents_[i].task.task_id, 2));
      }
    }
    paths_[i].push_back(agents_[i].state);
    actual_movements_[i].push_back((*actions)[i]);
  }
  return finished_tasks_this_timestep;
}

bool Simulator::ValidMoves(const vector<Agent>& agents,
                           const vector<Action>& actions) const {
  return action_model_.IsValid(agents, actions);
}

vector<Action> Simulator::PlanWrapper() {
  vector<Action> actions;
  planner_->MainWorkflowLayer(cutoff_time_limit_, agents_, time_step_,
                              &actions, &agent_goal_infos_);

  return actions;
}

std::vector<Action> Simulator::Plan() {
  if (planner_thread_started_ &&
      planner_future_.wait_for(std::chrono::milliseconds(static_cast<int>(
          planning_time_limit_ * 1000))) != std::future_status::ready) {
    std::cout << planner_thread_started_ << "     "
              << (planner_future_.wait_for(std::chrono::seconds(0)) !=
                  std::future_status::ready)
              << std::endl;
    DEV_INFO(
          "planner cannot run because the previous run is still running",
          time_step_);
    if (planner_future_.wait_for(std::chrono::milliseconds(static_cast<int>(
            planning_time_limit_ * 1000))) == std::future_status::ready) {
      planner_thread_.join();
      planner_thread_started_ = false;
      return planner_future_.get();
    }
    DEV_INFO("planner timeout", time_step_);
    return {};
  }

  std::packaged_task<std::vector<Action>()> task(
      std::bind(&Simulator::PlanWrapper, this));
  planner_future_ = task.get_future();
  if (planner_thread_.joinable()) {
    planner_thread_.join();
  }
  planner_thread_ = std::thread(std::move(task));
  planner_thread_started_ = true;
  if (planner_future_.wait_for(std::chrono::milliseconds(static_cast<int>(
          planning_time_limit_ * 1000))) == std::future_status::ready) {
    planner_thread_.join();
    planner_thread_started_ = false;
    return planner_future_.get();
  }
  DEV_INFO("planner timeout", time_step_);
  return {};
}

void Simulator::UpdateTasks() {
  for (int k = 0; k < agents_.size(); k++) {
    if (agents_[k].task.stage == TaskStage::Idle) {
      double v = 1.0 * rand() / RAND_MAX;
      if (v < p_idle_) continue;
      int i = task_counter_[k] * agents_.size() + k;
      agents_[k].task = tasks_proto_[i % tasks_proto_.size()];
      agents_[k].task.start_time_step = time_step_;
      agents_[k].task.stage = TaskStage::ToPickup;
      agents_[k].task.task_id = task_id_;
      agents_[k].task.assigned_agent_id = k;
      // events_[k].push_back(make_tuple(task_id_, time_step_, "assigned"));
      all_assigned_tasks_.push_back(agents_[k].task);
      task_id_++;
      task_counter_[k]++;
    }
  }
}


void Simulator::log_preprocessing(bool succ) {
  if (succ) {
    DEV_INFO(
        "Preprocessing success: Time:" , preprocess_time_,
        time_step_);
  } else {
    DEV_ERROR("Preprocessing timeout", time_step_);
  }
  DEV_FLUSH();
}

void Simulator::log_event_assigned(int agent_id, int task_id, int timestep) {
  DEV_INFO("Task " + std::to_string(task_id) +
                       " is assigned to agent " + std::to_string(agent_id),
                   timestep);
}

void Simulator::log_event_finished(int agent_id, int task_id, int timestep) {
  DEV_INFO("Agent " + std::to_string(agent_id) + " finishes task " +
                      std::to_string(task_id),
                  timestep);
}

void Simulator::SaveResults(const string& fileName) const {
  json js;
  // Save action model
  js["actionModel"] = "MAPF_T";
  js["pIdle"] = p_idle_;
  js["minimumEnergy"] = minimum_energy_;
  js["succ"] = succ_;
  {
    json parameters;
    parameters["inputJsonFile"] = input_json_file_;
    parameters["mapWeightsPath"] = map_weights_path_;
    parameters["rechargeThresholdFactor"] = recharge_threshold_factor_;
    parameters["rechargeThreshold"] =
        recharge_threshold_factor_ * action_model_.full_energy();
    parameters["energyWeight"] = energy_weight_;
    parameters["lnsMode"] = lns_mode_;
    parameters["seed"] = seed_;
    js["parameters"] = parameters;
  }
  {
    json statistics;
    statistics["numTaskFinished"] = num_of_task_finish_;
    statistics["numTaskAssigned"] = all_assigned_tasks_.size();
    statistics["timeStep"] = time_step_;
    statistics["preprocessTimeLimit"] =
        DoubleToStringWithFiveDecimalPlaces(preprocess_time_limit_);
    statistics["preprocessTime"] =
        DoubleToStringWithFiveDecimalPlaces(preprocess_time_);
    statistics["planningTimeLimit"] =
        DoubleToStringWithFiveDecimalPlaces(planning_time_limit_);
    statistics["AveragePlanTime"] = DoubleToStringWithFiveDecimalPlaces(
        std::accumulate(planner_times_.begin(), planner_times_.end(), 0.0) /
        planner_times_.size());
    statistics["maxPlanTime"] = DoubleToStringWithFiveDecimalPlaces(
        *std::max_element(planner_times_.begin(), planner_times_.end()));
    statistics["averageRePlanTime"] =
        DoubleToStringWithFiveDecimalPlaces(planner_->GetAverageReplanTime());
    statistics["averageLNSTime"] =
        DoubleToStringWithFiveDecimalPlaces(planner_->GetAverageLNSTime());
    statistics["throughputPerSecond"] =
        1.0 * num_of_task_finish_ / (planning_time_limit_ * time_step_);
    statistics["throughputPerTimeStep"] =
        1.0 * num_of_task_finish_ / time_step_;
    statistics["energyConsumptionPreSecond"] =
        1.0 * total_energy_consumption_ / (planning_time_limit_ * time_step_);
    statistics["energyConsumptionPreTimeStep"] =
        1.0 * total_energy_consumption_ / time_step_;
    statistics["chargeEnergyPreSecond"] =
        1.0 * total_charge_energy_ / (planning_time_limit_ * time_step_);
    statistics["chargeEnergyPreTimeStep"] =
        1.0 * total_charge_energy_ / time_step_;
    int sum_of_cost = 0;
    int makespan = 0;
    if (agents_.size() > 0) {
      sum_of_cost = solution_costs_[0];
      makespan = solution_costs_[0];
      for (int a = 1; a < agents_.size(); a++) {
        sum_of_cost += solution_costs_[a];
        if (solution_costs_[a] > makespan) {
          makespan = solution_costs_[a];
        }
      }
    }
    statistics["sumOfCost"] = sum_of_cost;
    statistics["makespan"] = makespan;
    js["statistics"] = statistics;
  }
  {
    json lns_info_js = json::array();
    std::vector<std::tuple<string, int, int>> lns_infos;
    planner_->GetLnsInfo(&lns_infos);
    for (int i = 0 ; i < lns_infos.size(); i++) {
      auto &lns_info = lns_infos[i];
      std::string info =
          std::get<0>(lns_info) + ": " + std::to_string(std::get<2>(lns_info)) +
          " / " + std::to_string(std::get<1>(lns_info)) + " = " +
          DoubleToStringWithFiveDecimalPlaces(std::get<2>(lns_info) * 1.0 /
                                              std::get<1>(lns_info));
      lns_info_js.push_back(info);
    }
    js["lnsInfo"]  = lns_info_js;
  }
  js["teamSize"] = agents_.size();

  // Save start locations[x,y,orientation]
  {
    json start = json::array();
    for (int i = 0; i < agents_.size(); i++) {
      json s = json::array();
      s.push_back(start_states_[i].location % env_.cols);
      s.push_back(start_states_[i].location / env_.cols);
      switch (start_states_[i].orientation) {
        case 0:
          s.push_back("E");
          break;
        case 1:
          s.push_back("S");
          break;
        case 2:
          s.push_back("W");
          break;
        case 3:
          s.push_back("N");
          break;
      }
      s.push_back(paths_[i][0].energy);
      start.push_back(s);
    }
    js["start"] = start;
  }
  {
    // Save actual paths
    json apaths = json::array();
    for (int i = 0; i < agents_.size(); i++) {
      std::string path;
      bool first = true;
      for (const auto action : actual_movements_[i]) {
        if (!first) {
          path += ",";
        } else {
          first = false;
        }
        if (action == Action::FW) {
          path += "F";
        } else if (action == Action::CR) {
          path += "R";
        } else if (action == Action::CCR) {
          path += "C";
        } else if (action == Action::W) {
          path += "W";
        } else if (action == Action::P) {
          path += "P";
        } else if (action == Action::D) {
          path += "D";
        } else if (action == Action::E) {
          path += "E";
        } else {
          std::cout << "action_error" << std::endl;
          exit(-1);
        }
      }
      apaths.push_back(path);
    }
    js["paths"] = apaths;
  }
  {
    json energy = json::array();
    for (int i = 0; i < paths_.size(); i++) {
      std::string path;
      bool first = true;
      for (int j = 1; j < paths_[i].size(); j++) {
        if (!first) {
          path += ",";
        } else {
          first = false;
        }
        path += std::to_string(static_cast<int>(paths_[i][j].energy + 0.5));
      }
      energy.push_back(path);
    }
    js["energy"] = energy;
  }
  {
    json schedule = json::array();
    std::vector<std::vector<Task>> agent_tasks(agents_.size());
    for (auto& task : all_assigned_tasks_) {
      agent_tasks[task.assigned_agent_id].push_back(task);
    }
    for (auto agent_task : agent_tasks) {
      std::string info = "";
      bool first = true;
      for (auto task : agent_task) {
        if (!first) {
          info += ",";
        } else {
          first = false;
        }
        info += std::to_string(task.start_time_step) + ":" +
                std::to_string(task.task_id);
      }
      schedule.push_back(info);
    }
    js["schedule"] = schedule;
  }
  {
    // planned paths
    js["preprocessTime"] = preprocess_time_;
    json planning_times = json::array();
    for (int i = 0; i < planner_times_.size(); i++) {
      planning_times.push_back(
          DoubleToStringWithFiveDecimalPlaces(planner_times_[i]) + " : " +
          DoubleToStringWithFiveDecimalPlaces(planner_->replan_times()[i]) +
          " + " +
          DoubleToStringWithFiveDecimalPlaces(planner_->lns_times()[i]));
    }
    js["plannerTimes"] = planning_times;
  }
  {
    json agent_goal_info_json = json::array();
    for (auto e : agent_goal_infos_) {
      json ev = json::array();
      int time_step;
      int agent_id;
      int goal_0;
      int goal_1;
      int goal_2;
      std::tie(time_step, agent_id, goal_0, goal_1, goal_2) = e;
      ev.push_back(time_step);
      ev.push_back(agent_id);
      json goal_json = json::array();
      goal_json.push_back(goal_0 % env_.cols);
      goal_json.push_back(goal_0 / env_.cols);
      if (goal_1 >= 0) {
        goal_json.push_back(goal_1 % env_.cols);
        goal_json.push_back(goal_1 / env_.cols);
      }
      if (goal_2 >= 0) {
        goal_json.push_back(goal_2 % env_.cols);
        goal_json.push_back(goal_2 / env_.cols);
      }
      ev.push_back(goal_json);
      agent_goal_info_json.push_back(ev);
    }
    js["agentGoalInfo"] = agent_goal_info_json;
  }
  {
    // Save events
    json events_json = json::array();

    for (auto e : events_) {
      json ev = json::array();
      int finish_time_step;
      int agent_id;
      int task_id;
      int target_id;
      std::tie(finish_time_step, agent_id, task_id, target_id) = e;
      ev.push_back(finish_time_step);
      ev.push_back(agent_id);
      ev.push_back(task_id);
      ev.push_back(target_id);
      events_json.push_back(ev);
    }
    js["events"] = events_json;
  }

  {
    // Save all tasks
    json tasks = json::array();
    for (auto t : all_assigned_tasks_) {
      json task = json::array();
      task.push_back(t.task_id);
      task.push_back(t.start_time_step);
      json target = json::array();
      target.push_back(t.pickup_loc % env_.cols);
      target.push_back(t.pickup_loc / env_.cols);
      target.push_back(t.delivery_loc % env_.cols);
      target.push_back(t.delivery_loc / env_.cols);
      task.push_back(target);
      tasks.push_back(task);
    }
    js["tasks"] = tasks;
  }

  std::ofstream f(fileName, std::ios_base::trunc | std::ios_base::out);
  f << std::setw(4) << js;
}

bool Simulator::CheckWellFormed() {
  for (int i = 0; i < agents_.size(); i++) {
    if (!env_.charges[agents_[i].state.location]) {
      std::cout << "Agent " << i << " start at " << agents_[i].state.location
                << " which is not a charge point" << std::endl;
      return false;
    }
  }
  minimum_energy_ = planner_->GetMinimumEnergyRequirement();
  if (minimum_energy_ >= action_model_.full_energy()) {
    std::cout << "The full energy should be strictly larger than "
              << minimum_energy_ << std::endl;
    return false;
  } else {
    std::cout << "The MinimumEnergyRequirement is " << minimum_energy_
              << std::endl;
  }
  return true;
}


