#include "model/action_model.h"
#include <utility>
#include <unordered_map>
#include <algorithm>



const std::vector<Action> ActionModel::normal_actions_ = {
    Action::FW, Action::CR, Action::CCR, Action::W};

ActionModel::ActionModel(const Environment& env, double idle_comsumption,
                         double active_unloaded_comsumption,
                         double active_loaded_comsumption,
                         double charge_energy_per_timestep, double full_energy)
    : env_(env),
      rows_(env.rows),
      cols_(env.cols),
      idle_comsumption_(idle_comsumption),
      active_unloaded_comsumption_(active_unloaded_comsumption),
      active_loaded_comsumption_(active_loaded_comsumption),
      charge_energy_per_timestep_(charge_energy_per_timestep),
      full_energy_(full_energy) {
  moves_.resize(4);
  moves_[0] = 1;
  moves_[1] = cols_;
  moves_[2] = -1;
  moves_[3] = -cols_;
}

std::vector<State> ActionModel::ResultStates(
    const vector<Agent>& prev, const vector<Action>& action) const {
  vector<State> next(prev.size());
  for (size_t i = 0; i < prev.size(); i++) {
    next[i] = ResultState(prev[i].state, action[i]);
  }
  return std::move(next);
}

State ActionModel::ResultState(const State& prev,
                               const Action& action) const {
  int new_location = prev.location;
  int new_orientation = prev.orientation;
  if (action == Action::FW) {
     new_location = new_location + moves_[prev.orientation];
    if (!IsForwardOutOfBoundary(prev) || env_.obstacles[new_location]) {
      return State(-1, -1, -1, -1, false);
    }
  } else if (action == Action::CR) {
    new_orientation = (prev.orientation + 1) % 4;

  } else if (action == Action::CCR) {
    new_orientation = (prev.orientation - 1) % 4;
    if (new_orientation == -1) new_orientation = 3;
  }
  double next_energy = NextEnergy(prev, action);
  if (next_energy < 1e-6) {
    return State(-1, -1, -1, -1, false);
  }
  bool next_is_loaded;
  if (action == Action::P) {
    if (prev.is_loaded == true) {
      return State(-1, -1, -1, -1, false);
    }
    next_is_loaded = true;
  } else if (action == Action::D) {
    if (prev.is_loaded == false) {
      return State(-1, -1, -1, -1, false);
    }
    next_is_loaded = false;
  } else {
    next_is_loaded = prev.is_loaded;
  }
  return State(new_location, prev.timestep + 1, new_orientation, next_energy,
               next_is_loaded);
}


bool ActionModel::IsForwardOutOfBoundary(const State& prev) const {
  int x = prev.location % (env_.cols);
  int y = prev.location / (env_.cols);
  int orient = prev.orientation;
  if (orient == 0) {
    // east
    if (x + 1 < env_.cols) {
      return true;
    }
  } else if (orient == 1) {
    // south
    if (y + 1 < env_.rows) {
      return true;
    }
  } else if (orient == 2) {
    // west
    if (x - 1 >= 0) {
      return true;
    }
  } else if (orient == 3) {
    // north
    if (y - 1 >= 0) {
      return true;
    }
  }
  return false;
}

bool ActionModel::IsValid(const std::vector<Agent>& agents,
                          const std::vector<Action>& actions) const {
  if (agents.size() != actions.size()) {
    std::cout << "incorrect vector size" << std::endl;
    return false;
  }

  std::vector<State> next_states = ResultStates(agents, actions);
  std::unordered_map<int, int> vertex_occupied;

  std::unordered_map<std::pair<int, int>, int, PairHash> edge_occupied;

  for (int i = 0; i < agents.size(); i++) {
    if (next_states[i].location < 0 ||
        next_states[i].location >= env_.obstacles.size() ||
        (abs(next_states[i].location / env_.cols -
             agents[i].state.location / env_.cols) +
             abs(next_states[i].location % env_.cols -
                 agents[i].state.location % env_.cols) >
         1)) {
      std::cout << "unallowed move" << i << " "
                << env_.Str(next_states[i].location, next_states[i].orientation)
                << std::endl;
      return false;
    }
    if (env_.obstacles[next_states[i].location] == 1) {
      std::cout << "move to obstacle" << std::endl;
      return false;
    }

    if (vertex_occupied.find(next_states[i].location) !=
        vertex_occupied.end()) {
      std::cout << "vertex conflict:" << i << " "
                << vertex_occupied[next_states[i].location] << " "
                << next_states[i].timestep << std::endl;
      return false;
    }

    if (edge_occupied.find({agents[i].state.location,
                            next_states[i].location}) != edge_occupied.end()) {
      std::cout
          << "edge conflict:" << i << " "
          << edge_occupied[{agents[i].state.location, next_states[i].location}]
          << " " << next_states[i].timestep << std::endl;
      return false;
    }

    vertex_occupied[next_states[i].location] = i;
    edge_occupied[{next_states[i].location, agents[i].state.location}] = i;
  }

  bool res = true;
  for (int i = 0; i < agents.size(); i++) {
    double next = NextEnergy(agents[i].state, actions[i]);
    if (next <= 0) {
      std::cout << "out-of-battery" << i << " " << agents[0].state.timestep + 1
                << " " << agents[i].state.energy << " "
                << PureEnergyComsumption(agents[i].state.is_loaded, actions[i])
                << std::endl;
      return false;
    }
  }
  return true;
}

double ActionModel::NextEnergy(const State& state, const Action& action) const {
  return NextEnergy(state.location, state.is_loaded, state.energy, action);
}

double ActionModel::NextEnergy(const int& loc, const bool& is_loaded,
                               const double &original_energy,
                               const Action& action) const {
  double res = original_energy;
  if (action == Action::W || action == Action::P || action == Action::D ||
      action == Action::E) {
    res -= idle_comsumption_;
    if (action == Action::E) {
      if (env_.charges[loc]) {
        res = std::min(full_energy_, res + charge_energy_per_timestep_);
      } else {
        std::cout << "Error. Charge at a non-charge vertex" << std::endl;
        exit(-1);
      }
    }
  } else if (is_loaded) {
    res -= active_loaded_comsumption_;
  } else {
    res -= active_unloaded_comsumption_;
  }
  // if (action == Action::FW && 10000==original_energy) {
  //   std::cout << original_energy << " " << active_loaded_comsumption_ << " "
  //             << active_unloaded_comsumption_ << " " << idle_comsumption_ << " "
  //             << charge_energy_per_timestep_ << " " << res << " " << action
  //             << std::endl;
  // }
  res = std::max(res, 0.0);
  return res;
}

double ActionModel::PureEnergyComsumption(
    const bool& is_loaded, const Action& action) const {
  if (action == Action::W || action == Action::P || action == Action::D ||
      action == Action::E) {
    return idle_comsumption_;
  } else if (is_loaded) {
    return active_loaded_comsumption_;
  } else {
    return active_unloaded_comsumption_;
  }
}

Action ActionModel::GetNormalAction(const State& prev,
                                    const State& next) const {
  if (prev.location != next.location) return Action::FW;
  if (next.orientation == (prev.orientation + 3) % 4) return Action::CCR;
  if (next.orientation == (prev.orientation + 1) % 4) return Action::CR;
  return Action::W;
}
