#pragma once
#include <string>
#include <list>
#include <tuple>
#include <vector>

#include "common/state.h"
#include "env/environment.h"
#include "common/agent.h"
#include "model/action.h"


class ActionModel {
 public:
  ActionModel() = default;
  ActionModel(const Environment& env, double idle_comsumption,
              double active_unloaded_comsumption,
              double active_loaded_comsumption,
              double charge_energy_per_timestep, double full_energy);

  bool IsValid(const vector<Agent>& prev, const vector<Action>& action) const;
  State ResultState(const State& prev,
                    const Action& action) const;
  double NextEnergy(const State& state, const Action& action) const;
  double NextEnergy(const int& loc, const bool& is_loaded,
                 const double& original_energy, const Action& action) const;
  double PureEnergyComsumption(const bool& is_loaded,
                               const Action& action) const;
  Action GetNormalAction(const State& prev, const State &next) const;
  bool IsForwardOutOfBoundary(const State& prev) const;
  static const std::vector<Action>& normal_actions() {
    return normal_actions_;
  }
  const double &full_energy() const {return full_energy_;}
  const std::vector<int>& moves() const { return moves_; }
  const double& active_unloaded_comsumption() const {
    return active_unloaded_comsumption_;
  }
  const double &charge_energy_per_timestep() const {
    return charge_energy_per_timestep_;
  }
  const double &idle_comsumption() const {return idle_comsumption_;}

 private:
  std::vector<State> ResultStates(const vector<Agent>& prev,
                                  const vector<Action>& action) const;
  static const std::vector<Action> normal_actions_;
  const Environment& env_;
  int rows_;
  int cols_;
  std::vector<int> moves_;
  double idle_comsumption_;
  double active_unloaded_comsumption_;
  double active_loaded_comsumption_;
  double charge_energy_per_timestep_;
  double full_energy_;
};


