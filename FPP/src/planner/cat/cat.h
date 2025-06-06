#pragma once
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/state.h"
#include "planner/cat/ts_state.h"

namespace planner{

class CAT {
 public:
  void Reset();
  void Add(const int &agent_id, const State &state);
  void Add(const int &agent_id, const State &state1, const State &state2);
  void Delete(const int &agent_id, const State &state);
  void Delete(const int &agent_id, const State &state1, const State &state2);
  bool IsVaild(const State &prev,
               const std::vector<int> &ignore_agent_ids) const;
  bool IsVaild(const State &prev, const State &next,
               const std::vector<int> &ignore_agent_ids) const;
  int GetAgentNumber(const int &location) const;
  void AddTimeStep(int time_step);
  void DeleteTimeStep(int time_step);
  int GetMaxConstraintedTimeStep() const;
  void GetConflictingAgents(const State &prev, const State &next,
                            std::vector<int> *conflicting_agents) const;
  void AddLocaionAgents(const int &location,
                        std::unordered_set<int> *agents) const;

 private:
  std::multiset<int> constrainted_time_steps_;
  std::unordered_map<int, std::vector<int>> loc_agents_;
  std::unordered_map<TSState, int> vertex_cat_;
  // std::unordered_map<int, std::pair<int, int>> charge_cat_;
  std::unordered_map<std::pair<TSState, TSState>, int> edge_cat_;
  // std::map<int, std::vector<TSState>> timestep2vertex_;
  // std::map<int, std::vector<std::pair<TSState, TSState>>> timestep2edge_;
  // int max_stop_time_;
};
};  // namespace planner

