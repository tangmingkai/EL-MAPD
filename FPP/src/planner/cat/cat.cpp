#include "planner/cat/cat.h"
#include "common/state.h"
namespace planner {
bool CAT::IsVaild(const State &prev,
                  const std::vector<int> &ignore_agent_ids) const {
  auto iter_vertex = vertex_cat_.find(TSState(prev.timestep, prev.location));
  if (iter_vertex == vertex_cat_.end()) return true;
  for (auto &id : ignore_agent_ids) {
    if (id == iter_vertex->second) return true;
  }
  return false;
}


bool CAT::IsVaild(const State &prev, const State &next,
                  const std::vector<int> &ignore_agent_ids) const {
  if (!IsVaild(next, ignore_agent_ids)) return false;
  auto prev_state = TSState(prev.timestep, next.location);
  auto next_state = TSState(next.timestep, prev.location);
  auto iter = edge_cat_.find({prev_state, next_state});
  if (iter == edge_cat_.end()) return true;
  for (auto &id : ignore_agent_ids) {
    if (id == iter->second) return true;
  }
  return false;
}

void CAT::Add(const int &agent_id, const State &state) {
  TSState s(state.timestep, state.location);
  loc_agents_[state.location].emplace_back(agent_id);
  if (vertex_cat_.find(s) != vertex_cat_.end()) {
    std::cout << "vertex conflict occur" << state.timestep << " "
              << state.location << " agents:" << agent_id << " "
              << vertex_cat_[s] << std::endl;
    exit(-1);
  }
  vertex_cat_[s] = agent_id;
}
void CAT::Add(const int &agent_id, const State &state1, const State &state2) {
  TSState s1(state1.timestep, state1.location);
  TSState s2(state2.timestep, state2.location);
  if (edge_cat_.find({s1, s2}) != edge_cat_.end()) {
    std::cout << "edge conflict occur" << std::endl;
    exit(-1);
  }
  edge_cat_[{s1, s2}] = agent_id;
}


void CAT::Delete(const int &agent_id, const State &state) {
  TSState s(state.timestep, state.location);
  {
    auto iter = vertex_cat_.find(s);
    if (iter == vertex_cat_.end()) {
      std::cout << agent_id << " " << state.location << " " << state.orientation
                << " " << state.timestep << " " << state.is_loaded << " "
                << state.energy << std::endl;
      std::cout << "delete vertex error" << std::endl;
      exit(-1);
    }
    vertex_cat_.erase(iter);
  }
  {
    auto iter = loc_agents_.find(state.location);
    if (iter == loc_agents_.end()) {
      std::cout << "delete vertex error2" << std::endl;
      exit(-1);
    }
    auto &agent_ids = iter->second;
    for (auto iter = agent_ids.begin(); iter != agent_ids.end(); iter++)
      if (*iter == agent_id) {
        agent_ids.erase(iter);
        return;
      }
    std::cout << agent_id << std::endl;
    std::cout << "delete vertex error3" << std::endl;
    exit(-1);
  }
}

void CAT::Delete(const int &agent_id, const State &state1,
                 const State &state2) {
  TSState s1(state1.timestep, state1.location);
  TSState s2(state2.timestep, state2.location);
  auto iter = edge_cat_.find({s1, s2});
  if (iter == edge_cat_.end()) {
    std::cout << "delete edge error" << std::endl;
    exit(-1);
  }
  edge_cat_.erase(iter);
}


void CAT::AddTimeStep(int time_step) {
  constrainted_time_steps_.insert(time_step);
}
void CAT::DeleteTimeStep(int time_step) {
  auto iter = constrainted_time_steps_.find(time_step);
  if (iter == constrainted_time_steps_.end()) {
    std::cout << "delete time step error" << std::endl;
    exit(-1);
  }
  constrainted_time_steps_.erase(iter);
}
int CAT::GetMaxConstraintedTimeStep() const {
  if (!constrainted_time_steps_.empty()) {
    return *constrainted_time_steps_.rbegin();
  } else {
    return -1;
  }
}

void CAT::GetConflictingAgents(
    const State &prev, const State &current,
    std::vector<int> *conflicting_agents) const {
  conflicting_agents->clear();
  {
    auto current_state = TSState(current.timestep, current.location);
    auto iter = vertex_cat_.find(current_state);
    if (iter != vertex_cat_.end()) {
      conflicting_agents->push_back(iter->second);
    }
  }
  {
    auto prev_state = TSState(prev.timestep, current.location);
    auto next_state = TSState(current.timestep, prev.location);
    auto iter = edge_cat_.find({prev_state, next_state});
    if (iter != edge_cat_.end()) {
      conflicting_agents->push_back(iter->second);
    }
  }
}

int CAT::GetAgentNumber(const int &location) const {
  auto iter = loc_agents_.find(location);
  if (iter == loc_agents_.end()) return 0;
  auto &agent_ids = iter->second;
  return agent_ids.size();
}

void CAT::AddLocaionAgents(const int &location,
                           std::unordered_set<int> *agents) const {
  auto iter = loc_agents_.find(location);
  if (iter == loc_agents_.end()) return;
  auto &agent_ids = iter->second;
  for (auto &agent_id : agent_ids) agents->insert(agent_id);
}

};  // namespace planner
