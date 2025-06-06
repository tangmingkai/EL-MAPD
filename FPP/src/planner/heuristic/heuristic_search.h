#pragma once
#include <vector>

#include "env/environment.h"
#include "model/action_model.h"
#include "planner/guidance_map/guidance_map.h"
#include "planner/heuristic/heuristic_state.h"
#include "planner/heuristic/open_list.h"
#include "planner/cost_calculator/cost_type.h"

namespace planner {
namespace heuristic {
class HeuristicSearch {
 public:
  HeuristicSearch(const Environment& env, int n_orients,
                  const GuidanceMap& guidance_map,
                  const ActionModel& action_model, const bool& is_loaded,
                  const double& energy_weight,
                  const CostType &cost_type);

  void Reset();

  ~HeuristicSearch();


  void ClearSuccessors();

  void AddSuccessor(int pos, int orient, double g, State* prev);

  // currently no heuristic is used, so it is dijkstra actually.
  void GetSuccessors(State* curr, int start_pos);

  State* AddState(int pos, int orient, double g, State* prev);

  void SearchForAll(int start_pos, int start_orient);

  const State* all_states() const {return all_states_;}

 private:
  int n_orients_;
  int n_states_;
  int max_states_;
  OpenList* open_list_;
  State* all_states_;

  int n_successors_;
  int max_successors_;
  State* successors_;

  const Environment& env_;
  const GuidanceMap& guidance_map_;
  const ActionModel &action_model_;
  const bool is_loaded_;
  const double energy_weight_;
  const CostType &cost_type_;
};
}  // namespace heuristic
}  // namespace planner
