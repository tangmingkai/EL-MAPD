#include <vector>

#include "env/environment.h"
#include "model/action_model.h"
#include "planner/guidance_map/guidance_map.h"
#include "planner/heuristic/heuristic_state.h"
#include "planner/heuristic/heuristic_search.h"
#include "planner/heuristic/open_list.h"
#include "planner/utils/utils.h"

namespace planner {
namespace heuristic {
HeuristicSearch::HeuristicSearch(const Environment& env, int n_orients,
                  const GuidanceMap& guidance_map,
                  const ActionModel& action_model, const bool& is_loaded,
                  const double& energy_weight,
                  const CostType &cost_type)
    : env_(env),
      n_orients_(n_orients),
      guidance_map_(guidance_map),
      action_model_(action_model),
      is_loaded_(is_loaded),
      energy_weight_(energy_weight),
      cost_type_(cost_type) {
  max_states_ = env.rows * env.cols * n_orients;
  n_states_ = 0;
  open_list_ = new OpenList(max_states_);
  all_states_ = new State[max_states_];
  max_successors_ = 8;
  successors_ = new State[max_successors_];
  Reset();
}

void HeuristicSearch::Reset() {
  open_list_->Clear();
  n_states_ = 0;
  for (int i = 0; i < max_states_; ++i) {
    if (all_states_[i].pos != -1) {
      all_states_[i].pos = -1;
      all_states_[i].orient = -1;
      all_states_[i].g = -1;
      all_states_[i].prev = nullptr;
    }
  }
  n_successors_ = 0;
}

HeuristicSearch::~HeuristicSearch() {
  delete open_list_;
  delete[] all_states_;
  delete[] successors_;
}

void HeuristicSearch::ClearSuccessors() { n_successors_ = 0; }

void HeuristicSearch::AddSuccessor(int pos, int orient, double g, State* prev) {
  successors_[n_successors_].pos = pos;
  successors_[n_successors_].orient = orient;
  successors_[n_successors_].g = g;
  successors_[n_successors_].prev = prev;
  ++n_successors_;
}

// currently no heuristic is used, so it is dijkstra actually.
void HeuristicSearch::GetSuccessors(State* curr, int start_pos) {
  ClearSuccessors();

  int pos = curr->pos;
  int x = pos % (env_.cols);
  int y = pos / (env_.cols);
  int orient = curr->orient;
  double cost;
  // FW
  if (!(pos != start_pos && env_.charges[pos])) {
    int next_pos;
    cost = Utils::GetActionCostByType(
        cost_type_, is_loaded_, pos, orient, Action::FW, action_model_,
        guidance_map_, energy_weight_);
    if (orient == 0) {
      // east
      if (x + 1 < env_.cols) {
        next_pos = pos + 1;
        if (env_.obstacles[next_pos] == false) {
          AddSuccessor(next_pos, orient, curr->g + cost, curr);
        }
      }
    } else if (orient == 1) {
      // south
      if (y + 1 < env_.rows) {
        next_pos = pos + env_.cols;
        if (env_.obstacles[next_pos] == false) {
          AddSuccessor(next_pos, orient, curr->g + cost, curr);
        }
      }
    } else if (orient == 2) {
      // west
      if (x - 1 >= 0) {
        next_pos = pos - 1;
        if (env_.obstacles[next_pos] == false) {
          AddSuccessor(next_pos, orient, curr->g + cost, curr);
        }
      }
    } else if (orient == 3) {
      // north
      if (y - 1 >= 0) {
        next_pos = pos - env_.cols;
        if (env_.obstacles[next_pos] == false) {
          AddSuccessor(next_pos, orient, curr->g + cost, curr);
        }
      }
    } else {
      std::cerr << "spatial search in heuristics: invalid orient: " << orient
                << endl;
      exit(-1);
    }
  }

  int next_orient;
  // CR
  cost = Utils::GetActionCostByType(cost_type_, is_loaded_, pos,
                                              orient, Action::CR, action_model_,
                                              guidance_map_, energy_weight_);
  next_orient = (orient + 1 + n_orients_) % n_orients_;
  AddSuccessor(pos, next_orient, curr->g + cost, curr);

  // CCR
  cost = Utils::GetActionCostByType(
      cost_type_, is_loaded_, pos, orient, Action::CCR, action_model_,
      guidance_map_, energy_weight_);
  next_orient = (orient - 1 + n_orients_) % n_orients_;
  AddSuccessor(pos, next_orient, curr->g + cost, curr);
}

State* HeuristicSearch::AddState(int pos, int orient, double g, State* prev) {
  int index;
  index = pos * n_orients_ + orient;
  State* s = &(all_states_[index]);
  if (index >= max_states_) {
    std::cout << "state is out-of-boundary" << std::endl;
    exit(-1);
  }
  if (s->pos != -1) {
    std::cout << "State {} {} already exists!" << pos << ", " << orient
              << std::endl;
    exit(-1);
  }

  s->pos = pos;
  s->orient = orient;
  s->g = g;
  s->prev = prev;

  ++n_states_;
  return s;
}

void HeuristicSearch::SearchForAll(int start_pos, int start_orient) {
  State* start = AddState(start_pos, start_orient, 0, nullptr);
  open_list_->Push(start);
  // std::cout << "start " << env_.Str(start_pos, start_orient) << std::endl;
  while (!open_list_->Empty()) {
    State* curr = open_list_->Pop();
    curr->closed = true;
    GetSuccessors(curr, start_pos);
    for (int i = 0; i < n_successors_; ++i) {
      State* next = &(successors_[i]);
      int index = next->pos * n_orients_ + next->orient;
      if (all_states_[index].pos == -1) {
        // new state
        State* new_state =
            AddState(next->pos, next->orient, next->g, next->prev);
        new_state->closed = false;
        open_list_->Push(new_state);
      } else {
        // old state
        State* old_state = &all_states_[index];
        if (next->g < old_state->g) {
          // we need to update the state
          old_state->Copy(next);
          if (old_state->closed) {
            old_state->closed = false;
            open_list_->Push(old_state);
          } else {
            open_list_->Increase(old_state);
          }
        }
      }
    }
  }
}

}  // namespace heuristic
}  // namespace planner
