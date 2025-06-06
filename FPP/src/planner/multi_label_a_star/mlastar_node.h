#pragma once
#include "planner/multi_label_a_star/multi_label_a_star.h"
namespace planner {
struct MLAStarNode {
  MLAStarNode() = default;
  MLAStarNode(const LState& state, const double& g, const double& f)
      : state(state), g(g), f(f) {}

  bool operator<(const MLAStarNode& n) const {
    if (f != n.f) return f > n.f;
    if (g != n.g) return g < n.g;
    // if (state.state.timestep != n.state.state.timestep)
    //   return state.state.timestep > n.state.state.timestep;
    return state.label < n.state.label;
  }

  LState state;
  double g;
  double f;
};
}  // namespace planner
