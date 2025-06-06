#pragma once
#include <boost/functional/hash.hpp>
#include "common/state.h"
namespace planner {
struct LState {
  int label;
  State state;
  LState() = default;
  LState(const int &label, const State &s)
      : label(label), state(s) {}
  bool operator==(const LState &other) const {
    // ignore energy and is_loaded
    return label == other.label && state.timestep == other.state.timestep &&
           state.location == other.state.location &&
           state.orientation == other.state.orientation;
  }
};
}  // namespace planner

namespace std {
template <>
struct hash<planner::LState> {
 public:
  size_t operator()(const planner::LState &x) const {
    // ignore energy and is_loaded
    size_t seed = 0;
    boost::hash_combine(seed, x.label);
    boost::hash_combine(seed, x.state.timestep);
    boost::hash_combine(seed, x.state.location);
    boost::hash_combine(seed, x.state.orientation);
    return seed;
  }
};  // namespace std
}