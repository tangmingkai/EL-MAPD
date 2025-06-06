#pragma once

#include <iostream>
#include <utility>
#include <boost/program_options.hpp>
#include <boost/functional/hash.hpp>
namespace planner {
struct TSState {
  TSState() = default;
  TSState(int time, int location) : time(time), location(location) {}

  bool operator==(const TSState& s) const {
    return time == s.time && location == s.location;
  }
  // bool operator!=(const TSState& s) const {
  //   return time != s.time || x != s.x || y != s.y;
  // }

  // friend std::ostream& operator<<(std::ostream& os, const TSState& s) {
  //   return os << "([" << s.time <<"], "<< s.x << ", " << s.y << ")";
  // }
  int time;
  int location;
};
}  // namespace planner

namespace std {
template <>
struct hash<planner::TSState> {
  size_t operator()(const planner::TSState& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.location);
    return seed;
  }
};
}  // namespace std

namespace std {
template <>
struct hash<std::pair<planner::TSState, planner::TSState>> {
  size_t operator()(
      const std::pair<planner::TSState, planner::TSState>& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<planner::TSState>()(s.first));
    boost::hash_combine(seed, std::hash<planner::TSState>()(s.second));
    return seed;
  }
};
};  // namespace std
