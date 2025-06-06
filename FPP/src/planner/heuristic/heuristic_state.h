#pragma once
#include <cstdlib>
#include <utility>

#include "boost/heap/pairing_heap.hpp"

namespace planner {

namespace heuristic {

struct State {
  int pos;
  int orient;
  double g;
  State* prev;

  State() : pos(-1), orient(-1), g(-1), prev(nullptr), closed(false) {}
  State(int pos, int orient, double g, State* prev)
      : pos(pos), orient(orient), g(g), prev(prev), closed(false) {}

  void Copy(const State* s) {
    pos = s->pos;
    orient = s->orient;
    g = s->g;
    prev = s->prev;
  }

  struct StateCompare {
    bool operator()(const State* s1, const State* s2) const {
      return s1->g > s2->g;
    }
  };

  struct StateHash {
    std::size_t operator()(const State* s) const {
      size_t loc_hash = std::hash<int>()(s->pos * 4 + s->orient);
      return loc_hash;
    }
  };

  struct StateEqual {
    bool operator()(const State* s1, const State* s2) const {
      return s1->pos == s2->pos && s1->orient == s2->orient;
    }
  };

  bool closed;
  int heap_index;
};
}  // namespace heuristic
}  // namespace planner
