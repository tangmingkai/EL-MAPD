#pragma once
#include <iostream>
//      N(3)
//       |
// W(2)-----E(0)
//       |
//      S(1)

// .------------>x
// |
// |
// |
// |
// v
// y

struct State {
  int location;
  int timestep;
  int orientation;  // 0:east, 1:south, 2:west, 3:north
  double energy;
  bool is_loaded;

  State()
      : location(-1),
        timestep(-1),
        orientation(-1),
        energy(-1),
        is_loaded(false) {}

  State(int location, int timestep, int orientation, double energy,
        bool is_loaded)
      : location(location),
        timestep(timestep),
        orientation(orientation),
        energy(energy),
        is_loaded(is_loaded) {}
};

std::ostream& operator<<(std::ostream& out, const State& s);
