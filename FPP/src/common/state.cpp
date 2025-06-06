#include "common/state.h"

std::ostream &operator<<(std::ostream &out, const State &state) {
  out << state.location << "," << state.orientation << ","
      << state.timestep << "," << state.energy <<"," << state.is_loaded;
  return out;
}