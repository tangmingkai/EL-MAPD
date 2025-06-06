#pragma once
#include "common/state.h"
#include "common/common.h"
struct Agent {
  Agent(const int &id, const State &start_state)
      : id(id), state(start_state), task() {}
  int id;
  State state;
  Task task;
};
