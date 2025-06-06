#pragma once
#include <iostream>

/*
  FW  - forward
  CR  - Clockwise rotate
  CCR - Counter clockwise rotate
  W   - Wait
  P   - Pickuop
  D   - Delivery
  E   - Charge
  NA  - Not applicable
*/

enum class Action { FW, CR, CCR, W, P, D, E, NA };

std::ostream& operator<<(std::ostream& stream, const Action& action);
