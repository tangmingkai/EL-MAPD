#include "model/action.h"
std::ostream& operator<<(std::ostream& stream, const Action& action) {
  if (action == Action::FW) {
    stream << "F";
  } else if (action == Action::CR) {
    stream << "R";
  } else if (action == Action::CCR) {
    stream << "C";
  } else if (action == Action::E) {
    stream << "E";
  } else if (action == Action::P) {
    stream << "P";
  } else if (action == Action::D) {
    stream << "D";
  } else {
    stream << "W";
  }
  return stream;
}