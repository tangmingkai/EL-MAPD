#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <utility>
#include "common/common.h"
struct Environment {
 public:
  int rows;
  int cols;
  std::vector<bool> obstacles;
  std::vector<bool> charges;
  std::vector<bool> task_points;
  std::string map_name;
  std::string file_storage_path;

  std::vector<int> loc2charge_id;
  std::vector<int> charges_locs;
  std::vector<int> task_locs;
  std::pair<int, int> GetXY(int loc) const { return {loc % cols, loc / cols}; }
  int GetLoc(int x, int y) const { return y * cols + x; }
  bool IsOutOfBoundary(int x, int y) const {
    return x < 0 || y < 0 || x >= cols || y >= rows;
  }
  Environment() {}
  void Load(const std::string &fname, const std::string &_file_storage_path);
  std::string StrOrient(int ori) const {
    if (ori == 0) return ">";
    if (ori == 1) return "v";
    if (ori == 2) return "<";
    return "^";
  }
  std::string Str(int loc, int ori) const {
    return "(" + std::to_string(loc % cols) + ", " +
           std::to_string(loc / cols) + ", " + StrOrient(ori) + ")";
  }
  std::string Str(int loc) const {
    return "(" + std::to_string(loc % cols) + ", " +
           std::to_string(loc / cols) + ")";
  }
};
