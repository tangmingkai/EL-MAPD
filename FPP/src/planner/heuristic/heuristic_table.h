#pragma once
#include <omp.h>

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "boost/filesystem.hpp"
#include "boost/format.hpp"
#include "common/common.h"
#include "env/environment.h"
#include "model/action_model.h"
#include "planner/heuristic/heuristic_search.h"


#define MAX_HEURISTIC 1e+100

namespace planner {

class HeuristicTable {
 public:
  HeuristicTable(const Environment& env, const GuidanceMap& guidance_map,
                 const ActionModel& action_model, const bool& is_loaded,
                 const double& energy_weight, const CostType& cost_type);
  ~HeuristicTable();

  void Preprocess(const std::string& suffix = "");
  void Save(const string& fpath);
  void Load(const string& fpath);

  double Get(int loc1, int loc2) const;
  double Get(int loc1, int orient1, int loc2) const;
  double Get(int loc1, int orient1, int loc2, int orient2) const;
  // void DumpMainHeuristics(int start_loc, string file_path_prefix);
  std::pair<int, int> GetFrom(int start_loc, int start_orient, int current_loc,
                              int current_orient) const;
  double energy_weight() const { return energy_weight_; }

 public:
  // weights is an array of [loc_size*n_orientations]
  void ParallelComputeWeightedHeuristics();
  void ComputeWeightedHeuristics(int start_loc_idx, double* values,
                                 heuristic::HeuristicSearch* planner);
  size_t GetFullID(const int& start_loc_idx, const int& start_orient,
                   const int& loc_idx, const int& orient) const;
  std::pair<int, int> GetCurrentPosLocFromFullID(int full_id) const;
  const Environment& env_;
  const GuidanceMap& guidance_map_;
  const ActionModel& action_model_;
  double energy_weight_;
  // loc1, loc2
  double* main_heuristics_;
  // loc1, loc2, orient1, orient2
  double* sub_heuristics_;
  double* full_heuristics_;
  int* full_from_;
  int* empty_locs_;
  int* loc_idxs_;
  int n_orientations_ = 4;
  size_t loc_size_ = 0;
  size_t state_size_;
  bool is_loaded_;
  // const CostCalculator &cost_calculator_;
  const CostType& cost_type_;
};
}  // namespace planner
