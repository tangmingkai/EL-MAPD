#include "planner/heuristic/heuristic_table.h"

#include <string>
#include "common/logger.h"


namespace planner {
HeuristicTable::HeuristicTable(const Environment &env,
                               const GuidanceMap &guidance_map,
                               const ActionModel &action_model,
                               const bool &is_loaded,
                               const double &energy_weight,
                               const CostType &cost_type)
    : env_(env),
      action_model_(action_model),
      guidance_map_(guidance_map),
      is_loaded_(is_loaded),
      energy_weight_(energy_weight),
      cost_type_(cost_type) {
  n_orientations_ = 4;
  loc_size_ = 0;
  for (int i = 0; i < env_.obstacles.size(); ++i) {
    if (!env_.obstacles[i]) {
      ++loc_size_;
    }
  }

  empty_locs_ = new int[loc_size_];
  loc_idxs_ = new int[env_.obstacles.size()];

  DEV_INFO("number of empty locations: {}", loc_size_);
  std::fill(loc_idxs_, loc_idxs_ + env_.obstacles.size(), -1);

  int loc_idx = 0;
  for (int loc = 0; loc < env_.obstacles.size(); ++loc) {
    if (!env.obstacles[loc]) {
      empty_locs_[loc_idx] = loc;
      loc_idxs_[loc] = loc_idx;
      ++loc_idx;
    }
  }
  ONLYDEV(assert(loc_idx == loc_size_);)

  state_size_ = loc_size_ * n_orientations_;
  main_heuristics_ = new double[loc_size_ * loc_size_];
  std::fill(main_heuristics_, main_heuristics_ + loc_size_ * loc_size_,
            MAX_HEURISTIC);
  // we keep start_loc, end_loc, start_orient, namely no goal_orient

  sub_heuristics_ = new double[state_size_ * loc_size_];
  full_heuristics_ = new double[state_size_ * state_size_];
  full_from_ = new int[state_size_ * state_size_];
}

HeuristicTable::~HeuristicTable() {
  delete[] empty_locs_;
  delete[] loc_idxs_;
  delete[] main_heuristics_;
  delete[] sub_heuristics_;
  delete[] full_heuristics_;
  delete[] full_from_;
}

// weights is an array of [loc_size*n_orientations]
void HeuristicTable::ParallelComputeWeightedHeuristics() {
  int n_threads = omp_get_max_threads();
  omp_set_num_threads(n_threads);
  cout << "number of threads used for heuristic computation: " << n_threads
       << endl;
  double *values = new double[n_threads * n_orientations_ * state_size_];
  heuristic::HeuristicSearch **planners =
      new heuristic::HeuristicSearch *[n_threads];
  for (int i = 0; i < n_threads; ++i) {
    planners[i] = new heuristic::HeuristicSearch(
        env_, n_orientations_, guidance_map_, action_model_, is_loaded_,
        energy_weight_, cost_type_);
  }

  int ctr = 0;
  int step = 100;
  auto start = std::chrono::steady_clock::now();
#pragma omp parallel for schedule(dynamic, 1)
  for (int loc_idx = 0; loc_idx < loc_size_; ++loc_idx) {
    int thread_id = omp_get_thread_num();
    int s_idx = thread_id * n_orientations_ * state_size_;
    ComputeWeightedHeuristics(loc_idx, values + s_idx, planners[thread_id]);
#pragma omp critical
    {
      ++ctr;
      if (ctr % step == 0) {
        auto end = std::chrono::steady_clock::now();
        double elapse = std::chrono::duration<double>(end - start).count();
        double estimated_remain = elapse / ctr * (loc_size_ - ctr);
        cout << ctr << "/" << loc_size_ << " completed in " << elapse
             << "s. estimated time to finish all: " << estimated_remain
             << "s.  estimated total time: " << (estimated_remain + elapse)
             << "s." << endl;
      }
    }
  }

  delete[] values;
  for (int i = 0; i < n_threads; ++i) {
    delete planners[i];
  }
  delete planners;
}

size_t HeuristicTable::GetFullID(const int &start_loc_idx,
                                 const int &start_orient, const int &loc_idx,
                                 const int &orient) const {
  return ((start_loc_idx * loc_size_ + loc_idx) * n_orientations_ +
          start_orient) * n_orientations_ + orient;
}

std::pair<int, int> HeuristicTable::GetCurrentPosLocFromFullID(
    int full_id) const {
  int orient = full_id % n_orientations_;
  int loc_idx = (full_id / n_orientations_ / n_orientations_) % loc_size_;
  return {empty_locs_[loc_idx], orient};
}

void HeuristicTable::ComputeWeightedHeuristics(
    int start_loc_idx, double *values, heuristic::HeuristicSearch *planner) {
  int start_loc = empty_locs_[start_loc_idx];

  std::fill(values, values + n_orientations_ * state_size_, MAX_HEURISTIC);
  for (int start_orient = 0; start_orient < n_orientations_; ++start_orient) {
    planner->Reset();

    planner->SearchForAll(start_loc, start_orient);
    for (int loc_idx = 0; loc_idx < loc_size_; ++loc_idx) {
      for (int orient = 0; orient < n_orientations_; ++orient) {
        int loc = empty_locs_[loc_idx];
        const heuristic::State *state =
            planner->all_states() + loc * n_orientations_ + orient;
        double cost = state->g;
        if (cost < 0) {
          cost = MAX_HEURISTIC;
        }
        if (cost > MAX_HEURISTIC) {
          std::cerr << "cost: " << cost << " > " << MAX_HEURISTIC << endl;
          exit(-1);
        }

        size_t value_idx =
            start_orient * state_size_ + loc_idx * n_orientations_ + orient;
        values[value_idx] = cost;

        size_t main_idx = start_loc_idx * loc_size_ + loc_idx;
        if (cost < main_heuristics_[main_idx]) {
          main_heuristics_[main_idx] = cost;
        }

        size_t full_idx =
            GetFullID(start_loc_idx, start_orient, loc_idx, orient);
        full_heuristics_[full_idx] = cost;
        if (state->prev) {
          full_from_[full_idx] =
              GetFullID(start_loc_idx, start_orient,
                        loc_idxs_[state->prev->pos], state->prev->orient);
          // std::cout << env_.Str(empty_locs_[loc_idxs_[state->prev->pos]],
          //                       state->prev->orient)
          //           << " -> " << env_.Str(empty_locs_[loc_idx], orient)
          //           << " " << cost << std::endl;
          // std::cout << loc_size_ << std::endl;
        } else {
          full_from_[full_idx] = -1;
        }
      }
    }
  }

  for (int start_orient = 0; start_orient < n_orientations_; ++start_orient) {
    for (int loc_idx = 0; loc_idx < loc_size_; ++loc_idx) {
      double cost = MAX_HEURISTIC;
      for (int orient = 0; orient < n_orientations_; ++orient) {
        size_t value_idx =
            start_orient * state_size_ + loc_idx * n_orientations_ + orient;
        auto value = values[value_idx];
        if (value < cost) {
          cost = value;
        }
      }
      size_t sub_idx = (start_loc_idx * loc_size_ + loc_idx) * n_orientations_ +
                       start_orient;
      size_t main_idx = start_loc_idx * loc_size_ + loc_idx;
      double diff = cost - main_heuristics_[main_idx];
      if (diff < 0) {
        std::cerr << "diff: " << diff << " < 0" << endl;
        exit(-1);
      }

      if (diff > MAX_HEURISTIC) {
        std::cerr << "diff: " << diff << " > " << MAX_HEURISTIC << endl;
        exit(-1);
      }
      sub_heuristics_[sub_idx] = diff;
    }
  }
}

// TODO add check
double HeuristicTable::Get(int loc1, int loc2) const {
  int loc_idx1 = loc_idxs_[loc1];
  int loc_idx2 = loc_idxs_[loc2];

  if (loc_idx1 == -1 || loc_idx2 == -1) {
    return MAX_HEURISTIC;
  }

  size_t idx = loc_idx1 * loc_size_ + loc_idx2;
  return main_heuristics_[idx];
}

double HeuristicTable::Get(int loc1, int orient1, int loc2) const {
  int loc_idx1 = loc_idxs_[loc1];
  int loc_idx2 = loc_idxs_[loc2];

  if (loc_idx1 == -1 || loc_idx2 == -1) {
    return MAX_HEURISTIC;
  }

  size_t main_idx = loc_idx1 * loc_size_ + loc_idx2;
  size_t sub_idx =
      (loc_idx1 * loc_size_ + loc_idx2) * n_orientations_ + orient1;
  return main_heuristics_[main_idx] + sub_heuristics_[sub_idx];
}

double HeuristicTable::Get(int loc1, int orient1, int loc2, int orient2) const {
  int loc_idx1 = loc_idxs_[loc1];
  int loc_idx2 = loc_idxs_[loc2];
  if (loc_idx1 == -1 || loc_idx2 == -1) {
    return MAX_HEURISTIC;
  }
  size_t full_idx = GetFullID(loc_idx1, orient1, loc_idx2, orient2);
  return full_heuristics_[full_idx];
}

std::pair<int, int> HeuristicTable::GetFrom(int start_loc, int start_orient,
                                            int current_loc,
                                            int current_orient) const {
  int loc_idx1 = loc_idxs_[start_loc];
  int loc_idx2 = loc_idxs_[current_loc];
  int from_id =
      full_from_[GetFullID(loc_idx1, start_orient, loc_idx2, current_orient)];
  if (from_id < 0)
    return {-1, -1};
  else
    return GetCurrentPosLocFromFullID(from_id);
}

void HeuristicTable::Preprocess(const std::string &suffix) {
  std::string fname = env_.map_name.substr(0, env_.map_name.size() - 4);
  std::string folder = env_.file_storage_path;
  if (folder[folder.size() - 1] !=
      boost::filesystem::path::preferred_separator) {
    folder += boost::filesystem::path::preferred_separator;
  }
  string fpath;

  fpath =
      folder + fname + "_weighted_heuristics_no_rotation_v4_" + suffix + ".gz";

  if (boost::filesystem::exists(fpath)) {
    Load(fpath);
  } else {
    ParallelComputeWeightedHeuristics();
    // ONLYDEV(save(fpath));
  }
}

void HeuristicTable::Save(const string &fpath) {
  std::ofstream fout;
  fout.open(fpath, std::ios::binary | std::ios::out);

  boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
  outbuf.push(boost::iostreams::zlib_compressor());
  outbuf.push(fout);

  std::ostream out(&outbuf);

  // save loc size
  out.write((char *)&loc_size_, sizeof(int));

  // save empty locs
  out.write((char *)empty_locs_, sizeof(int) * loc_size_);

  // save main heuristics
  out.write((char *)main_heuristics_, sizeof(double) * loc_size_ * loc_size_);

  // save sub heuristics

  out.write((char *)sub_heuristics_, sizeof(double) * state_size_ * loc_size_);
  out.write((char *)full_heuristics_,
            sizeof(double) * state_size_ * state_size_);

  boost::iostreams::close(outbuf);
  fout.close();
}

void HeuristicTable::Load(const string &fpath) {
  DEV_INFO("[start] load heuristics from {}.", fpath);
  std::ifstream fin;
  fin.open(fpath, std::ios::binary | std::ios::in);

  boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
  inbuf.push(boost::iostreams::zlib_decompressor());
  inbuf.push(fin);

  std::istream in(&inbuf);

  // load loc size
  int _loc_size;
  in.read((char *)&_loc_size, sizeof(int));

  // check loc size
  if (_loc_size != loc_size_) {
    std::cerr << "the sizes of empty locations don't match!" << endl;
    exit(-1);
  }

  // load empty locs
  int *_empty_locs = new int[loc_size_];
  in.read((char *)_empty_locs, sizeof(int) * loc_size_);

  // check empty locs
  for (auto i = 0; i < loc_size_; ++i) {
    if (_empty_locs[i] != empty_locs_[i]) {
      std::cerr << "the empty locations don't match!" << endl;
      exit(-1);
    }
  }

  // load main heurisitcs
  in.read((char *)main_heuristics_, sizeof(double) * loc_size_ * loc_size_);

  // load sub heuristics
  in.read((char *)sub_heuristics_, sizeof(double) * state_size_ * loc_size_);
  in.read((char *)full_heuristics_, sizeof(double) * state_size_ * state_size_);
}
};  // namespace planner
