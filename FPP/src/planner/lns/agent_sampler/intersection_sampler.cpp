#include "planner/lns/agent_sampler/intersection_sampler.h"
#include <queue>

namespace planner {
namespace lns {
IntersectionSampler::IntersectionSampler(const Environment &env,
                    const ActionModel &action_model,
                    const CostCalculator &cost_calculator,
                    const int &neighbor_size,
                    const int &thread_num,
                    const double &recharge_threshold,
                    const uint &seed):
                    env_(env),
                    neighbor_size_(neighbor_size),
                    seed_(seed),
                    mt_(seed) {
  const int dirx[] = {0, 1, 0, -1};
  const int diry[] = {1, 0, -1, 0};
  intersection_locs_.clear();
  is_intersections_.resize(env.rows*env.cols);
  for (int i = 0; i < env.rows*env.cols; i++) {
    is_intersections_[i] = false;
    if (env.obstacles[i] == true) continue;
    int degree = 0;
    auto xy = env.GetXY(i);
    for (int k = 0; k < 4; k++) {
      int x1 = xy.first + dirx[k];
      int y1 = xy.first + diry[k];
      if (env.IsOutOfBoundary(x1, y1)) continue;
      int loc1 = env.GetLoc(x1, y1);
      if (env.obstacles[loc1] == true) continue;
      degree++;
    }
    if (degree > 2) {
      intersection_locs_.push_back(i);
      is_intersections_[i] = true;
    }
  }
}

void IntersectionSampler::Reset() {
  has_update_ = true;
}

void IntersectionSampler::HaveUpdate(const Neighbor &neighbor)  {
  has_update_ = true;
}
bool IntersectionSampler::Sample(const TimeLimiter &time_limiter,
                                 const std::vector<PathPlan> &path_plans,
                                 const int &thread_id, const CAT &cat,
                                 Neighbor *neighbor) {
  if (has_update_) {
    good_start_intersections_locs_.clear();
    for (int i = 0; i < intersection_locs_.size(); i++) {
      if (cat.GetAgentNumber(intersection_locs_[i]) >= 1) {
        good_start_intersections_locs_.emplace_back(intersection_locs_[i]);
      }
    }
    if (time_limiter.Timeout()) return false;
    has_update_ = false;
  }

  if (good_start_intersections_locs_.empty()) return false;
  const int dirx[] = {0, 1, 0, -1};
  const int diry[] = {1, 0, -1, 0};
  std::unordered_set<int> neighbors_set;
  int location = good_start_intersections_locs_
      [rand_r(&seed_) % good_start_intersections_locs_.size()];
  cat.AddLocaionAgents(location, &neighbors_set);
  if (neighbors_set.size() == 0) {
    std::cout << "neighbors_set.size() = 0" << std::endl;
    exit(-1);
  }

  if (neighbors_set.size() < neighbor_size_) {
    std::unordered_set<int> closed;
    std::queue<int> open;
    open.push(location);
    while (!open.empty() && neighbors_set.size() < neighbor_size_) {
      int curr = open.front();
      closed.insert(curr);
      open.pop();
      for (int k = 0; k < 4; k++) {
        int x1 = curr % env_.cols + dirx[k];
        int y1 = curr / env_.rows + diry[k];
        if (env_.IsOutOfBoundary(x1, y1)) continue;
        int loc1 = env_.GetLoc(x1, y1);
        if (closed.find(loc1) != closed.end()) continue;
        if (is_intersections_[loc1] == false) continue;
        cat.AddLocaionAgents(loc1, &neighbors_set);
        if (neighbors_set.size() >= neighbor_size_) break;
        open.push(loc1);
      }
    }
  }
  neighbor->agent_ids.clear();
  for (auto id : neighbors_set) {
    if (!path_plans[id].IsIdle()) {
      neighbor->agent_ids.push_back(id);
    }
  }
  if (neighbor->agent_ids.size() > neighbor_size_) {
    std::shuffle(neighbor->agent_ids.begin(), neighbor->agent_ids.end(), mt_);
    neighbor->agent_ids.resize(neighbor_size_);
  }
  return neighbor->agent_ids.size()>=2;
}
}  // namespace lns
}  // namespace planner
