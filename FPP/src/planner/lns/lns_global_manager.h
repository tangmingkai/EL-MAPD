#pragma once
#include <memory>
#include <vector>
#include <utility>
#include "planner/cost_calculator/cost_calculator.h"
#include "planner/lns/lns_local_optimizer.h"

namespace planner {

namespace lns {

class LNSGlobalManager {
 public:
  LNSGlobalManager(const Environment &env, const ActionModel &action_model,
                   const CostCalculator &cost_calculator,
                   const MLAStar &single_agent_solver, const int &neighbor_size,
                   const int &thread_num, const double &reaction_factor,
                   const double &decay_factor,
                   const double &recharge_threshold,
                   const int &lns_mode,
                   const uint &seed);

  ~LNSGlobalManager() = default;
  bool ParallelALNSOptimizationLayer(
      const TimeLimiter &time_limiter, std::vector<PathPlan> *path_plans,
      CAT *cat, std::vector<int> *charge_point_allocated_agent_id);
  void GetLnsInfo(std::vector<tuple<string, int, int>> *global_lns_infos) const;

 private:
  void Reset();
  bool ParallelOptimize(
      const TimeLimiter& time_limiter, const double& original_sum_of_cost,
      const std::vector<PathPlan>& path_plans, const CAT& cat,
      const std::vector<int>& charge_point_allocated_agent_id);
  std::vector<LNSLocalOptimizer> local_optimizers_;
  std::vector<std::vector<Neighbor>> updating_queues_;
  std::vector<std::unique_ptr<std::mutex>> mtx_for_updating_queues_;
  std::vector<Neighbor> neighbor_pool_;
  std::vector<std::pair<int,int>> lns_infos_;
  double decay_factor_;
  double reaction_factor_;
  unsigned int seed_;
  int thread_num_;
};

}  // namespace lns

}  // namespace planner
