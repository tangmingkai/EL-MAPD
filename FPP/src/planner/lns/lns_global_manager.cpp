#include "planner/lns/lns_global_manager.h"

#include <cstdlib>
#include <vector>
#include <tuple>
#include <string>
#include "omp.h"

namespace planner {

namespace lns {

LNSGlobalManager::LNSGlobalManager(
    const Environment &env, const ActionModel &action_model,
    const CostCalculator &cost_calculator, const MLAStar &single_agent_solver,
    const int &neighbor_size, const int &thread_num,
    const double &reaction_factor, const double &decay_factor,
    const double &recharge_threshold, const int &lns_mode, const uint &seed)
    : seed_(seed) {
  thread_num_ = thread_num;
  for (auto i = 0; i < thread_num_; ++i) {
    local_optimizers_.push_back(LNSLocalOptimizer(
        env, action_model, cost_calculator, single_agent_solver, neighbor_size,
        thread_num, i, reaction_factor, decay_factor, recharge_threshold,
        lns_mode, seed + i * 10 + 1));
  }
  updating_queues_.resize(thread_num_);
  neighbor_pool_.resize(thread_num_);
  for (int i = 0; i < thread_num_; i++)
    mtx_for_updating_queues_.push_back(std::make_unique<std::mutex>());
}

bool LNSGlobalManager::ParallelALNSOptimizationLayer(
    const TimeLimiter &time_limiter, std::vector<PathPlan> *path_plans,
    CAT *cat, std::vector<int> *charge_point_allocated_agent_id) {
  if (time_limiter.Timeout()) return true;
  double original_sum_of_cost = 0;
  for (auto& plan : *path_plans) {
    original_sum_of_cost += plan.main_cost;
  }
  ParallelOptimize(time_limiter, original_sum_of_cost, *path_plans, *cat,
                   *charge_point_allocated_agent_id);
  double min_cost = original_sum_of_cost;
  int id = -1;
  for (auto i = 0; i < local_optimizers_.size(); i++) {
    if (min_cost > local_optimizers_[i].total_cost()) {
      min_cost = local_optimizers_[i].total_cost();
      id = i;
    }
  }
  if (id != -1) {
    local_optimizers_[id].MoveResult(path_plans, cat,
                                     charge_point_allocated_agent_id);
  }
  return true;
}

bool LNSGlobalManager::ParallelOptimize(
    const TimeLimiter& time_limiter,
    const double &original_sum_of_cost,
    const std::vector<PathPlan>& path_plans,
    const CAT &cat,
    const std::vector<int>& charge_point_allocated_agent_id) {
#pragma omp parallel for
  for (int i = 0; i < thread_num_; ++i) {
    local_optimizers_[i].Init(original_sum_of_cost, path_plans, cat,
                              charge_point_allocated_agent_id);
    updating_queues_[i].clear();
  }
#pragma omp parallel for
  for (int i = 0; i < thread_num_; ++i) {
    neighbor_pool_[i].Reset();
    while (true) {
      if (time_limiter.Timeout()) break;
      local_optimizers_[i].Generate(time_limiter, &neighbor_pool_[i]);
      if (time_limiter.Timeout()) break;
#pragma omp critical
      {
        if (!time_limiter.Timeout()) {
          if (neighbor_pool_[i].succ) {
            for (int j = 0; j < thread_num_; ++j) {
              if (j == i) continue;
              mtx_for_updating_queues_[j]->lock();
              updating_queues_[j].push_back(neighbor_pool_[i]);
              mtx_for_updating_queues_[j]->unlock();
            }
          }
        }
      }
      if (time_limiter.Timeout()) break;
      mtx_for_updating_queues_[i]->lock();
      auto updating_queue = updating_queues_[i];
      updating_queues_[i].clear();
      mtx_for_updating_queues_[i]->unlock();
      for (auto& neighbor : updating_queue) {
        if (time_limiter.Timeout()) break;
        local_optimizers_[i].Update(neighbor);
      }
    }
  }
  return true;
}

void LNSGlobalManager::GetLnsInfo(
    std::vector<tuple<string, int, int>> *global_lns_infos) const {
  global_lns_infos->clear();
  if (!local_optimizers_.empty()) {
    auto sampler_names = local_optimizers_[0].GetSamplerNames();
    global_lns_infos->resize(sampler_names.size());
    for (int i = 0; i < sampler_names.size(); i++) {
      (*global_lns_infos)[i] = {sampler_names[i], 0, 0};
    }
    for (int i = 0; i < local_optimizers_.size(); i++) {
      for (int j = 0; j < sampler_names.size(); j++) {
        std::get<1>((*global_lns_infos)[j]) +=
            local_optimizers_[i].lns_infos()[j].first;
        std::get<2>((*global_lns_infos)[j]) +=
            local_optimizers_[i].lns_infos()[j].second;
      }
    }
  }
}
}  // namespace lns
}  // namespace planner
