#pragma once
#include <vector>
#include <string>
#include <utility>
#include "planner/cat/cat.h"
#include "planner/lns/agent_sampler/base_agent_sampler.h"
#include "planner/multi_label_a_star/multi_label_a_star.h"
#include "planner/cost_calculator/cost_calculator.h"
namespace planner {

namespace lns {

class LNSLocalOptimizer {
 public:
  LNSLocalOptimizer(const Environment &env, const ActionModel &action_model,
                    const CostCalculator &cost_calculator,
                    const MLAStar &single_agent_solver,
                    const int &neighbor_size, const int &thread_num,
                    const int &thread_id, const double &reaction_factor,
                    const double &decay_factor,
                    const double &recharge_threshold,
                    const int &lns_mode,
                    const uint &seed);
  void Update(const Neighbor& neighbor);
  void Init(const double &original_sum_of_cost,
            const std::vector<PathPlan> &path_plans, const CAT &cat,
            const std::vector<int> &charge_point_allocated_agent_id);
  void Reset();
  void Generate(const TimeLimiter &time_limiter,
                Neighbor *neighbor);
  double total_cost() const { return total_cost_; }
  // const std::vector<PathPlan> &path_plans() const { return path_plans_; }
  // const std::vector<int> &charge_point_allocated_agent_id() const {
  //   return charge_point_allocated_agent_id_;
  // }
  // const CAT& cat() { return cat_;}
  void MoveResult(std::vector<PathPlan>* path_plans,
                  CAT *cat,
                  std::vector<int>* charge_point_allocated_agent_id);
  const std::vector<pair<int, int>> &lns_infos() const { return lns_infos_; }
  std::vector<string> GetSamplerNames() const {
    std::vector<string> sampler_names;
    for (auto &sampler : agent_samplers_) {
      sampler_names.push_back(sampler->name());
    }
    return sampler_names;
  }

 private:
  void UpdateWeight(const int &sampler_id,
                    const bool &succ,
                    const double &old_sum_of_cost,
                       const double &new_sum_of_cost,
                                     const int &neighbor_agent_size);
  void UpdateSamplers(const Neighbor &neighbor);
  void SampleAgents(const TimeLimiter &time_limiter,
                                     Neighbor *neighbor);
  void GetNextActions(std::vector<Action>* actions) const;
  bool PriorityPlanning(const TimeLimiter &time_limiter,
                        const std::vector<PathPlan> &old_path_plans,
                        std::vector<int> *replan_agent_ids,
                        std::vector<PathPlan> *new_path_plans);
  std::vector<std::pair<int, int>> lns_infos_;
  const Environment &env_;
  const CostCalculator &cost_calculator_;
  const MLAStar &single_agent_solver_;
  int thread_id_;
  double reaction_factor_;
  double decay_factor_;
  uint seed_;
  std::vector<PathPlan> path_plans_;
  std::vector<double> sampler_weights_;
  std::vector<BaseAgentSamplerPtr> agent_samplers_;
  std::vector<int> charge_point_allocated_agent_id_;
  double total_cost_;
  double recharge_threshold_;
  CAT cat_;
  std::mt19937 mt_;
};
}

}