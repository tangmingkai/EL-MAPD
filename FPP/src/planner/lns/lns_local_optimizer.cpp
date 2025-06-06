#include "planner/lns/lns_local_optimizer.h"

#include <memory>
#include <algorithm>
#include <utility>
#include "planner/lns/agent_sampler/intersection_sampler.h"
#include "planner/lns/agent_sampler/random_agent_sampler.h"
#include "planner/lns/agent_sampler/random_walk_sampler.h"
#include "planner/lns/agent_sampler/optimal_path_sampler.h"
#include "planner/lns/agent_sampler/charge_agent_sampler.h"
#include "planner/utils/utils.h"

namespace planner {

namespace lns {

LNSLocalOptimizer::LNSLocalOptimizer(const Environment &env,
                                     const ActionModel &action_model,
                                     const CostCalculator &cost_calculator,
                                     const MLAStar &single_agent_solver,
                                     const int &neighbor_size,
                                     const int &thread_num,
                                     const int &thread_id,
                                     const double &reaction_factor,
                                     const double &decay_factor,
                                     const double &recharge_threshold,
                                     const int &lns_mode,
                                     const uint &seed)
    : env_(env),
      cost_calculator_(cost_calculator),
      single_agent_solver_(single_agent_solver),
      thread_id_(thread_id),
      reaction_factor_(reaction_factor),
      decay_factor_(decay_factor),
      recharge_threshold_(recharge_threshold),
      seed_(seed),
      mt_(seed) {
  if (lns_mode == 0 || lns_mode == 1 || lns_mode == 3 || lns_mode == 4) {
    agent_samplers_.push_back(std::make_shared<RandomWalkSampler>(
        env, action_model, cost_calculator, neighbor_size, thread_num,
        recharge_threshold, seed * 1234));
    agent_samplers_.push_back(std::make_shared<IntersectionSampler>(
        env, action_model, cost_calculator, neighbor_size, thread_num,
        recharge_threshold, seed * 1234 + 1));
    agent_samplers_.push_back(std::make_shared<RandomAgentSampler>(
        env, action_model, cost_calculator, neighbor_size, thread_num,
        recharge_threshold, seed * 1234 + 2));
  }
  if (lns_mode == 0 || lns_mode == 2 || lns_mode == 3) {
    agent_samplers_.push_back(std::make_shared<OptimalPathSampler>(
        env, action_model, cost_calculator, neighbor_size, thread_num,
        recharge_threshold, seed * 1234 + 3));
  }
  if (lns_mode == 0 || lns_mode == 2 || lns_mode == 4) {
    agent_samplers_.push_back(std::make_shared<ChargeAgentSampler>(
        env, action_model, cost_calculator, neighbor_size, thread_num,
        recharge_threshold, seed * 1234 + 4));
  }
  lns_infos_.resize(agent_samplers_.size(), {0, 0});
  sampler_weights_.assign(agent_samplers_.size(), 1);
}

void LNSLocalOptimizer::Init(const double &original_sum_of_cost,
            const std::vector<PathPlan> &path_plans, const CAT &cat,
            const std::vector<int> &charge_point_allocated_agent_id) {
  total_cost_ = original_sum_of_cost;
  path_plans_ = path_plans;
  cat_ = cat;
  charge_point_allocated_agent_id_ = charge_point_allocated_agent_id;
  // sampler_weights_.assign(agent_samplers_.size(), 1);
  for (auto &agent_sampler : agent_samplers_) {
    agent_sampler->Reset();
  }
}

void LNSLocalOptimizer::UpdateSamplers(const Neighbor &neighbor) {
  for (auto &agent_sampler : agent_samplers_) {
    agent_sampler->HaveUpdate(neighbor);
  }
}

void LNSLocalOptimizer::Update(const Neighbor &neighbor) {
  if (!neighbor.succ) return;
  if (neighbor.agent_ids.size() != neighbor.path_plans.size()) {
    std::cout << "Why neighbor.agent_ids.size()!=neighbor.path_plans.size()"
              << std::endl;
    exit(-1);
  }
  double new_sum_of_cost = total_cost_;
  for (auto &agent_id : neighbor.agent_ids) {
    new_sum_of_cost -= path_plans_[agent_id].main_cost;
  }
  for (auto &plan : neighbor.path_plans) {
    new_sum_of_cost += plan.main_cost;
  }
  if (new_sum_of_cost >= total_cost_) return;
  for (auto &plan : neighbor.path_plans) {
    if (!cat_.IsVaild(plan.path[plan.current_id], neighbor.agent_ids)) return;
    for (int i = plan.current_id + 1; i < plan.path.size(); i++) {
      if (!cat_.IsVaild(plan.path[i - 1], plan.path[i], neighbor.agent_ids))
        return;
    }
    int charge_agent_id =
        charge_point_allocated_agent_id_[plan.charge_point_id];
    if (charge_agent_id != -1) {
      bool is_safe = false;
      for (auto agent_id : neighbor.agent_ids) {
        if (agent_id == charge_agent_id) {
          is_safe = true;
          break;
        }
      }
      if (!is_safe) return;
    }
  }

  total_cost_ = new_sum_of_cost;
  for (auto &id : neighbor.agent_ids) {
    if (id >=  path_plans_.size()) {
      std::cout << "Error1:" << id << " " << path_plans_.size() << std::endl;
      exit(-1);
    }
    Utils::DeleteOldPathPlan(path_plans_[id], id, &cat_,
                             &charge_point_allocated_agent_id_);
  }
  for (int i = 0; i < neighbor.agent_ids.size(); i++) {
    Utils::AddNewPathPlan(neighbor.path_plans[i], neighbor.agent_ids[i], &cat_,
                          &charge_point_allocated_agent_id_);
    path_plans_[neighbor.agent_ids[i]] = neighbor.path_plans[i];
  }
  UpdateSamplers(neighbor);
}

void LNSLocalOptimizer::Generate(const TimeLimiter &time_limiter,
                                 Neighbor *neighbor) {
  neighbor->Reset();
  SampleAgents(time_limiter, neighbor);
  if (time_limiter.Timeout()) {
    return;
  }
  bool succ = PriorityPlanning(time_limiter, path_plans_,
                               &(neighbor->agent_ids), &(neighbor->path_plans));
  if (!succ) {
    for (int j = 0; j < neighbor->path_plans.size(); j++) {
      Utils::DeleteOldPathPlan(neighbor->path_plans[j], neighbor->agent_ids[j],
                               &cat_, &charge_point_allocated_agent_id_);
    }
    for (int j = 0; j < neighbor->agent_ids.size(); j++) {
      Utils::AddNewPathPlan(path_plans_[neighbor->agent_ids[j]],
                            neighbor->agent_ids[j], &cat_,
                            &charge_point_allocated_agent_id_);
    }
    UpdateWeight(neighbor->sampler_id, false, 0, 0, 0);
    return;
  }
  if (neighbor->agent_ids.size() != neighbor->path_plans.size()) {
    std::cout << "Why2 neighbor.agent_ids.size()!=neighbor.path_plans.size()"
              << std::endl;
    exit(-1);
  }
  double new_sum_of_cost = total_cost_;
  for (auto &agent_id : neighbor->agent_ids) {
    new_sum_of_cost -= path_plans_[agent_id].main_cost;
  }
  for (auto &plan : neighbor->path_plans) {
    new_sum_of_cost += plan.main_cost;
  }
  if (new_sum_of_cost >= total_cost_) {
    for (int i = 0; i < neighbor->agent_ids.size(); i++) {
      Utils::DeleteOldPathPlan(neighbor->path_plans[i], neighbor->agent_ids[i],
                               &cat_, &charge_point_allocated_agent_id_);
    }
    for (int i = 0; i < neighbor->agent_ids.size(); i++) {
      Utils::AddNewPathPlan(path_plans_[neighbor->agent_ids[i]],
                            neighbor->agent_ids[i], &cat_,
                            &charge_point_allocated_agent_id_);
    }
    UpdateWeight(neighbor->sampler_id, false, 0, 0, 0);
    return;
  }
  UpdateWeight(neighbor->sampler_id, true, total_cost_, new_sum_of_cost,
               neighbor->agent_ids.size());
  total_cost_ = new_sum_of_cost;
  for (int i = 0; i < neighbor->agent_ids.size(); i++) {
    path_plans_[neighbor->agent_ids[i]] = neighbor->path_plans[i];
  }
  UpdateSamplers(*neighbor);
  neighbor->succ = true;
  lns_infos_[neighbor->sampler_id].second++;
}

void LNSLocalOptimizer::SampleAgents(const TimeLimiter &time_limiter,
                                     Neighbor *neighbor) {
  bool succ = false;
  while (!succ) {
    int sampler_id = RouletteWheel(sampler_weights_, &seed_);
    if (time_limiter.Timeout()) return;
    lns_infos_[sampler_id].first++;
    neighbor->sampler_id = sampler_id;
    succ = agent_samplers_[sampler_id]->Sample(time_limiter, path_plans_,
                                               thread_id_, cat_, neighbor);
    if (succ == false) {
      UpdateWeight(sampler_id, false, 0, 0, 0);
    }
  }
}

bool LNSLocalOptimizer::PriorityPlanning(
    const TimeLimiter &time_limiter,
    const std::vector<PathPlan> &old_path_plans,
    std::vector<int> *replan_agent_ids, std::vector<PathPlan> *new_path_plans) {
  std::shuffle(replan_agent_ids->begin(), replan_agent_ids->end(), mt_);
  for (auto id : *replan_agent_ids) {
    Utils::DeleteOldPathPlan(old_path_plans[id], id, &cat_,
                             &charge_point_allocated_agent_id_);
  }
  for (int i = 0; i < replan_agent_ids->size(); i++) {
    int id = (*replan_agent_ids)[i];
    if (time_limiter.Timeout()) return false;
    auto &old_path_plan = old_path_plans[id];
    auto &current_state = old_path_plan.path[old_path_plan.current_id];
    double min_cost = MAX_HEURISTIC;
    int charge_point_id = -1;
    if (old_path_plan.IsPassedDelivery())
      charge_point_id = cost_calculator_.SelectChargeID(
          charge_point_allocated_agent_id_, id, current_state.location,
          current_state.orientation);
    else
      charge_point_id = cost_calculator_.SelectChargeID(
        charge_point_allocated_agent_id_, id, old_path_plan.DeliveryLoc());
    if (charge_point_id < 0) {
      std::cout << "Why not enough charge point" << std::endl;
      exit(-1);
    }
    if (time_limiter.Timeout()) return false;
    std::vector<int> goals{old_path_plan.PickupLoc(),
                           old_path_plan.DeliveryLoc(),
                           env_.charges_locs[charge_point_id]};
    std::vector<std::vector<State>> paths;
    PathPlan plan;
    bool succ = single_agent_solver_.RechargeAndFallbackPlanningLayer(
        time_limiter, LState(old_path_plan.GetCurrentStage(), current_state),
        cat_, goals, old_path_plan.status, {}, recharge_threshold_, &plan);
    if (!succ) {
      return false;
    } else {
      Utils::AddNewPathPlan(plan, id, &cat_, &charge_point_allocated_agent_id_);
      new_path_plans->push_back(plan);
    }
  }
  return true;
}

void LNSLocalOptimizer::UpdateWeight(const int &sampler_id,
                                     const bool &succ,
                                     const double &old_sum_of_cost,
                                     const double &new_sum_of_cost,
                                     const int &neighbor_agent_size) {
  if (succ && old_sum_of_cost > new_sum_of_cost) {
    sampler_weights_[sampler_id] =
        reaction_factor_ *
            std::max((old_sum_of_cost - new_sum_of_cost) / neighbor_agent_size,
                     1.0) +
        (1 - reaction_factor_) * sampler_weights_[sampler_id];

  } else {
    sampler_weights_[sampler_id] =
        (1 - decay_factor_) * sampler_weights_[sampler_id];
  }
}

void LNSLocalOptimizer::MoveResult(std::vector<PathPlan>* path_plans,
                  CAT *cat,
                  std::vector<int>* charge_point_allocated_agent_id) {
  (*path_plans) = path_plans_;
  (*cat) = cat_;
  (*charge_point_allocated_agent_id) =
      charge_point_allocated_agent_id_;
}

}  // namespace lns
}  // namespace planner
