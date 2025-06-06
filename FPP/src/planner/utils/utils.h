#include <utility>
#include <vector>
#include <tuple>

#include "planner/multi_label_a_star/mlastar_node.h"
namespace planner {
class Utils {
 public:
  static void GetVaildNeighbors(
    const Environment &env,
    const CostCalculator &cost_calculator,
    const ActionModel &action_model,
    const MLAStarNode &current, const CAT &cat, const std::vector<int> &goals,
    const std::vector<double> &acc_main_heuristic,
    const std::vector<double> &acc_energy_heuristic,
    const std::vector<int> &ignore_agent_ids,
    std::vector<std::pair<MLAStarNode, Action>> *vaild_next_nodes);
  static void DeleteOldPathPlan(
      const PathPlan &old_plan, const int &agent_id, CAT *cat,
      std::vector<int> *charge_point_allocated_agent_id);
  static void AddNewPathPlan(const PathPlan &new_plan, const int &agent_id,
                             CAT *cat,
                             std::vector<int> *charge_point_allocated_agent_id);
  static void PriorityPlanning(
      const TimeLimiter &time_limiter, const int &time_step,
      const std::vector<Agent> &agents, std::vector<int> *replan_agent_ids,
      std::vector<std::tuple<int, int, int, int, int>> *agent_goal_infos);
  static void ConstructPureRechargePlan(const State &current_state,
                                        const int &charge_point_id,
                                        const ActionModel &action_model,
                                        PathPlan *plan);
  // static void ConstructInitialPlan(const State &current_state,
  //                                const int& charge_point_id,
  //                                PathPlan* plan);
  static double GetActionCostByType(const CostType &cost_type,
                                    const bool &is_loaded, const int &location,
                                    const int &orientation,
                                    const Action &action,
                                    const ActionModel &action_model,
                                    const GuidanceMap &guidance_map,
                                    const double &energy_weight);
  static double GetActionMainCost(const bool &is_loaded, const int &location,
                                  const int &orientation, const Action &action,
                                  const ActionModel &action_model,
                                  const GuidanceMap &guidance_map,
                                  const double &energy_weight);
  static double GetActionPureEnergy(const bool &is_loaded, const int &location,
                                    const int &orientation,
                                    const Action &action,
                                    const ActionModel &action_model);
  static void ConstructOptimalPathAndAction(
    const CostCalculator &cost_calculator, const ActionModel &action_model,
    const LState &start_state, const std::vector<int> &goals,
    const HeuristicTable &unloaded_heuristic_table,
    const HeuristicTable &loaded_heuristic_table,
    const bool &enable_charge, PathPlan *plan);

 private:
  static void AppendOptimalPathAndAction(const int &start_loc,
                                       const int &start_ori, const int &end_loc,
                                       const int &end_ori,
                                       const HeuristicTable &heuristic_table,
                                       const ActionModel &action_model,
                                       PathPlan *plan);
};
}  // namespace planner
