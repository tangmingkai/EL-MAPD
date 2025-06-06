#include <signal.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <climits>
#include <memory>
#include <stdexcept>


#include "common/nlohmann/json.hpp"
#include "env/environment.h"
#include "model/action_model.h"
#include "simulator/simulator.h"

#include <execinfo.h>
#include <signal.h>
namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;
std::unique_ptr<Simulator> simulator;

int main(int argc, char **argv) {
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "inputFile,i", po::value<std::string>()->required(), "input file name")(
      "outputFile,o", po::value<std::string>()->default_value("./test.json"),
          "output file name")
      ("simulationTime", po::value<int>()->default_value(5000),
          "run simulation")(
      "fileStoragePath", po::value<std::string>()->default_value(""),
          "the path to the storage path")
      ("planningTimeLimit", po::value<double>()->default_value(1),
          "the time limit for planner in second")
      ("cutoffTimeLimit", po::value<double>()->default_value(-1),
          "the time limit for cutoff in second")
      ("preprocessTimeLimit", po::value<double>()->default_value(20000.0),
          "the time limit for preprocessing in second")
      ("mapWeightsPath", po::value<std::string>()->default_value(""),
          "map_weights_path")
      ("rechargeThresholdFactor", po::value<double>()->default_value(1.0),
          "rechargeThresholdFactor")
      ("energyWeight", po::value<double>()->default_value(0.0),
          "the weight of energy")
      ("lnsMode", po::value<int>()->default_value(1),
          "0:full; 1:original; 2:new two lns; 3:original + "
          "lns1; 4:original + lns2; -1: none")
      ("seed", po::value<uint>()->default_value(6), "random seed")
      ("checkWellFormed", po::value<bool>()->default_value(false),
          "checkWellFormed");
  clock_t start_time = clock();
  po::store(po::parse_command_line(argc, argv, desc), vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }po::value<uint>()->default_value(6),
  po::notify(vm);
  boost::filesystem::path p(vm["inputFile"].as<std::string>());
  boost::filesystem::path dir = p.parent_path();
  std::string base_folder = dir.string();
  if (base_folder.size() > 0 && base_folder.back() != '/') {
    base_folder += "/";
  }
  ONLYDEV(g_logger.Init("logs/run", debug);)

  auto input_json_file = vm["inputFile"].as<std::string>();
  json data;
  std::ifstream f(input_json_file);
  if (!f.is_open()) {
    std::cerr << "Input file not exist" << std::endl;
    exit(1);
  }
  try {
    data = json::parse(f);
  } catch (json::parse_error error) {
    std::cerr << "Failed to load " << input_json_file << std::endl;
    std::cerr << "Message: " << error.what() << std::endl;
    exit(1);
  }

  auto map_path = read_param_json<std::string>(data, "mapFile");

  Environment* env = new Environment();
  env->Load(base_folder + map_path, vm["fileStoragePath"].as<std::string>());

  double preprocess_time_limit = vm["preprocessTimeLimit"].as<double>();
  double planning_time_limit = vm["planningTimeLimit"].as<double>();
  double idle_comsumption =
      read_param_json<double>(data, "idleComsumption") * planning_time_limit;
  double active_unloaded_comsumption =
      read_param_json<double>(data, "activeUnloadedComsumption") *
      planning_time_limit;
  double active_loaded_comsumption =
      read_param_json<double>(data, "activeLoadedComsumption") *
      planning_time_limit;
  double charge_energy_per_timestep =
      read_param_json<double>(data, "chargeEnergyPerTimestep") *
      planning_time_limit;
  double cutoff_time_limit = vm["cutoffTimeLimit"].as<double>();
  if (cutoff_time_limit <= 0) {
    cutoff_time_limit = planning_time_limit - 0.15;
  }
  // double preprocess_time_limit = vm["preprocessTimeLimit"].as<double>();
  // double planning_time_limit = vm["planTimeLimit"].as<double>();
  // double idle_comsumption =
  //     read_param_json<double>(data, "idleComsumption");
  // double active_unloaded_comsumption =
  //     read_param_json<double>(data, "activeUnloadedComsumption");
  // double active_loaded_comsumption =
  //     read_param_json<double>(data, "activeLoadedComsumption");
  // double charge_energy_per_timestep =
  //     read_param_json<double>(data, "chargeEnergyPerTimestep");

  double full_energy = read_param_json<double>(data, "fullEnergy");  // 10000
  double p_idle = read_param_json<double>(data, "pIdle");      // 0.0
  double energy_weight = vm["energyWeight"].as<double>();   // 0.5
  ActionModel *action_model = new ActionModel(*env, idle_comsumption,
              active_unloaded_comsumption, active_loaded_comsumption,
              charge_energy_per_timestep, full_energy);
  int team_size = read_param_json<int>(data, "teamSize");

  std::vector<State> start_states = read_start_vec(
      base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
  std::vector<Task> tasks = read_task_vec(
      base_folder + read_param_json<std::string>(data, "taskFile"));
  if (start_states.size() > tasks.size()) {
    DEV_WARN(
        "Not enough tasks for robots (number of tasks < team size)");
  }

  std::string map_weights_path = vm["mapWeightsPath"].as<std::string>();

  double recharge_threshold_factor = vm["rechargeThresholdFactor"].as<double>();
  int lns_mode = vm["lnsMode"].as<int>();
  bool check_well_formed = vm["checkWellFormed"].as<bool>();
  uint seed = vm["seed"].as<uint>();
  simulator = std::make_unique<Simulator>(
      *env, *action_model, tasks, start_states, preprocess_time_limit,
      planning_time_limit, cutoff_time_limit, p_idle, input_json_file,
      map_weights_path, energy_weight, recharge_threshold_factor, lns_mode,
      seed, check_well_formed);

  simulator->Simulate(vm["simulationTime"].as<int>());
  simulator->SaveResults(vm["outputFile"].as<std::string>());
  puts("Finished");

  delete action_model;
  delete env;
  _exit(0);
}
