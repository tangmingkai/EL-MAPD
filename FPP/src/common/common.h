#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/tokenizer.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <iostream>
#include <list>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>
#include <string>

#include "common/nlohmann/json.hpp"
#include "common/state.h"
#include "common/tasks.h"

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::fibonacci_heap;

using std::cout;
using std::deque;
using std::endl;
using std::list;
using std::make_pair;
using std::make_tuple;
using std::max;
using std::min;
using std::ostream;
using std::pair;
using std::priority_queue;
using std::string;
using std::tuple;
using std::vector;

////////////////////////////////////////////////////
inline std::vector<State> read_start_vec(string fname, int team_size) {
  std::vector<State> res;
  string line;
  std::ifstream myfile(fname.c_str());
  if (!myfile.is_open()) return {};

  getline(myfile, line);
  while (!myfile.eof() && line[0] == '#') {
    getline(myfile, line);
  }

  boost::char_separator<char> sep(" ");
  boost::tokenizer<boost::char_separator<char>> tok(line, sep);
  boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

  int max_team_size = atoi((*beg).c_str());
  if (max_team_size < team_size) {
    std::cerr << "Input file wrong, no enough agents in agent file";
    exit(-1);
  }
  // My benchmark
  for (int i = 0; i < team_size; i++) {
    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
      getline(myfile, line);
    }
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    int loc = atoi((*beg).c_str());
    beg++;
    if (beg == tok.end()) {
      std::cerr << "Input file wrong. No initial orient";
      exit(-1);
    }
    int orient = atoi((*beg).c_str());
    beg++;
    if (beg == tok.end()) {
      std::cerr << "Input file wrong. No initial energy";
      exit(-1);
    }
    int energy = atoi((*beg).c_str());
    res.push_back(State(loc, 0, orient, energy, false));
  }
  myfile.close();

  return res;
}

inline std::vector<Task> read_task_vec(string fname) {
  std::vector<Task> res;
  string line;
  std::ifstream myfile(fname.c_str());
  if (!myfile.is_open()) return {};

  getline(myfile, line);
  while (!myfile.eof() && line[0] == '#') {
    getline(myfile, line);
  }

  boost::char_separator<char> sep(" ");
  boost::tokenizer<boost::char_separator<char>> tok(line, sep);
  boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

  int team_size = atoi((*beg).c_str());
  // My benchmark
  for (int i = 0; i < team_size; i++) {
    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
      getline(myfile, line);
    }
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    int pickup = atoi((*beg).c_str());
    beg++;
    if (beg == tok.end()) {
      std::cerr << "Input file wrong. No delivery";
      exit(-1);
    }
    int delivery = atoi((*beg).c_str());
    Task task;
    task.task_id = i;
    task.pickup_loc = pickup;
    task.delivery_loc = delivery;
    res.push_back(task);
  }
  myfile.close();

  return res;
}

template <typename T>
T read_param_json(nlohmann::json& data, std::string name) {
  if (!data.contains(name)) {
    std::cerr << "missing property " << name << " in the input JSON."
              << std::endl;
    exit(1);
  }
  try {
    return data[name].get<T>();
  } catch (nlohmann::json::type_error error) {
    std::cerr << "Incorrect input JSON format for " << name << std::endl;
    std::cerr << "Message: " << error.what() << std::endl;
    exit(1);
  }
}

template <typename T>
T read_param_json(nlohmann::json& data, std::string name, T default_value) {
  if (!data.contains(name)) {
    return default_value;
  }
  try {
    return data[name].get<T>();
  } catch (nlohmann::json::type_error error) {
    std::cerr << "Incorrect input JSON format for " << name << std::endl;
    std::cerr << "Message: " << error.what() << std::endl;
    exit(1);
  }
}

struct PairHash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    auto hash1 = std::hash<T1>{}(pair.first);
    auto hash2 = std::hash<T2>{}(pair.second);
    return hash1 ^ hash2;  // Combine the two hashes
  }
};

inline int RouletteWheel(const std::vector<double>& weights,
                         unsigned int* seed) {
  double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
  double r = 1.0 * rand_r(seed) / RAND_MAX;
  double threshold = weights[0];
  int id = 0;
  while (threshold < r * sum) {
    id++;
    threshold += weights[id];
  }
  return id;
}

inline std::string DoubleToStringWithFiveDecimalPlaces(double value) {
  std::ostringstream streamObj;
  // 设置保留五位小数
  streamObj << std::fixed << std::setprecision(5) << value;
  return streamObj.str();
}

#ifdef DEV
#define ONLYDEV(code) code
#else
#define ONLYDEV(code)
#endif