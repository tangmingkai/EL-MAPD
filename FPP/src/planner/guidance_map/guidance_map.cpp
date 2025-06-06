#include "planner/guidance_map/guidance_map.h"

#include "boost/filesystem.hpp"

namespace planner {

GuidanceMap::GuidanceMap(const Environment &env)
    : env_(env), n_orientation_(5) {}

std::string GuidanceMap::Load(const string &weights_path) {
  // : make weights float
  // we have at least 5 weights for a location: right,down,left,up,stay
  std::cout << "map_weights_path:" << weights_path << std::endl;
  map_weights_ = std::vector<double>(env_.rows * env_.cols * 5, 1);
  std::string suffix = "all_one";
  if (weights_path != "") {
    std::ifstream f(weights_path);
    try {
      nlohmann::json _weights = nlohmann::json::parse(f);
      if (_weights.size() != map_weights_.size()) {
        std::cerr << "map weights size mismatch" << std::endl;
        exit(-1);
      }
      for (int i = 0; i < map_weights_.size(); ++i) {
        map_weights_[i] = _weights[i].get<double>();
        if (map_weights_[i] < 1) {
          std::cerr << "map weights is less than 1" << map_weights_[i]
                    << std::endl;
          exit(-1);
        }
      }
    } catch (nlohmann::json::parse_error error) {
      std::cerr << "Failed to load " << weights_path << std::endl;
      std::cerr << "Message: " << error.what() << std::endl;
      exit(1);
    }
    boost::filesystem::path _weights_path(weights_path);
    suffix = _weights_path.stem().string();
  } else {
    std::cout << "all one guidance" << std::endl;
  }
  return suffix;
}

double GuidanceMap::GetWeight(const int &location, const int &orientation,
                           const Action &action) const {
  int weight_idx;
  if (action != Action::FW) {
    weight_idx = location * n_orientation_ + 4;
  } else {
    // int next_pos;
    if (orientation == 0) {
      // east
      // next_pos = location + 1;
      weight_idx = location * n_orientation_;
    } else if (orientation == 1) {
      // south
      // next_pos = location + env_.cols;
      weight_idx = location * n_orientation_ + 1;
    } else if (orientation == 2) {
      // west
      // next_pos = location - 1;
      weight_idx = location * n_orientation_ + 2;
    } else if (orientation == 3) {
      // north
      // next_pos = location - env_.cols;
      weight_idx = location * n_orientation_ + 3;
    }
  }
  return map_weights_[weight_idx];
}
}  // namespace planner
