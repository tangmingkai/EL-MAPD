#include "env/environment.h"

#include "common/nlohmann/json.hpp"

void Environment::Load(const std::string &fname,
                       const std::string &_file_storage_path) {
  std::cout << fname << std::endl;
  std::string line;
  std::ifstream myfile((fname).c_str());
  if (!myfile.is_open()) {
    cout << "Map file " << fname << " does not exist. " << std::endl;
    exit(-1);
  }
  file_storage_path = _file_storage_path;
  clock_t t = std::clock();
  size_t pos = fname.rfind('.');  // position of the file extension
  map_name = fname.substr(fname.find_last_of("/") + 1,
                          pos);  // get the name without extension
  getline(myfile, line);
  if (line[0] == 't') {
    // Benchmark
    boost::char_separator<char> sep(" ");
    getline(myfile, line);
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg;
    beg = tok.begin();
    beg++;
    rows = atoi((*beg).c_str());  // read number of rows
    getline(myfile, line);
    boost::tokenizer<boost::char_separator<char>> tok2(line, sep);
    beg = tok2.begin();
    beg++;
    cols = atoi((*beg).c_str());  // read number of cols
    getline(myfile, line);        // skip "map"

  } else {
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    rows = atoi((*beg).c_str());  // read number of rows
    beg++;
    cols = atoi((*beg).c_str());  // read number of cols
  }
  obstacles.clear();
  obstacles.resize(cols * rows, 0);
  charges.clear();
  charges.resize(cols * rows, false);
  charges_locs.clear();
  task_points.resize(cols * rows, false);
  task_locs.clear();
  loc2charge_id.resize(cols*rows , -1);
  for (int i = 0; i < rows; i++) {
    getline(myfile, line);
    for (int j = 0; j < cols; j++) {
      int id = cols * i + j;
      if (line[j] == '@' || line[j] == 'T') {
        obstacles[id] = true;
      } else {
        obstacles[id] = false;
      }
      if (line[j] == 'C') {
        charges[id] = true;
        loc2charge_id[id] = charges_locs.size();
        charges_locs.push_back(id);
      } else {
        charges[id] = false;
      }
      if (line[j] == 'E' || line[j] == 'S') {
        task_points[id] = true;
        task_locs.push_back(id);
      } else {
        task_points[id] = false;
      }
    }
  }
  myfile.close();
  double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
  // cout << "Map size: " << rows << "x" << cols;
  // cout << "\tDone! (load time: " << runtime << " s)" << std::endl;
}
