#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main()
{
  // Set start & goal
  std::vector<double> start{0.5, 0.5, 0.5};
  std::vector<double> goal{380.5, 380.5, 80.5};
  // Create a map
  std::vector<double> origin{0, 0, 0};      // set origin at (0, 0, 0)
  std::vector<int> dim{400, 400, 100};      // set the number of cells in each dimension as 20, 10, 1
  double res = 1.0;                         // set resolution as 1m
  std::vector<int> data;                    // occupancy data, the subscript follows: id = x + dim.x * y + dim.x * dim.y * z;
  data.resize(dim[0] * dim[1] * dim[2], 0); // initialize as free map, free cell has 0 occupancy

  YAML::Emitter out;
  out << YAML::BeginSeq;
  // Encode start coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "start" << YAML::Value << YAML::Flow << start;
  out << YAML::EndMap;
  // Encode goal coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "goal" << YAML::Value << YAML::Flow << goal;
  out << YAML::EndMap;
  // Encode origin coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "origin" << YAML::Value << YAML::Flow << origin;
  out << YAML::EndMap;
  // Encode dimension as number of cells
  out << YAML::BeginMap;
  out << YAML::Key << "dim" << YAML::Value << YAML::Flow << dim;
  out << YAML::EndMap;
  // Encode resolution
  out << YAML::BeginMap;
  out << YAML::Key << "resolution" << YAML::Value << res;
  out << YAML::EndMap;
  // Encode occupancy
  out << YAML::BeginMap;
  out << YAML::Key << "data" << YAML::Value << YAML::Flow << data;
  out << YAML::EndMap;

  out << YAML::EndSeq;
  std::cout << "Here is the example map:\n"
            << out.c_str() << std::endl;

  std::ofstream file;
  file.open("trey.yaml");
  file << out.c_str();
  file.close();

  return 0;
}
