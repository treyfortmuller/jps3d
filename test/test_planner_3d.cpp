#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <chrono>

using namespace JPS;

int main(int argc, char **argv)
{
  // Catch argument errors
  if (argc != 2)
  {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read the map from yaml
  auto mapTimeStart = std::chrono::steady_clock::now();
  MapReader<Vec3i, Vec3f> reader(argv[1], true); // Map read from a given file
  if (!reader.exist())
  {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  auto mapTimeEnd = std::chrono::steady_clock::now();
  auto mapTime = std::chrono::duration_cast<std::chrono::seconds>(mapTimeEnd - mapTimeStart).count();
  std::cout << "Reading map from YAML time in seconds : " << mapTime << " s" << std::endl;

  // store map in map_util, create a new planner, set the start and end goals
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  const Vec3f start(reader.start(0), reader.start(1), reader.start(2));
  const Vec3f goal(reader.goal(0), reader.goal(1), reader.goal(2));

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner

  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();

  // Run JPS!
  Timer time_jps(false);
  auto startTime = std::chrono::steady_clock::now();
  // Plan from start to goal using JPS
  bool valid_jps = planner_ptr->plan(start, goal, 1, true);
  auto endTime = std::chrono::steady_clock::now();
  auto planTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

  // This guy's JPS timer
  // double dt_jps = time_jps.Elapsed().count();
  // printf("JPS Planner takes: %f ms\n", dt_jps);

  // Output JPS planning stats
  std::cout << "JPS plan time in ms : " << planTime << " ms" << std::endl;
  printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
  printf("JPS Path: \n");
  auto path_jps = planner_ptr->getRawPath();
  for (const auto &it : path_jps)
    std::cout << it.transpose() << std::endl;

  // Run a timed A* plan on the same grid with the same start and end point and output stats
  // Timer time_astar(true);
  // bool valid_astar = planner_ptr->plan(start, goal, 1, false); // Plan from start to goal using A*
  // double dt_astar = time_astar.Elapsed().count();
  // printf("AStar Planner takes: %f ms\n", dt_astar);
  // printf("AStar Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
  // printf("AStar Path: \n");
  // auto path_astar = planner_ptr->getRawPath();
  // for (const auto &it : path_astar)
  //   std::cout << it.transpose() << std::endl;

  return 0;
}
