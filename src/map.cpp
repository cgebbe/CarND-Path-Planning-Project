#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "map.h"

using namespace std;
using std::string;


struct::Map get_map()
{
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    // feed info from file
    struct::Map map;
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    string line;
    while (getline(in_map_, line)) {
      std::istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map.map_waypoints_x.push_back(x);
      map.map_waypoints_y.push_back(y);
      map.map_waypoints_s.push_back(s);
      map.map_waypoints_dx.push_back(d_x);
      map.map_waypoints_dy.push_back(d_y);
    }
    return map;
}
