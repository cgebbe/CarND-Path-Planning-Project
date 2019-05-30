#ifndef MAP_H
#define MAP_H

#include <vector>

using namespace std;

struct CarPos {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed ;
};

struct Map {
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};

struct::Map get_map();

#endif MAP_H
