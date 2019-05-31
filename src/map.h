#ifndef MAP_H
#define MAP_H

#include <vector>
#include "../thirdparty/spline.h"

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
    vector<double> wp_x;
    vector<double> wp_y;
    vector<double> wp_s;
    vector<double> wp_dx;
    vector<double> wp_dy;
    tk::spline spline_x;
    tk::spline spline_y;
};

struct::Map get_map();

#endif MAP_H
