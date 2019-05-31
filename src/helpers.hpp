#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "map.h"

using namespace std;

// Inline functions
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double calc_distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Checks if the SocketIO event has JSON data.
string hasData(string s);

// Calculate closest waypoint to current x, y position
int get_closest_waypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int get_next_waypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> convert_xy_to_sd(double x, double y, double theta, struct::Map& map);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> convert_sd_to_xy(double s, double d, struct::Map& map);

#endif  // HELPERS_H
