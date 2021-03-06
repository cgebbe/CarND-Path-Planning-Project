#include <math.h>
#include <string>
#include <vector>
#include "map.h"
#include "helpers.hpp"
#include "../thirdparty/spline.h"

// for convenience
using std::string;
using std::vector;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Calculate closest waypoint to current x, y position
int get_closest_waypoint(double x, double y, const vector<double> &maps_x,
                         const vector<double> &maps_y) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); ++i) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = calc_distance(x,y,map_x,map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int get_next_waypoint(double x, double y, double theta, const vector<double> &maps_x,
                      const vector<double> &maps_y) {
    int closestWaypoint = get_closest_waypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = std::min(2*pi() - angle, angle);

    if (angle > pi()/2) {
        ++closestWaypoint;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> convert_xy_to_sd(double x, double y, double theta, struct::Map& map) {
    vector<double> maps_x = map.wp_x;
    vector<double> maps_y = map.wp_y;

    int next_wp = get_next_waypoint(x,y, theta, maps_x, maps_y);
    int prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    double frenet_d = calc_distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = calc_distance(center_x,center_y,x_x,x_y);
    double centerToRef = calc_distance(center_x,center_y,proj_x,proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += calc_distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += calc_distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> convert_sd_to_xy(double s, double d, struct::Map& map) {
    bool use_new_version = false;
    if (use_new_version ) {
        // prevent strange errors at end of track
        if (s > 64959.9) {
            s -= 6945.9;
        }

        // get position
        double x_center = map.spline_x(s);
        double y_center = map.spline_y(s);

        // calculate gradient of spline at x,y position and heading
        double diff = 1;
        double dy = (map.spline_y(s+diff) - map.spline_y(s-diff));
        double dx = (map.spline_x(s+diff) - map.spline_x(s-diff));
        double heading = atan2(dy, dx);

        // get norm d-vector by rotating dy,dx
        //d=0;
        double perp_heading = heading-pi()/2;
        double x = x_center + d*cos(perp_heading);
        double y = y_center + d*sin(perp_heading);

        return {x,y};
    }
    else {
        vector<double> maps_s = map.wp_s;
        vector<double> maps_x = map.wp_x;
        vector<double> maps_y = map.wp_y;

        // find previous waypoint
        int prev_wp = -1;
        while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
            ++prev_wp;
        }

        // get next waypoint
        int wp2 = (prev_wp+1)%maps_x.size();

        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                               (maps_x[wp2]-maps_x[prev_wp]));

        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);
        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

        double perp_heading = heading-pi()/2;

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};
    }
}
