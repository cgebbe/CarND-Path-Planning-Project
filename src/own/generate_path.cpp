#include <vector>
#include "generate_path.hpp"
#include "../helpers.hpp"
#include "../../thirdparty/spline.h"

struct::Path get_next_path(struct::CarPos& car_pos,
                           struct::Path path_remaining,
                           struct::Map& map)
{

    if (path_remaining.x.size() == 0 ) {
        struct::Path anchor_points;
        double vel_in_mph = 47.5;
        double vel_in_m_per_s = vel_in_mph * 0.44704;
        double dist_per_20ms_in_m = vel_in_m_per_s * 0.020;
        int num_points = 20;
        for (int i = 0; i < num_points; ++i) {
            double next_s = car_pos.s + (i+1)*dist_per_20ms_in_m;
            double next_d = 6; // center = 0, d_lane = 4m, d=6m -> center of middle lane
            std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);

            anchor_points.x.push_back(xy[0]);
            anchor_points.y.push_back(xy[1]);
        }
        return anchor_points;
    }
    else if (path_remaining.x.size() > 0) {
        if (path_remaining.x.size() < 10) {
            // define anchor points spaced wide apart
            struct::Path anchor_points;
            double vel_in_mph = 47.5;
            double vel_in_m_per_s = vel_in_mph * 0.44704;
            double dist_per_20ms_in_m = vel_in_m_per_s * 0.020;
            int num_points = 40;
            for (int i = 0; i < num_points; ++i) {
                double next_s = car_pos.s + (i+1)*dist_per_20ms_in_m*10;
                double next_d = 6; // center = 0, d_lane = 4m, d=6m -> center of middle lane
                std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);

                anchor_points.x.push_back(xy[0]);
                anchor_points.y.push_back(xy[1]);
            }

            // based on anchor points and remaining path, estimate actual path
            struct::Path path = smoothen_path(anchor_points, path_remaining, vel_in_m_per_s);
            return path;
        }
        else {
            // simply return existing remaining path
            return path_remaining;
        }
    }
}

struct::Path smoothen_path(struct::Path anchor_points,
                           struct::Path path_remaining,
                           double v_goal)
{
    // get car state at last point of remaining path
    CarState last = get_last_car_state(path_remaining);
    double x = last.x;
    double y = last.y;
    double v = last.v;
    double dv = last.dv;
    double ddv = last.ddv;
    double yaw = last.yaw;
    double dyaw = last.dyaw;

    // control params
    double dyaw_gain = 0.1;
    double dyaw_max = 2./360.*2.*pi();
    double ddv_gain = 0.1;
    double ddv_max = 10.;
    double dv_max = 5.;
    double dt = 0.020; // in s

    int num_points = 100;
    int idx_anchor = 0;
    for (int i=0; i< num_points; i++) {
        // set yaw rate
        idx_anchor = get_next_anchor(anchor_points, idx_anchor, x, y);
        double yaw_goal = get_yaw(x, y, anchor_points.x[idx_anchor], anchor_points.y[idx_anchor]);
        double yaw_diff = yaw_goal - yaw;
        //double dyaw_control = clip(yaw_diff * dyaw_gain, -dyaw_max, +dyaw_max);
        double dyaw_control = sign(yaw_diff) * dyaw_gain;
        dyaw += dyaw_control;
        yaw += dyaw;

        // set velocity
        double v_diff = v_goal - v;
        //double ddv_control = clip(v_diff * ddv_gain, -ddv_max, +ddv_max);
        double ddv_control = sign(v_diff) * ddv_gain;
        ddv = clip(ddv + ddv_control, -ddv_max, +ddv_max);
        dv = clip(dv + ddv, -dv_max, +dv_max);
        v += dv;

        // calculate displacement
        x += dt * v * cos(yaw);
        y += dt * v * sin(yaw);
        path_remaining.x.push_back(x);
        path_remaining.y.push_back(y);
    }

    return path_remaining;
}

int get_next_anchor(struct::Path anchor_points, int idx_anchor_last, double x, double y)
{
    double dist_min_in_m = 20;
    assert(0 <= idx_anchor_last && idx_anchor_last < anchor_points.x.size());
    for (int idx_anchor = idx_anchor_last; idx_anchor < anchor_points.x.size(); idx_anchor++) {
        double dist = calc_distance(x, y,
                                    anchor_points.x[idx_anchor], anchor_points.y[idx_anchor]);
        if (dist >= dist_min_in_m) {
            return idx_anchor;
        }
    }
    return 0;
}

struct::CarState get_last_car_state(struct::Path path)
{
    // calculate distances
    vector<double> distances;
    for (int i=1; i<path.x.size(); i++) {
        double dist = calc_distance(path.x[i], path.y[i],
                                    path.x[i-1], path.y[i-1]);
        distances.push_back(dist);
    }

    // calculate cumulative distance
    vector<double> distance_cumulative = distances;
    for (int i=1; i<distance_cumulative.size(); i++) {
        distance_cumulative[i] = distance_cumulative[i-1] + distances[i];
    }

    // get current rate and velocities from last three points of previous path
    double dt = 0.020;
    vector<double> v = calc_time_derivative(distance_cumulative, dt);
    vector<double> dv = calc_time_derivative(v, dt);
    vector<double> ddv = calc_time_derivative(dv, dt);

    // get yaw
    vector<double> dx = calc_time_derivative(path.x, dt);
    vector<double> dy = calc_time_derivative(path.y, dt);
    vector<double> yaw;
    for (int i=0; i<dx.size(); i++) {
        double angle = atan2(dy[i], dx[i]);
        yaw.push_back(angle);
    }
    vector<double> dyaw = calc_time_derivative(yaw, dt);

    // init car state
    CarState last;
    last.x = get_last_elem(path.x);
    last.y = get_last_elem(path.y);
    last.yaw = get_last_elem(yaw);
    last.dyaw = get_last_elem(dyaw);
    last.v = get_last_elem(v);
    last.dv = get_last_elem(dv);
    last.ddv = get_last_elem(ddv);

    return last;
}

void check_path(struct::Path path)
{
    // calculate distance between each waypoint
    vector<double> distances;
    for (int i=0; i<path.x.size()-1; i++) {
        double dist = calc_distance(path.x[i], path.y[i],
                                    path.x[i+1], path.y[i+1]);
        distances.push_back(dist);
    }

    // calculate cumulative distance
    vector<double> distance_cumulative = distances;
    for (int i=1; i<distance_cumulative.size(); i++) {
        distance_cumulative[i] = distance_cumulative[i-1] + distances[i];
    }


    // calculate velocity and acceleration
    double dt = 0.020;
    vector<double> velocities = calc_time_derivative(distance_cumulative, dt);
    vector<double> accelerations = calc_time_derivative(velocities, dt);

    return;
}

vector<double> calc_time_derivative(vector<double> x, double dt)
{
    // returns vector as derivative with size x.size()-1
    vector<double> derivative;
    for (int i=1; i<x.size(); i++) {
        double x_per_t = (x[i] - x[i-1]) / dt;
        derivative.push_back(x_per_t);
    }
    return derivative;
}


void test_spline_xy()
{
    // create spiral
    vector<double> x_guess;
    vector<double> x_fit;
    vector<double> y_fit;
    for (int i=0; i<36; i++){
        double phi = i*10 / 360*2*pi();
        double radius = 1 / (i/36+1);
        if (i % 2) {
            x_fit.push_back( radius * cos(phi));
            y_fit.push_back( radius * sin(phi));
        }
        x_guess.push_back(radius*cos(phi));
    }

    // create spline
    tk::spline y_per_x;
    y_per_x.set_points(x_fit, y_fit);

    // eval
    vector<double> y_guess;
    for (auto x : x_guess){
        y_guess.push_back(y_per_x(x));
    }

    return;
}
