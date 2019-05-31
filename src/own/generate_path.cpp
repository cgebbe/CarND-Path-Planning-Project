#include <vector>
#include "generate_path.hpp"
#include "../helpers.hpp"
#include "../../thirdparty/spline.h"

struct::Path get_next_path(struct::CarPos& car_pos,
                           struct::Path path_remaining,
                           struct::Map& map)
{

    // Problem: at the beginning, there is no remaining path. -> Fake one by simulating point behind!
    if (path_remaining.x.size() == 0 ) {
        double v_barely = 0.01;
        for (double dt=-0.040; dt<=0.001; dt+=0.020) {
            double x_behind = car_pos.x + v_barely * dt * cos(car_pos.yaw);
            double y_behind = car_pos.y + v_barely * dt * sin(car_pos.yaw);
            path_remaining.x.push_back(x_behind);
            path_remaining.y.push_back(y_behind);
        }
    }

    if (path_remaining.x.size() >= 5) {
        // if remaining path still has multiple points, do not create new path
        return path_remaining;
    }
    else {
        // append a few anchor points spaced 30m, 60m, 90m apart
        struct::Path anchor_points;
        double vel_in_mph = 47.5;
        double vel_in_m_per_s = vel_in_mph * 0.44704;
        double dist_per_20ms_in_m = vel_in_m_per_s * 0.020;
        int num_points = 4;
        for (int i = 0; i < num_points; ++i) {
            double next_s = car_pos.s + (i+1)*30;
            double next_d = 6; // center = 0, d_lane = 4m, d=6m -> center of middle lane
            std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);

            anchor_points.x.push_back(xy[0]);
            anchor_points.y.push_back(xy[1]);
        }

        // based on anchor points and remaining path, estimate actual path
        struct::Path path = smoothen_path(anchor_points, path_remaining, car_pos, vel_in_m_per_s);
        return path;
    }
}

struct::Path smoothen_path(struct::Path anchor_points,
                           struct::Path path_remaining,
                           struct::CarPos car_pos,
                           double v_goal)
{
    CarState last_state = get_last_car_state(path_remaining);
    CarPos last_pos;
    last_pos.x = last_state.x;
    last_pos.y = last_state.y;
    last_pos.yaw = last_state.yaw;

    // fuse anchor points and remaining path points into a common path list
    Path path_in_MCS = path_remaining;
    for(int i=0; i<anchor_points.x.size(); i++) {
        path_in_MCS.x.push_back(anchor_points.x[i]);
        path_in_MCS.y.push_back(anchor_points.y[i]);
    }

    // Convert all points from map coordinate system (CS) into vehicle CS
    Path path_in_VCS = convert_MCS_to_VCS(path_in_MCS, last_pos);

    // fit y(x) in vehicle CS
    tk::spline y_per_x_in_VCS;
    y_per_x_in_VCS.set_points(path_in_VCS.x, path_in_VCS.y);

    // sample points such that velocity = target velocity
    struct::Path path_sampled_VCS = convert_MCS_to_VCS(path_remaining, last_pos);
    for (double dt = 0.020; dt<2; dt+=0.020) {
        double x_VCS = dt * v_goal;
        path_sampled_VCS.x.push_back(x_VCS);
        path_sampled_VCS.y.push_back(y_per_x_in_VCS(x_VCS));
    }

    // convert sampled points from vehicle CS to map CS
    Path path_sampled_in_MCS = convert_VCS_to_MCS(path_sampled_VCS, last_pos);

    return path_sampled_in_MCS;
}


struct::Path convert_MCS_to_VCS(struct::Path path, CarPos car_pos)
{
    for (int i=0; i<path.x.size(); i++) {
        double x = path.x[i]-car_pos.x;
        double y = path.y[i]-car_pos.y;
        path.x[i] = x * cos(-car_pos.yaw) - y * sin(-car_pos.yaw);
        path.y[i] = x * sin(-car_pos.yaw) + y * cos(-car_pos.yaw);
    }
    return path;
}


struct::Path convert_VCS_to_MCS(struct::Path path, CarPos car_pos)
{
    for (int i=0; i<path.x.size(); i++) {
        double x = path.x[i] * cos(car_pos.yaw) - path.y[i] * sin(car_pos.yaw);
        double y = path.x[i] * sin(car_pos.yaw) + path.y[i] * cos(car_pos.yaw);
        path.x[i] = x + car_pos.x;
        path.y[i] = y + car_pos.y;
    }
    return path;
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
