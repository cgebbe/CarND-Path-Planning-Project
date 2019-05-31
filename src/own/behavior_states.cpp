#include "behavior_states.hpp"
#include "../helpers.hpp"
#include "other_cars.hpp"

State::~State() {}
Path State::get_next_path(StateMachine& machine,
                         CarPos& car_pos,
                         Path& path_remaining,
                         vector<OtherCar>& other_cars,
                         Map& map)
{}
void State::set_state(StateMachine &machine, State *state)
{
    State* aux = machine.mState;
    machine.mState = state;
    delete aux;
}


//===================================

Init::~Init() {}
Path Init::get_next_path(StateMachine& machine,
                         CarPos& car_pos,
                         Path& path_remaining,
                         vector<OtherCar>& other_cars,
                         Map& map)
{
    // State Init should only be run for one loop, after that State KeepLane
    set_state(machine, new KeepLane());

    /* Empty remaining path should only occur at the beginning or when debugging.
     * Problem: Without remaining path, no last velocity can be estimated
     * Solution -> fake remaining path with very slow velocity
     */
    path_remaining.x.clear();
    path_remaining.y.clear();
    double v_slow = 0.01;
    for (double dt=-0.060; dt<=0.001; dt+=0.020) {
        double x_behind = car_pos.x + v_slow * dt * cos(car_pos.yaw);
        double y_behind = car_pos.y + v_slow * dt * sin(car_pos.yaw);
        path_remaining.x.push_back(x_behind);
        path_remaining.y.push_back(y_behind);
    }

    // Set only one anchor point 50m far away in center lane
    Path anchor_points;
    double next_s = car_pos.s + 50;
    double next_d = car_pos.d; // center = 0, d_lane = 4m, d=6m -> center of middle lane
    std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);
    anchor_points.x.push_back(xy[0]);
    anchor_points.y.push_back(xy[1]);

    // based on anchor points and remaining path, estimate actual path
    double t_end = 4;
    double v_end = 44 * 0.44704;
    double dv_end = 3 * 0.44704;
    Path path = smoothen_path(anchor_points,
                              path_remaining,
                              car_pos,
                              t_end,
                              v_end,
                              dv_end
                              );
    return path;
}


//===================================

KeepLane::KeepLane() {
    lane_id = 1;
}
KeepLane::~KeepLane() {}
Path KeepLane::get_next_path(StateMachine& machine,
                             CarPos& car_pos,
                             Path& path_remaining,
                             vector<OtherCar>& other_cars,
                             Map& map)
{
    if (path_remaining.x.size() >= 8) {
        // if remaining path still has multiple points, simply return it
        return path_remaining;
    }
    else if (path_remaining.x.size() < 4) {
        /* When the remaining path has less than 4 members, dv cannot be calculated.
         * Without dv, the future velocity cannot be connected smoothly with the remaining path.
         * Thus, start from scratch. Happens in debug node and is here for convenience.
         */
        set_state(machine, new Init());
        return machine.get_next_path(car_pos,
                                     path_remaining,
                                     other_cars,
                                     map);
    }
    else {
        /* "Normal" case
         */

        /* Usually, use 47.5 mph as target end speed
         * However, if a car is in front of us, use its current speed as target speed
         */
        double v_end = 47.5 * 0.44704;
        vector<OtherCar> cars_same_lane = get_cars_in_lane(other_cars, lane_id);
        double s_last = 50; // threshold, forget anything more far away
        for (auto car : cars_same_lane) {
            if (car.s > car_pos.s) { // if car is in front
                if (car.s - car_pos.s < s_last) { // if car is nearer than any previous car
                    v_end = sqrt(car.vx*car.vx + car.vy*car.vy);
                }
            }
        }

        // define anchor points to keep lane
        struct::Path anchor_points;
        int num_points = 4;
        for (int i = 0; i < num_points; ++i) {
            double next_s = car_pos.s + (i+1)*30;
            double next_d = 6; // center = 0, d_lane = 4m, d=6m -> center of middle lane
            std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);
            anchor_points.x.push_back(xy[0]);
            anchor_points.y.push_back(xy[1]);
        }

        // based on anchor points and remaining path, estimate actual path
        double t_end = 1.0;
        double dv_end = 0;
        struct::Path path = smoothen_path(anchor_points,
                                          path_remaining,
                                          car_pos,
                                          t_end,
                                          v_end,
                                          dv_end
                                          );
        return path;
    }
}
