#include "behavior_states.hpp"
#include "../helpers.hpp"
#include "lane_costs.hpp"


State::~State(){}

Path State::decide_path(StateMachine& machine,
                        CarPos& car_pos,
                        Path& path_remaining,
                        vector<OtherCar>& other_cars,
                        Map& map)
{
    State* next_state = decide_state(car_pos, path_remaining, other_cars, map);
    set_state(machine, next_state);
    Path path = next_state->get_path(car_pos, path_remaining, other_cars, map);
    return path;
}

State* State::decide_state(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{}

Path State::get_path(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{}

void State::set_state(StateMachine &machine, State *state)
{
    if (state == machine.mState) {
        return;
    }
    else {
        State* aux = machine.mState;
        machine.mState = state;
        delete aux;
    }
}


//===================================


Init::Init(int id_lane) {
    m_id_lane = id_lane;
    m_cnt_runs = 0;
}
Init::~Init() {}

State* Init::decide_state(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{
    if (m_cnt_runs == 0) {
        m_cnt_runs++;
        return this;
    }
    else {
        State* next_state = new KeepLane(m_id_lane);
        return next_state;
    }
}

Path Init::get_path(CarPos& car_pos,
                    Path& path_remaining,
                    vector<OtherCar>& other_cars,
                    Map& map)
{
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

KeepLane::KeepLane(int id_lane){
    m_id_lane = id_lane;
}
KeepLane::~KeepLane() {}
State* KeepLane::decide_state(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{
    if (path_remaining.x.size()<4) {
        State* next_state = new Init(m_id_lane);
        return next_state;
    }
    else {
        return this;
    }
}

Path KeepLane::get_path(CarPos& car_pos,
                        Path& path_remaining,
                        vector<OtherCar>& other_cars,
                        Map& map)
{
    if (path_remaining.x.size() >= 8) {
        // if remaining path still has multiple points, simply return it
        return path_remaining;
    }
    else {
        // get target speed for lane
        TargetSpeed end = get_target_speed_for_lane(other_cars, car_pos, m_id_lane);

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
        struct::Path path = smoothen_path(anchor_points,
                                          path_remaining,
                                          car_pos,
                                          end.t,
                                          end.v_in_meter_per_s,
                                          end.dv
                                          );
        return path;
    }
}


float KeepLane::get_cost(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{
    return 0;
}

