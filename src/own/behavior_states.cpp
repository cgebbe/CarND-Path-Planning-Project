#include "behavior_states.hpp"
#include "../helpers.hpp"
#include "lane_costs.hpp"
#include "math.h"

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
    double t_end = 1;
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
    m_cnt_runs = 0;
}
KeepLane::~KeepLane() {}
State* KeepLane::decide_state(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{
    // if there is no remaining path (usually due to debug) -> change to init state
    if (path_remaining.x.size()<4) {
        State* next_state = new Init(m_id_lane);
        return next_state;
    }

    // if there is still a remaining path, first complete this cycle. Otherwise lanechange has old info
    if (path_remaining.x.size() >= 8) {
        // if remaining path still has multiple points, simply return it
        return this;
    }

    // At least keep lane for a few cycles (to avoid changing two lanes at same time)
    if (m_cnt_runs <= 6) {
        return this;
    }

    // "normal" behavior: return state with lowest cost
    double cost_curr = this->get_cost(car_pos, path_remaining, other_cars, map);
    double cost_keepLane = cost_curr;
    State* next_state = this;
    for (int id_lane = -1; id_lane !=3; id_lane+=2) {
        ChangeLane* state = new ChangeLane(m_id_lane + id_lane);
        double cost = state->get_cost(car_pos, path_remaining, other_cars, map);
        if (cost < cost_curr) {
            cost_curr = cost;
            next_state = state;
        }
    }
    return next_state;
}

Path KeepLane::get_path(CarPos& car_pos,
                        Path& path_remaining,
                        vector<OtherCar>& other_cars,
                        Map& map)
{

    // if there is still a remaining path, first complete this cycle. Otherwise lanechange may have old info
    if (path_remaining.x.size() >= 8) {
        // if remaining path still has multiple points, simply return it
        return path_remaining;
    }
    else {

        // state has generated a new path once...
        m_cnt_runs++;

        // define anchor points to keep lane
        struct::Path anchor_points;
        int num_points = 2;
        for (int i = 0; i < num_points; ++i) {
            double next_s = car_pos.s + (i+1)*30;
            double next_d = lane2d(m_id_lane); // center = 0, d_lane = 4m, d=6m -> center of middle lane
            std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);
            anchor_points.x.push_back(xy[0]);
            anchor_points.y.push_back(xy[1]);
        }

        // get target speed for lane and only plan 0.5s ahead
        TargetSpeed end = get_target_speed_for_lane(other_cars, car_pos, m_id_lane);
        end.t = 0.5;

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


double KeepLane::get_cost(CarPos &car_pos,
                          Path &path_remaining,
                          vector<OtherCar> &other_cars,
                          Map &map)
{
    // keeping lane is always feasible and considered safe...
    double cost_feasible = 0;
    double cost_safety = 0;

    // is lane free ? -> currently not used, but might be nice addon...
    double s_diff = get_distance_to_next_car(m_id_lane, car_pos, other_cars);
    double cost_lane_free = max(0.,1-s_diff/150.);

    // the lower the lane speed, the worse (=higher costs)
    TargetSpeed lane_speed = get_target_speed_for_lane(other_cars,car_pos,m_id_lane);
    double lane_speed_in_mph = lane_speed.v_in_meter_per_s / 0.44704;
    double cost_speed = (100-lane_speed_in_mph)/100; // in interval [0,1]

    // choose either speed or lane_free as efficiency
    double cost_efficiency = cost_lane_free;


    // center lane speed is best because of most freedo
    double cost_freedom = (m_id_lane==1) ? 0 : 1;

    // scale costs by importance and add them up
    double cost_tot = 0;
    cost_tot += scale_cost(cost_feasible,4);
    cost_tot += scale_cost(cost_safety,3);
    cost_tot += scale_cost(cost_efficiency,2);
    cost_tot += scale_cost(cost_freedom,-1);
    return cost_tot;
}



//===================================

ChangeLane::ChangeLane(int id_lane){
    m_id_lane = id_lane;
}
ChangeLane::~ChangeLane() {}

State* ChangeLane::decide_state(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{
    KeepLane* next_state = new KeepLane(m_id_lane);
    return next_state;
}

Path ChangeLane::get_path(CarPos& car_pos,
                          Path& path_remaining,
                          vector<OtherCar>& other_cars,
                          Map& map)
{
    // define anchor points to keep lane
    struct::Path anchor_points;
    int num_points = 2;
    for (int i = 0; i < num_points; ++i) {
        double next_s = car_pos.s + (i+1)*60;
        double next_d = lane2d(m_id_lane);
        std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);
        anchor_points.x.push_back(xy[0]);
        anchor_points.y.push_back(xy[1]);
    }

    // set target speed for lane as current car speed
    // TargetSpeed end = get_target_speed_for_lane(other_cars, car_pos, m_id_lane);
    TargetSpeed end;
    //end.v_in_meter_per_s = min(45 * 0.44704, car_pos.speed_in_meter_per_s);
    end.v_in_meter_per_s = car_pos.speed_in_meter_per_s;
    end.t = 1.0;
    //end.v_in_meter_per_s -= 5 * 0.44704;

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


double ChangeLane::get_cost(CarPos &car_pos, Path &path_remaining, vector<OtherCar> &other_cars, Map &map)
{
    // is lane change feasible?
    double cost_feasible;
    if (m_id_lane < 0 or m_id_lane > 2) {
        cost_feasible = 1;
    }
    else {
        cost_feasible = 0;
    }

    // is lane change safe?
    double cost_safety;
    if (is_lane_safe(m_id_lane,car_pos, other_cars)){
        cost_safety = 0;
    }
    else {
        cost_safety = 1;
    }

    // is lane free ? -> currently not used, but might be nice addon...
    double s_diff = get_distance_to_next_car(m_id_lane, car_pos, other_cars);
    double cost_lane_free = max(0.,1-s_diff/150.);

    // the lower the lane speed, the worse (=higher costs)
    TargetSpeed lane_speed = get_target_speed_for_lane(other_cars,car_pos,m_id_lane);
    double lane_speed_in_mph = lane_speed.v_in_meter_per_s / 0.44704;
    double cost_speed = (100-lane_speed_in_mph)/100; // in interval [0,1]

    // choose either speed or lane_free as efficiency
    double cost_efficiency = cost_lane_free;

    // center lane is best = lowest cost because of additional freedom
    double cost_freedom = (m_id_lane==1) ? 0 : 1;

    // scale costs by importance and add them up
    double cost_tot = 0;
    cost_tot += scale_cost(cost_feasible,4);
    cost_tot += scale_cost(cost_safety,3);
    cost_tot += scale_cost(cost_efficiency,2);
    cost_tot += scale_cost(cost_freedom,-1);
    return cost_tot;
}





