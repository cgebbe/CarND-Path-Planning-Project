#include "lane_costs.hpp"
#include "math.h"

TargetSpeed get_target_speed_for_lane(vector<OtherCar>& other_cars,
                                      CarPos& car_pos,
                                      int& lane_id
                                      )
{
    // find nearest car in lane
    bool is_car_in_lane = false;
    OtherCar nearest_car;
    double d = lane2d(lane_id);
    double d_tol = 3;
    double s_last = 100; // threshold, ignore anything which is farer away
    for (auto car : other_cars) {
        if (d-d_tol <= car.d && car.d <= d+d_tol) { // same lane
            if (car.s > car_pos.s) { // if car is in front
                double s_diff = car.s - car_pos.s;
                if (s_diff < s_last) { // if car is nearer than any previous car
                    s_last = s_diff;
                    is_car_in_lane = true;
                    nearest_car =car;
                }
            }
        }
    }

    // choose target speed
    TargetSpeed target;

    if (!is_car_in_lane) {
        // if no car is in lane, end speed shall be constant 47.5 mph, reached in 1s
        target.v_in_meter_per_s = 47.5 * 0.44704;
        target.dv = 0;
        target.t = 1;
    }
    else {
        double nearest_car_v_in_meter_per_s = sqrt(pow(nearest_car.vx,2) + pow(nearest_car.vy,2) );
        target.v_in_meter_per_s = min(nearest_car_v_in_meter_per_s, 47.5*0.44704);
        target.dv = 0;
        target.t = 1;
    }
    return target;

    /*
        double dummy = sqrt(pow(4,2) + pow(3,2));
        double nearest_car_v_in_meter_per_s = sqrt(pow(nearest_car.vx,2) + pow(nearest_car.vy,2) );
        double s_diff_in_m = nearest_car.s - car_pos.s;
        double v_diff_in_m = nearest_car_v_in_meter_per_s - car_pos.speed_in_meter_per_s;
        double t_until_crash = - s_diff_in_m / v_diff_in_m;

        bool is_crash_soon = (0 <t_until_crash && t_until_crash < 2);
        if (is_crash_soon || v_diff_in_m > 25*0.44704 || s_diff_in_m < 10) {
            // maximum break
            target.v_in_meter_per_s = (car_pos.speed_in_mph - 5) * 0.44704;
            target.dv = -2*0.44704;
            target.t = 0.5;
        }
        else {
            // slow down to vehicle speed, but not too quickly
            if (v_diff_in_m > 20 * 0.44704) {
                target.v_in_meter_per_s = (car_pos.speed_in_mph - 20) * 0.44704;
            }
            else {
                target.v_in_meter_per_s = min(nearest_car_v_in_meter_per_s, 47.5*0.44704);
            }
        }
    }

    return target; */
}
