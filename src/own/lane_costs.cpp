#include "lane_costs.hpp"
#include "math.h"

TargetSpeed get_target_speed_for_lane(vector<OtherCar>& other_cars,
                                      CarPos& car_pos,
                                      int id_lane
                                      )
{
    // find nearest car in lane
    bool in_front = true;
    bool is_nearest_car_found;
    OtherCar nearest_car = get_nearest_car_in_lane(other_cars,
                                                   car_pos,
                                                   id_lane,
                                                   in_front,
                                                   &is_nearest_car_found
                                                   );

    // choose target speed
    TargetSpeed target;

    if (!is_nearest_car_found) {
        // if no car is in lane, end speed shall be constant 47.5 mph, reached in 1s
        target.v_in_meter_per_s = 47.5 * 0.44704;
        target.dv = 0;
        target.t = 1;
    }
    else {
        // Otherwise, adjust speed to speed of nearest car in front
        // Also take into account distance, so that it still accelerates if farer away than target distance
        double s_diff = nearest_car.s - car_pos.s;
        double target_distance = 15;
        double factor_distance = 1.0 * s_diff / target_distance;
        double nearest_car_v_in_meter_per_s = sqrt(pow(nearest_car.vx,2) + pow(nearest_car.vy,2) );
        target.v_in_meter_per_s = factor_distance * nearest_car_v_in_meter_per_s;
        target.v_in_meter_per_s = min(target.v_in_meter_per_s, 47.5*0.44704);
        target.dv = 0;
        target.t = 1;
    }
    return target;
}

double get_distance_to_next_car(int id_lane,
                                CarPos& car_pos,
                                vector<OtherCar> other_cars
                                )
{
    bool is_in_front = true;
    bool is_nearest_car_found;
    OtherCar nearest_car = get_nearest_car_in_lane(other_cars,
                                                   car_pos,
                                                   id_lane,
                                                   is_in_front,
                                                   &is_nearest_car_found
                                                   );
    double s_diff;
    if (is_nearest_car_found) {
        s_diff = nearest_car.s - car_pos.s;
    }
    else {
        s_diff = 100; // as a maximum
    }
    return s_diff ;
}

bool is_lane_safe(int id_lane,
                  CarPos& car_pos,
                  vector<OtherCar> other_cars
                  )
{

    // for both in front of us and behind us, check distance to nearest car and
    for (int is_in_front = 0; is_in_front <=1; is_in_front++) {
        bool is_nearest_car_found;
        OtherCar nearest_car = get_nearest_car_in_lane(other_cars,
                                                       car_pos,
                                                       id_lane,
                                                       is_in_front,
                                                       &is_nearest_car_found
                                                       );

        if (is_nearest_car_found) {
            // calculate t_until_crash
            double nearest_car_v_in_meter_per_s = sqrt(pow(nearest_car.vx,2) + pow(nearest_car.vy,2) );
            double v_diff = nearest_car_v_in_meter_per_s - car_pos.speed_in_meter_per_s;
            double s_diff = nearest_car.s - car_pos.s;
            double t_until_crash = - s_diff / v_diff;
            bool is_crash_soon = (0< t_until_crash && t_until_crash < 1.5);

            // decide whether lane is safe
            if (abs(s_diff) < 10 || is_crash_soon) {
                return false;
            }
        }
    }

    return true;
}

OtherCar get_nearest_car_in_lane(vector<OtherCar>& other_cars,
                                 CarPos& car_pos,
                                 int id_lane,
                                 bool is_in_front,
                                 bool* is_nearest_car_found
                                 )
{
    // preprocess
    *is_nearest_car_found = false;
    int factor_reverse = is_in_front ? +1: -1;
    OtherCar nearest_car;

    // find nearest car in lane
    double d = lane2d(id_lane);
    double d_tol = 3;
    double s_last = 100; // threshold, ignore anything which is farer away
    for (auto car : other_cars) {
        if (d-d_tol <= car.d && car.d <= d+d_tol) { // goal lane
            if (factor_reverse * (car.s - car_pos.s) > 0) { // if car is in front / behind
                double s_diff = abs(car.s - car_pos.s);
                if (s_diff < s_last) { // if car is nearer than any previous car
                    s_last = s_diff;
                    *is_nearest_car_found = true;
                    nearest_car =car;
                }
            }
        }
    }

    return nearest_car;
}
