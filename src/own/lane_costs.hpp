#ifndef OTHER_CARS_H
#define OTHER_CARS_H

#include "../map.h"
#include <vector>

struct TargetSpeed {
    double v_in_meter_per_s; // in meter per second
    double dv; // in meter per s^2
    double t; // in s
};

TargetSpeed get_target_speed_for_lane(vector<OtherCar>& other_cars,
                                      CarPos& car_pos,
                                      int& lane_id
                                      );

inline double lane2d(int lane_id) {
    double d = 2 + lane_id * 4;
    return d;
}

#endif // OTHER_CARS_H
