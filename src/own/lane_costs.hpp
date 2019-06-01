#ifndef OTHER_CARS_H
#define OTHER_CARS_H

#include "../map.h"
#include <vector>
#include "assert.h"
#include "math.h"

struct TargetSpeed {
    double v_in_meter_per_s; // in meter per second
    double dv; // in meter per s^2
    double t; // in s
};

TargetSpeed get_target_speed_for_lane(vector<OtherCar>& other_cars,
                                      CarPos& car_pos,
                                      int lane_id
                                      );

OtherCar get_nearest_car_in_lane(vector<OtherCar>& other_cars,
                                 CarPos& car_pos,
                                 int lane_id,
                                 bool in_front,
                                 bool* is_car_in_lane
                                 );
bool is_lane_safe(int id_lane,
                  CarPos& car_pos,
                  vector<OtherCar> other_cars
                  );

double get_distance_to_next_car(int id_lane,
                                CarPos& car_pos,
                                vector<OtherCar> other_cars
                                );


inline double lane2d(int lane_id) {
    double d = 2 + lane_id * 4;
    return d;
}

inline double scale_cost(double cost, int scale) {
    assert(0<= cost && cost <= 1); // in range [0,1]
    cost = (cost*8) + 1; // in range [1,9]
    cost = cost * pow(10,scale); // in range [10-90] or [100-900]
    return cost;
}


#endif // OTHER_CARS_H
