#ifndef OTHER_CARS_H
#define OTHER_CARS_H

#include "../map.h"
#include <vector>

vector<OtherCar> get_cars_in_lane(vector<OtherCar> other_cars,
                          int id_lane
                          );

inline double lane2d(int lane_id) {
    double d = 2 + lane_id * 4;
    return d;
}

#endif // OTHER_CARS_H
