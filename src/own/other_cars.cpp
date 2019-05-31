#include "other_cars.hpp"

vector<OtherCar> get_cars_in_lane(vector<OtherCar> other_cars,
                          int lane_id
                          )
{
    vector<OtherCar> cars_same_lane;
    double d = lane2d(lane_id);
    double d_tol = 2;
    for (auto car : other_cars) {
        if (d-d_tol <= car.d && car.d <= d+d_tol) {
            cars_same_lane.push_back(car);
        }
    }
    return cars_same_lane;
}


