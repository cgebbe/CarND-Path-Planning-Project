#ifndef GENERATE_PATH_H
#define GENERATE_PATH_H

#include "../map.h"
#include <vector>
#include <math.h>

struct CarState {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double dyaw;
    double v;
    double dv;
    double ddv;
};


struct::Path smoothen_path(struct::Path anchor_points,
                           struct::Path path_remaining,
                           struct::CarPos car_pos,
                           double dt_end,
                           double v_end,
                           double dv_end
                           );

struct::CarState get_last_car_state(struct::Path path);


struct::Path convert_MCS_to_VCS(struct::Path path, CarPos car_pos);

struct::Path convert_VCS_to_MCS(struct::Path path, CarPos car_pos);

int get_next_anchor(struct::Path anchor_points, int idx_anchor_last, double x, double y);


/* Small helper functions */


void check_path(struct::Path path);

vector<double> calc_time_derivative(vector<double> x, double dt);

void test_spline_xy();

inline double get_last_elem(vector<double> x) {
    if (x.size() > 0) {
        return x[x.size()-1];
    }
    else {
        return 999;
    }
}

inline double get_yaw(double x_start, double y_start, double x_end, double y_end) {
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double yaw = atan2(dy,dx);
    return yaw;
}


inline double clip(double x, double x_min, double x_max) {
    x = std::min(x, x_max);
    x = std::max(x, x_min);
    return x;
}

template <typename T>
inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
}


#endif //GENERATE_PATH_H
