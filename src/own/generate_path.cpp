#include <vector>
#include "generate_path.hpp"
#include "../helpers.hpp"

struct::path get_next_path(struct::CarPos& car_pos,
                           struct::Map& map)
{
    struct::path next_path;
    double vel_in_mph = 49.1;
    double vel_in_m_per_s = vel_in_mph * 0.44704;
    double dist_per_20ms_in_m = vel_in_m_per_s * 0.02;
    for (int i = 0; i < 50; ++i) {
        double next_s = car_pos.s + (i+1)*dist_per_20ms_in_m;
        double next_d = 6; // center = 0, d_lane = 4m, d=6m -> center of middle lane
        std::vector<double> xy = convert_sd_to_xy(next_s, next_d, map);

        next_path.x.push_back(xy[0]);
        next_path.y.push_back(xy[1]);
    }
    return next_path;
}
