#include "../map.h"

struct path {
    vector<double> x;
    vector<double> y;
};

struct::path get_next_path(struct::CarPos& car_pos,
                           struct::Map& map);
