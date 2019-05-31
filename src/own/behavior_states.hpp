#ifndef BEHAVIOR_STATES_H
#define BEHAVIOR_STATES_H

#include "behavior_machine.h"
#include "../map.h"
#include "generate_path.hpp"

// forward declare StateMachine due to circular dependency
class StateMachine;

class State {
public:
    virtual Path get_next_path(StateMachine& machine,
                               CarPos& car_pos,
                               Path& path_remaining,
                               vector<OtherCar>& other_cars,
                               Map& map);
    virtual ~State();
protected:
    void set_state(StateMachine& machine,
                   State* state);
};


class Init : public State {
public:
    virtual Path get_next_path(StateMachine& machine,
                       CarPos& car_pos,
                       Path& path_remaining,
                       vector<OtherCar>& other_cars,
                       Map& map);
    virtual ~Init();
};

class KeepLane : public State {
public:
    KeepLane();
    virtual Path get_next_path(StateMachine& machine,
                       CarPos& car_pos,
                       Path& path_remaining,
                       vector<OtherCar>& other_cars,
                       Map& map);
    virtual ~KeepLane();
private:
    int lane_id;
};



#endif // BEHAVIOR_STATES_H
