#ifndef MACHINE_H
#define MACHINE_H

#include "behavior_states.hpp"
#include "generate_path.hpp"

// forward declare state due to circular dependency
class State;

class StateMachine
{
    friend class State;

public:
    StateMachine();

    Path get_next_path(CarPos& car_pos,
                       Path& path_remaining,
                       vector<OtherCar>& other_cars,
                       Map& map);
    ~StateMachine();

private:
    State* mState;
};

#endif
