#include "behavior_machine.h"

StateMachine::StateMachine() {
    int id_lane = 1; // 0 = left, 1 = center, 2 = right
    mState = new Init(id_lane);
}

StateMachine::~StateMachine() {
    delete mState;
}

Path StateMachine::decide_path(CarPos &car_pos,
                            Path &path_remaining,
                            vector<OtherCar> &other_cars,
                            Map &map
                            )
{
    return mState->decide_path(*this,
                                 car_pos,
                                 path_remaining,
                                 other_cars,
                                 map
                                 );
}

