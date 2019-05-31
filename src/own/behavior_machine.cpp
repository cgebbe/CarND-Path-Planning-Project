#include "behavior_machine.h"

StateMachine::StateMachine() {
    mState = new Init();
}

StateMachine::~StateMachine() {
    delete mState;
}

Path StateMachine::get_next_path(CarPos &car_pos,
                            Path &path_remaining,
                            vector<OtherCar> &other_cars,
                            Map &map
                            )
{
    return mState->get_next_path(*this,
                                 car_pos,
                                 path_remaining,
                                 other_cars,
                                 map
                                 );
}

