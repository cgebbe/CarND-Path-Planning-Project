#ifndef BEHAVIOR_STATES_H
#define BEHAVIOR_STATES_H

#include "behavior_machine.h"
#include "../map.h"
#include "generate_path.hpp"

// forward declare StateMachine due to circular dependency
class StateMachine;

class State {
public:
    virtual ~State();
    Path decide_path(StateMachine& machine,
                     CarPos& car_pos,
                     Path& path_remaining,
                     vector<OtherCar>& other_cars,
                     Map& map);
protected:
    void set_state(StateMachine& machine,
                   State* state);
    virtual State* decide_state(CarPos& car_pos,
                               Path& path_remaining,
                               vector<OtherCar>& other_cars,
                               Map& map);
    virtual Path get_path(CarPos& car_pos,
                          Path& path_remaining,
                          vector<OtherCar>& other_cars,
                          Map& map);
};


class Init : public State {
public:
    Init(int id_lane);
    virtual ~Init();
private:
    virtual State* decide_state(CarPos& car_pos,
                               Path& path_remaining,
                               vector<OtherCar>& other_cars,
                               Map& map);
    virtual Path get_path(CarPos& car_pos,
                          Path& path_remaining,
                          vector<OtherCar>& other_cars,
                          Map& map);
    int m_id_lane;
    int m_cnt_runs;
};

class KeepLane : public State {
public:
    KeepLane(int id_lane);
    virtual ~KeepLane();
private:
    virtual State* decide_state(CarPos& car_pos,
                               Path& path_remaining,
                               vector<OtherCar>& other_cars,
                               Map& map);
    virtual Path get_path(CarPos& car_pos,
                          Path& path_remaining,
                          vector<OtherCar>& other_cars,
                          Map& map);
    virtual float get_cost(CarPos& car_pos,
                           Path& path_remaining,
                           vector<OtherCar>& other_cars,
                           Map& map);
    int m_id_lane;
};



class ChangeLane : public State {
public:
    ChangeLane(int id_lane);
    virtual ~ChangeLane();
private:
    virtual State* decide_state(CarPos& car_pos,
                               Path& path_remaining,
                               vector<OtherCar>& other_cars,
                               Map& map);
    virtual Path get_path(CarPos& car_pos,
                          Path& path_remaining,
                          vector<OtherCar>& other_cars,
                          Map& map);
    virtual float get_cost(CarPos& car_pos,
                           Path& path_remaining,
                           vector<OtherCar>& other_cars,
                           Map& map);
    int m_id_lane;
};


#endif // BEHAVIOR_STATES_H
