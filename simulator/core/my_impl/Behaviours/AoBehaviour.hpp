//
// Created by LCR on 7/15/20.
//

#ifndef AOBEHAVIOUR_HPP
#define AOBEHAVIOUR_HPP

#include "Behaviour.hpp"

class AoBehaviour : public Behaviour {

public:
    AoBehaviour(BehaviourType t);
    Vector update(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Following(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Merging(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Stopping(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    void getObstacle(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    
    
    double last_time;
    std::vector<std::pair<int, int>> allWayStopWaitingidList;
};




#endif //AOBEHAVIOUR_HPP
