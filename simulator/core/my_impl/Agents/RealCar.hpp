//
// Created by msclab on 10/30/18.
//

#ifndef AGENTSIM_REALCAR_H
#define AGENTSIM_REALCAR_H

#include "Agent.hpp"

///
/// RealCar is a kind of agent that is controlled by humans or autonomous driving program.
/// It is the car in the testing ground. It moves in the real world and sends its states to this program.
class RealCar : public Agent {

public:
    RealCar(int id, Vector initialState);
    explicit RealCar(int id, Vector initialState, Planner *planner, Controller *controller, Model *model);
    void Run();
    AgentType getType() const override;

private:

};


#endif //AGENTSIM_REALCAR_H
