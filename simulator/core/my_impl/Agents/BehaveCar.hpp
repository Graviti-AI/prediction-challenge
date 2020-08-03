//
// Created by LCR on 7/18/20.
//

#ifndef AGENTSIM_BEHAVECAR_HPP
#define AGENTSIM_BEHAVECAR_HPP

#include "Agent.hpp"

///
/// BehaveCar is a kind of agent that only use behaviour class for update.
/// It is the "car" put in the lab. It doesn't move in the real world
class BehaveCar : public Agent {

public:
    BehaveCar(int id, Vector initialState);
    explicit BehaveCar(int id, Vector initialState, Planner *planner, Controller *controller, Model *model);
    void Run();
    AgentType getType() const override;

private:


};


#endif //AGENTSIM_BEHAVECAR_HPP
