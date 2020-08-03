//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_HUMANCAR_HPP
#define AGENTSIM_HUMANCAR_HPP

#include "Agent.hpp"

///
/// HumanCar is a kind of agent that is controlled by humans.
/// It is the "car" put in the lab. It doesn't move in the real world
class HumanCar : public Agent {

public:
    HumanCar(int id, Vector initialState);
    explicit HumanCar(int id, Vector initialState, Planner *planner, Controller *controller, Model *model);
    void Run();
    AgentType getType() const override;

private:


};


#endif //AGENTSIM_HUMANCAR_HPP
