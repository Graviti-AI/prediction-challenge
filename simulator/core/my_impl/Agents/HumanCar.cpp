//
// Created by mscsim on 8/23/18.
//

#include "HumanCar.hpp"
#include "../Simulator/Simulator.hpp"

/// Constructor.
/// \param id id of the new human car.
/// \param initialState initial state of the human car.
HumanCar::HumanCar(int id, Vector initialState, Planner *planner, Controller *controller, Model *model)
        : Agent(id, initialState) {

}

HumanCar::HumanCar(int id, Vector initialState)
        : Agent(id, initialState) {

}
void HumanCar::Run() {
    // Get the input vector from the planner, by giving human inputs and agent state.

    vector<Agent *> agents =  Simulator::agentsForThread;
// Behaviour
    Vector input = planner->update(this->getState(), Simulator::humanInputsForThread.at(this), agents, behaviour->obstacles_info_);

    // Get the output of the controller.
    Vector intermediate = controller->update(input);

    // Calculate the next state of the agent from the model.
    Vector nextState = model->update(this->getState(), intermediate);

    this->setNextState(nextState); // Set the next state to apply, but not apply right now.
    this->setPreState(this->getState());
    this->applyNextState();
}
/// Type getter (overridden)
/// \return type (human car)
AgentType HumanCar::getType() const {
    return AgentType::HumanCar;
}
