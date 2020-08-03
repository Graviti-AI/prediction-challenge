//
// Created by mscsim on 8/23/18.
//

#include "PedestrianAgent.hpp"
#include "../Simulator/Simulator.hpp"
#include <stdexcept>
#include <iostream>
#include <fstream>

using namespace std;
/// Constructor.
/// \param id the id of the agent.
/// \param initialState the initial state of the agent.
PedestrianAgent::PedestrianAgent(int id, Vector initialState, Vector destination) :id(id), dimState(initialState.size()), arrivalFlag(false){
    this->state = initialState;
	this->destination = destination;
}

PedestrianAgent::PedestrianAgent(int id, Vector initialState, Vector destination, Planner *planner, Controller *controller, Model *model) :id(id), dimState(initialState.size()), planner(planner), controller(controller), model(model), arrivalFlag(false){
    this->state = initialState;
    this->destination = destination;
}

/// Get the agent id.
/// \return agent id
int PedestrianAgent::getId() const {
    return this->id;
}

bool PedestrianAgent::getArrivalFlag() const {
	return this->arrivalFlag;
}

/// Get the current state vector of the agent.
/// \return current state.
Vector PedestrianAgent::getState() const {
    return this->state;
}

/// Set the next state vector of the agent.
/// \param state next state to set (but not applied immediately)
void PedestrianAgent::setNextState(Vector state) {
    if (this->dimState != state.size()) {
        throw std::runtime_error("bad dimension");
    }
    this->nextState = state;
}

///
/// Apply the next state which was previously set.
void PedestrianAgent::applyNextState() {
    this->num ++;
    if (this->nextState.size() != this->dimState) {
        std::cout << num;
        throw std::runtime_error("bad dimension");
    }
    this->state = this->nextState;

    this->nextState = Vector();
}

void PedestrianAgent::Run() {
    // Get the input vector from the planner, by giving human inputs and agent state.
    Vector input = planner->update(this->getState(), Simulator::humanInputsForThread.at(this), Simulator::agentsForThread, std::vector<Obstacle_info>());
    //Vector input = planner->update(this->getState(), Vector{0,0,0}, Simulator::agentsForThread);

    // Get the output of the controller.
    Vector intermediate = controller->update(input);

    // Calculate the next state of the agent from the model.
    Vector nextState = model->update(this->getState(), intermediate);

    //Vector nextState = this->getState();
    //nextState[0] += 0.1;
    //nextState[1] += 0.1;

    this->setNextState(nextState); // Set the next state to apply, but not apply right now.
    this->applyNextState();
}

void PedestrianAgent::setPlanner(Planner *p){
    this->planner = p;
}

void PedestrianAgent::setController(Controller *c){
    this->controller = c;
}

void PedestrianAgent::setModel(Model *m){
    this->model = m;
}