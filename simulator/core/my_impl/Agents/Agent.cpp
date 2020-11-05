//
// Created by mscsim on 8/23/18.
//

#include "Agent.hpp"
#include "../Simulator/Simulator.hpp"
#include <stdexcept>
#include <iostream>
#include <fstream>

using namespace std;
/// Constructor.
/// \param id the id of the agent.
/// \param initialState the initial state of the agent.
Agent::Agent(int id, Vector initialState) :id(id), dimState(initialState.size()){
    this->state = initialState;

    is_ego_car_ = false;
    in_predictor = nullptr;
    ex_predictor = nullptr;
}
Agent::Agent(int id, Vector initialState, Planner *planner, Controller *controller, Model *model)
: id(id), dimState(initialState.size()), planner(planner), controller(controller), model(model) {
    this->state = initialState;
    
    is_ego_car_ = false;
    in_predictor = nullptr;
    ex_predictor = nullptr;
}


/// Get the agent id.
/// \return agent id
int Agent::getId() const {
    return this->id;
}

/// Get the current state vector of the agent.
/// \return current state.
Vector Agent::getState() const {
    return this->state;
}

/// Get the previous state vector of the agent
/// \return pre_state.

std::vector<Vector>  Agent::get_preState() const{
    return this->preState;
}

/// Current state setter
void Agent::setState(Vector st) {
    this->state = st;
}

/// Set the next state vector of the agent.
/// \param state next state to set (but not applied immediately)
void Agent::setNextState(Vector state) {
    if (this->dimState != state.size()) {
        throw std::runtime_error("bad dimension");
    }
    this->nextState = state;
}

/// Set the pre state vector of the agent.
/// \param state current state
void Agent::setPreState(Vector state) {
    if (this->dimState != state.size()) {
        throw std::runtime_error("bad dimension");
    }
    this->preState.push_back(state);
}

///
/// Apply the next state which was previously set.
void Agent::applyNextState() {
    this->num ++;
    if (this->nextState.size() != this->dimState) {
        std::cout << num;
        throw std::runtime_error("bad dimension");
    }
    this->state = this->nextState;

    this->nextState = Vector();
}

/*
void Agent::Run(Planner* planner, Controller* controller, Model* model, std::vector<Agent*> agents, Vector &humanInputs) {
    // Get the input vector from the planner, by giving human inputs and agent state.
    Vector input = (*planner).update(this->getState(), humanInputs, agents);

    // Get the output of the controller.
    Vector intermediate = (*controller).update(input);

    // Calculate the next state of the agent from the model.
    Vector nextState = (*model).update(this->getState(), intermediate);

    this->setNextState(nextState); // Set the next state to apply, but not apply right now.
}
 */
///
///Calculate next state via agent's planner, controller and model.
void Agent::Run() {
    assert(false); //TODO: this method has not been updated;

    /*
    // Get the input vector from the planner, by giving human inputs and agent state.
    if (isRunning) {
        return;
    } else {
        isRunning = true;
    }
    vector<Agent *> agents =  Simulator::agentsForThread;
    // Behaviour
    Vector BehaviourState = behaviour->update(this->getState(), Simulator::humanInputsForThread[this], agents);
    Vector input;

    switch (behaviour->getMode()) {
    case Mode::following:
        input = fplanner->update(this->getState(), Simulator::humanInputsForThread[this], agents, behaviour->obstacles_info_);
        if (fplanner->false_flag) input = BehaviourState;
        break;
    case Mode::merging:
        input = lplanner->update(this->getState(), Simulator::humanInputsForThread[this], agents, behaviour->obstacles_info_);
        if (lplanner->false_flag) 
        input = BehaviourState;
        break;
    case Mode::linechange:
        input = lplanner->update(this->getState(), Simulator::humanInputsForThread[this], agents, behaviour->obstacles_info_);
        if (lplanner->false_flag) 
        input = BehaviourState;
        break;
    case Mode::stopping:
        input = BehaviourState;
        break;
    case Mode::allWayStopping:
        input = BehaviourState;
        break;
    default:
        input = BehaviourState;
        break;
    }

    // Get the output of the controller.
    Vector intermediate = controller->update(input);
    // Calculate the next state of the agent from the model.
    Vector nextState = model->update(this->getState(), intermediate);
    mapinfo->update(nextState);
    if (mapinfo->HasArrivedDestination_) hasReachedDestinaiton = true;
    
    this->setNextState(nextState); // Set the next state to apply, but not apply right now.
    this->setPreState(this->getState());
    this->applyNextState();
    PredictTra_ = predictor->update(nextState, agents);
    */
}
///
/// \param p the planner that will be set to the agent
void Agent::setPlanner(Planner *p) {
    this->planner = p;
}
///
/// \param c the controller that will be set to the agent
void Agent::setController(Controller *c) {
    this->controller = c;
}
///
/// \param m the model that will be set to the agent
void Agent::setModel(Model *m) {
    this->model = m;
}

void Agent::setBehaviour(Behaviour *b){
    this -> behaviour = b;
}

void Agent::setMapinfo(MapInfo *m){
    this -> mapinfo = m;
}

void Agent::setfollowingPlanner(Planner *p){
    this->fplanner = p;
}
void Agent::setlinechangePlanner(Planner *p){
    this->lplanner = p;
}

bool Agent::isEgoCar() const{
    return is_ego_car_;
}
void Agent::setEgoCar(){
    assert(is_ego_car_ == false);
    is_ego_car_ = true;
}

void Agent::setInPredictor(Predictor *p){
    in_predictor = p;
}
Predictor* Agent::getInPredictor(){
    return in_predictor;
}

void Agent::setExPredictor(Predictor *p){
    ex_predictor = p;
}
Predictor*  Agent::getExPredictor(){
    return ex_predictor;
}

void Agent::setPlanner(Planner* p) {
    planner = p;
}

Planner* Agent::getPlanner() {
    return planner;
}

void Agent::setLPlanner(Planner* p) {
    lplanner = p;
}

Planner* Agent::getLPlanner() {
    return lplanner;
}
   
void Agent::setFPlanner(Planner* p) {
    fplanner = p;
}

Planner* Agent::getFPlanner() {
    return fplanner;
}

void Agent::setPlannedTraj(std::vector<TraPoints> p) {
    planned_traj = p;
}

std::vector<TraPoints> Agent::getPlannedTraj() {
    return planned_traj;
}