//
// Created by SYF on 8/2/20
//

#include <assert.h>

#include "PyPredictor.hpp"
#include "../Simulator/Simulator.hpp"


PyPredictor::PyPredictor(MapInfo* map, double time_step, double horizon): Predictor(map,time_step,horizon){
    state = 0;
}

int PyPredictor::get_state(){
    return state;
}

void PyPredictor::set_state(int s){
    state=s;
}

void PyPredictor::set_client_traj(PredictTra uploaded_traj){
    assert(state == 1);
    ClientTraj = uploaded_traj;
    this->set_state(2);
}

PredictTra PyPredictor::update(Vector currentState, std::vector<Agent*> agents){
    assert(state == 3);
    this->set_state(0);
    return ClientTraj;
}
