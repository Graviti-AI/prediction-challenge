//
// Created by SYF on 8/2/20
//

#include <assert.h>
#include "PyPredictor.hpp"


PyPredictor::PyPredictor(MapInfo* map, double time_step, double horizon): Predictor(map,time_step,horizon){
}


void PyPredictor::set_traj(PredictTra traj){
    assert(state == PredictorState::wait4upload);

    ClientTraj = traj;
    state = PredictorState::wait4update;
}

PredictTra PyPredictor::update(Vector currentState, std::vector<Agent*> agents){
    assert(state == PredictorState::fine);
    state = PredictorState::wait4fetch;

    while (state != PredictorState::wait4update){
        usleep(1e6 * SIM_TICK); //TODO: change the sleep time
    }
    return ClientTraj;
}
