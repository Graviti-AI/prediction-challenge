//
// Created by SYF on 8/2/20
//

#include <assert.h>
#include "PyPredictor.hpp"


PyPredictor::PyPredictor(Agent* agent_ibt, double time_step, double horizon): Predictor(agent_ibt,time_step,horizon){
}

PredictorType PyPredictor::getType() const{
    return PredictorType::PyPredictor;
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
        usleep(1e6 * SIM_TICK_SECOND); //TODO: change the sleep time
    }
    return ClientTraj;
}
