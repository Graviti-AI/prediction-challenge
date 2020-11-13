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
    assert(state == SubprocessState::wait4upload);

    ClientTraj = traj;
    state = SubprocessState::wait4update;
}

PredictTra PyPredictor::update(Vector currentState, std::vector<Agent*> agents){
    assert(state == SubprocessState::fine);
    state = SubprocessState::wait4fetch;

    while (state != SubprocessState::wait4update){
        usleep(1e6 * SIM_TICK); //TODO: change the sleep time
    }
    return ClientTraj;
}
