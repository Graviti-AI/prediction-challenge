//
// Created by SYF on 10/10/2020
//

#include <assert.h>
#include "NoPredictor.hpp"


NoPredictor::NoPredictor(Agent* agent_ibt, double time_step, double horizon): Predictor(agent_ibt,time_step,horizon){
}

PredictorType NoPredictor::getType() const{
    return PredictorType::NoPredictor;
}

void NoPredictor::set_traj(PredictTra traj){
    assert(false);  // python client wouldn't send back to this predictor.
}

PredictTra NoPredictor::update(Vector currentState, std::vector<Agent*> agents){
    assert(state == SubprocessState::fine);
    state = SubprocessState::wait4update;
    return PredictTra();
}
