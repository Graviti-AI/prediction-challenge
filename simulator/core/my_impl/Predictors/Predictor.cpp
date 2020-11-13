//
// Created by LCR on 7/18/20.
//
#include "Predictor.hpp"

Predictor::Predictor(Agent* agent_ibt, double time_step, double horizon){
    agent_ibt_ = agent_ibt;
    time_step_ = time_step;
    horizon_ = horizon;

    state = SubprocessState::fine;
}

SubprocessState Predictor::get_state(){
    return state;
}

void Predictor::set_state(SubprocessState s){
    state = s;
}
