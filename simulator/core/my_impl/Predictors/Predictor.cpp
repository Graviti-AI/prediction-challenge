//
// Created by LCR on 7/18/20.
//
#include "Predictor.hpp"

Predictor::Predictor(MapInfo* map,double time_step,double horizon){
    mapinfo_ = map;
    time_step_ = time_step;
    horizon_ = horizon;
    
    state = PredictorState::fine;
}

PredictorState Predictor::get_state(){
    return state;
}

void Predictor::set_state(PredictorState s){
    state = s;
}
