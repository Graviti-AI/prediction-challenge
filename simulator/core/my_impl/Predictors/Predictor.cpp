//
// Created by LCR on 7/18/20.
//
#include "Predictor.hpp"

Predictor::Predictor(MapInfo* map,double time_step,double horizon){
    mapinfo_ = map;
    time_step_ = time_step;
    horizon_ = horizon;
}