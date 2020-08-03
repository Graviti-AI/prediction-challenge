//
// Created by LCR on 7/18/20.
//

#ifndef AGENTSIM_CONSTANTSPEEDPREDICTOR_HPP
#define AGENTSIM_CONSTANTSPEEDPREDICTOR_HPP
#include "Predictor.hpp"

class ConstantSpeedPredictor: public Predictor{
public:
    ConstantSpeedPredictor(MapInfo* map,double time_step,double horizon);
    PredictTra update(Vector currentState,std::vector<Agent*> agents);
};

#endif //AGENTSIM_CONSTANTSPEEDPREDICTOR_HPP