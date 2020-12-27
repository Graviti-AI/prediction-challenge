//
// Created by LCR on 7/18/20.
//

#ifndef AGENTSIM_CONSTANTSPEEDPREDICTOR_HPP
#define AGENTSIM_CONSTANTSPEEDPREDICTOR_HPP
#include "Predictor.hpp"

// TODO: obseleted
class ConstantSpeedPredictor: public Predictor{
public:
    ConstantSpeedPredictor(Agent* agent_ibt, double time_step, double horizon);

    PredictorType getType() const;
    PredictTra update(Vector currentState,std::vector<Agent*> agents);
    void set_traj(PredictTra traj);
};

#endif //AGENTSIM_CONSTANTSPEEDPREDICTOR_HPP