//
// Created by SYF on 10/7/20.
//

#ifndef AGENTSIM_GROUNDTRUTHPREDICTOR_HPP
#define AGENTSIM_GROUNDTRUTHPREDICTOR_HPP
#include "Predictor.hpp"

class GroundTruthPredictor: public Predictor{
public:
    GroundTruthPredictor(Agent* agent_ibt, double time_step, double horizon);

    PredictTra update(Vector currentState,std::vector<Agent*> agents);
    void set_traj(PredictTra traj);
};

#endif //AGENTSIM_GROUNDTRUTHPREDICTOR_HPP