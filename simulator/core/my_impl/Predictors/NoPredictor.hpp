//
// Created by SYF on 10/10/2020
//

#ifndef AGENTSIM_NOPREDICTOR_HPP
#define AGENTSIM_NOPREDICTOR_HPP

#include "Predictor.hpp"
#include "../simulator_state.hpp"



class NoPredictor : public Predictor{
    public:
        NoPredictor(Agent* agent_ibt, double time_step, double horizon);

        PredictorType getType() const;
        PredictTra update(Vector currentState, std::vector<Agent*> agents);
        void set_traj(PredictTra traj);
};

#endif