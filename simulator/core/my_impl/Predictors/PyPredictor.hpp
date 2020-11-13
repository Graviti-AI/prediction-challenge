//
// Created by SYF on 8/2/20
//

#ifndef AGENTSIM_PYPREDPREDICTOR_HPP
#define AGENTSIM_PYPREDPREDICTOR_HPP

#include "Predictor.hpp"

class PyPredictor : public Predictor{
    public:
        PyPredictor(Agent* agent_ibt, double time_step, double horizon);

        PredictorType getType() const;
        PredictTra update(Vector currentState, std::vector<Agent*> agents);
        void set_traj(PredictTra traj);

    private:
        PredictTra ClientTraj;
};

#endif