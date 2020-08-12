//
// Created by SYF on 8/2/20
//

#ifndef AGENTSIM_PYPEEDPREDICTOR_HPP
#define AGENTSIM_PYPEEDPREDICTOR_HPP

#include "Predictor.hpp"
#include "../simulator_state.hpp"



class PyPredictor : public Predictor{
    public:
        PyPredictor(MapInfo* map, double time_step, double horizon);

        PredictTra update(Vector currentState, std::vector<Agent*> agents);

        void set_client_traj(PredictTra uploaded_traj);

    protected:

        bool flag;
        PredictTra ClientTraj;

};


#endif