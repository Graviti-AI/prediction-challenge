//
// Created by SYF on 8/2/20
//

#include <assert.h>

#include "PyPredictor.hpp"
#include "../Simulator/Simulator.hpp"


PyPredictor::PyPredictor(MapInfo* map, double time_step, double horizon): Predictor(map,time_step,horizon){
    flag=false;
}

void PyPredictor::set_client_traj(PredictTra uploaded_traj){
    flag=true;
    ClientTraj = uploaded_traj;
}

PredictTra PyPredictor::update(Vector currentState, std::vector<Agent*> agents){
    while (flag == false){
        sleep(1.0);
    }

    assert(flag == true);

    flag = false;
    return ClientTraj;
}
