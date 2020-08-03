//
// Created by SYF on 8/2/20
//

#ifndef AGENTSIM_PYPEEDPREDICTOR_HPP
#define AGENTSIM_PYPEEDPREDICTOR_HPP

#include "Predictor.hpp"

#include <string>


class PyPredictor : public Predictor{
    public:
        PyPredictor(
            MapInfo* map, double time_step, double horizon, 
            std::string conda_env, std::string py_path);

        PredictTra update(Vector currentState, std::vector<Agent*> agents);

        static int PORTNUM;

    protected:
        int con;
        char msg[100];

};


#endif