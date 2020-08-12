//
// Created by LCR on 7/18/20.
//

#ifndef AGENTSIM_PREDICTOR_HPP
#define AGENTSIM_PREDICTOR_HPP

#include<vector>
#include <stdexcept>
#include "../Maps/MapInfo.hpp"

typedef std::vector<double> Vector;

class Agent;
struct TraPoints
{
    double t;
    double x;
    double y;
    double theta;
    double delta_theta;
    double v;
    double a;
    double jerk;
    ConstLanelet current_lanelet;
    double s_of_current_lanelet;
    double d_of_current_lanelet;
};
struct OneTra
{
    std::vector<TraPoints> Traj;
    std::vector<ConstLanelet> confilictlanes;
    double Probability;
};
struct PredictTra
{
    std::vector<OneTra> Trajs;
};
class Predictor{
public:
    Predictor(MapInfo* map,double time_step,double horizon);
    virtual PredictTra update(Vector currentState,std::vector<Agent*> agents) = 0;
    virtual void set_client_traj(PredictTra uploaded_traj) = 0;

    MapInfo* mapinfo_;
    double time_step_;
    double horizon_;
};


#endif //AGENTSIM_AGENT_HPP
