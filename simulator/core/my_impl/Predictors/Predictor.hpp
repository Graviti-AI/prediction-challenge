//
// Created by LCR on 7/18/20.
//

#ifndef AGENTSIM_PREDICTOR_HPP
#define AGENTSIM_PREDICTOR_HPP

#include<vector>
#include <stdexcept>
#include "../simulator_state.hpp"

typedef std::vector<double> Vector;

class Agent;
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

///////////////////////////////////////////////

enum PredictorType {
    ConstantSpeedPredictor = 0,
    GroundTruthPredictor = 1,
    NoPredictor = 2,
    PyPredictor = 3,
};


class Predictor{
public:
    Predictor(Agent* agent_ibt, double time_step, double horizon);

    SubprocessState get_state();
    void set_state(SubprocessState s);

    virtual PredictorType getType() const = 0;
    virtual PredictTra update(Vector currentState,std::vector<Agent*> agents) = 0;
    virtual void set_traj(PredictTra traj) = 0;

protected:
    Agent* agent_ibt_; // agent_ibt means `the agent it belongs to`
    double time_step_;
    double horizon_;

    SubprocessState state;
};


#endif //AGENTSIM_AGENT_HPP
