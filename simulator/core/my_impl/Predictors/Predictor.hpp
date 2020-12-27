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
struct TraPoints // the data type of points used in the simulator
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


///////////////////////////////////////////////

// designed for py_predictor
enum PredictorState {
    fine = 0,           // ready for the next update time
    wait4fetch = 1,     // the py_predictor is waiting, and the historical trajectory hasn't been sent to the client
    wait4upload = 2,    // the py_predictor is waiting, and the historical trajectory has been sent to the client
    wait4update = 3,    // the prediction has done

    // NOTE: If the predictor is not a py_predictor, directly set the state as `wait4update` when going into `xxPredictor::update()`
};

enum PredictorType {
    ConstantSpeedPredictor = 0,
    GroundTruthPredictor = 1,
    NoPredictor = 2,
    PyPredictor = 3,
};


class Predictor{
public:
    Predictor(Agent* agent_ibt, double time_step, double horizon); // ibt means `it belongs to`

    //about PredictorState
    PredictorState get_state();
    void set_state(PredictorState s);

    virtual PredictorType getType() const = 0;
    virtual PredictTra update(Vector currentState,std::vector<Agent*> agents) = 0; // the main function
    virtual void set_traj(PredictTra traj) = 0; // set the predicted results from the client

protected:
    Agent* agent_ibt_; // agent_ibt means `the agent it belongs to`
    double time_step_;
    double horizon_;

    PredictorState state;
};


#endif //AGENTSIM_AGENT_HPP
