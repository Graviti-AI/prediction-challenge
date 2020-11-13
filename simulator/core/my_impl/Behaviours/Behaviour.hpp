//
// Created by mscsim on 8/23/18.
//

#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include <vector>
#include <ctime>

#include "../threadPool/Task.hpp"
#include "../Planners/Planner.hpp"
#include "../Controllers/Controller.hpp"
#include "../Models/Model.hpp"
#include "../alglib/interpolation.h"
#include "../Maps/MapInfo.hpp"
#include "../Predictors/Predictor.hpp"

typedef std::vector<double> Vector;
class Simulator;


/*
 * OVM parameters
 */
const double L = 20;      // System Size
const double C = 2.0;

// end of OVM parameters

/// This enumerator stands for the type of an Behaviour.
enum BehaviourType {
    IDM = 0,
    OVM,
    GFM
};

enum Mode {
    following = 0,
    merging,
    yielding,
    stopping,
    vanishing,
    synchronizing,
    allWayStopping,
    linechange,
    end
};
struct Params {
    double maxAcc = 3;
    double maxDec = -4;
    double maxVelocity = 50;
    double minGapBetweenCars = 2.5;
    double safeTimeHeadway = 1.8;
    double desiredVel = 8.0;
};
struct Obstacle_info{
    Agent* agentptr;
    TraPoints point_in;
    double distence;
    bool yielding=true;
};
struct Linechange_info{

};
/// The parent class of all Behaviours. An Behaviour has a state vector and an id.
/** You can get the current state, or set the next state followed by applying the next state.*/
class Behaviour {

public:
    Behaviour(Agent* agent_ibt, BehaviourType t);

    BehaviourType getType() {
        return type;
    }

    Mode getMode() {
        return mode;
    }

    virtual Vector update(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents) = 0;
    
    MapInfo* mapinfo_;
    std::vector<Obstacle_info> obstacles_info_;
    Linechange_info linechange_info_;
    PlannerPre new_pre_;
protected:
    float IDMAcceleration(const Vector& egoState, Vector& aheadVehicleState, double ego_length, double ahead_length);
    inline double V(double dx){ return tanh(dx - C)+tanh(C);}; /*!< calculate the optimal velocity */
    float OVMAcceleration(const Vector& egoState, Vector& aheadVehicleState);
    float GFMAcceleration(const Vector& egoState, Vector& aheadVehicleState);
    Params params;
    BehaviourType type;
    Mode mode;

    Agent* agent_ibt_;
};


#endif //BEHAVIOUR_HPP
