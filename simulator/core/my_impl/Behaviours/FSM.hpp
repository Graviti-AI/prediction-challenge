

#ifndef FSM_HPP
#define FSM_HPP
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include "../threadPool/Task.hpp"
#include "../Agents/Agent.hpp"
#include "../Planners/Planner.hpp"
#include "../Controllers/Controller.hpp"
#include "../Models/Model.hpp"
#include "../alglib/interpolation.h"
#include <vector>
#include <ctime>
#include "Behaviour.hpp"
using namespace lanelet;
using namespace lanelet::matching;
// typedef std::vector<double> Vector;
class Simulator;



enum FSMMode {
        cruise=0,
        merge,
        carfollowing,
        changelane,
        intersection_pass,
        intersection_stop
};


/// The parent class of all Behaviours. An FSM has a state vector and an id.
/** You can get the current state, or set the next state followed by applying the next state.*/
class FSM {

public:
    int mergingIndex = 0;
    std::vector<std::pair<double, double>> mergingTrajectory;
    LineString2d mergingPath;
    LineString2d wholePath;
    FSM(BehaviourType t, LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr);
    void generateRoutingReferencePath();
    LineString2d getRoutingReferencePath();
    LineString2d getEgoReferencePath();
    LineString2d getLeftReferencePath();
    LineString2d getRightReferencePath();

    std::vector<Agent* > TargetAents;

    // explicit FSM(int id, Vector initialState, Planner *planner, Controller *controller, Model *model);
    void init();
    int getId() const;

    Vector getState() const;
    void update_s(double s);
    void setNextState(Vector state);
    void setPreState(Vector state);
    void applyNextState();
    void setPlanner(Planner *p);
    void setController(Controller *c);
    void setModel(Model *m);
    void Add_s(double s) {s_ += s;};
    ConstLanelet getCurrentLanelet() { return currentLanelet_;}
    int getCurrentLaneletId() { return currentLanelet_.id();}
    void setRoutingGraph(routing::RoutingGraphPtr& rgPtr);
    bool setRoutingPath(ConstLanelet& startLanelet, ConstLanelet& destinationLanelet);
    void setCurrentLanelet(ConstLanelet& current);
    Vector getCurrentState();
    ConstLanelet getNextLanelet();
    bool moveToLanelet(ConstLanelet& toLabelet);

    std::vector<Agent* >& getRelatedAgents() {
        return relatedAgents;
    }
    std::vector<Agent* > getSurroundingAgents(Vector egoState, std::vector<Agent*> agents);
    std::vector<Agent* > getOtherAgents(Vector egoState, std::vector<Agent*> agents);
    std::vector<Agent* > getTargetAgents();
    BehaviourType getType() {
        return type;
    }

    FSMMode getMode() {
        return mode;
    }

    double getS() {
        return s_;
    }

    LaneletSequence getShortestPath() {
        return shortestPath_.getRemainingLane(startLanelet_);
    }

    Vector findClosestPrecByLane(const Vector& egoState, std::vector<Agent*> agents, int laneId, double maxL = 50.0);
    Vector findClosestSuccByLane(const Vector& egoState, std::vector<Agent*> agents, int laneId);
    FSMMode GetMode(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Update(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector CarFollowing(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Merge(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Cruise(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Changelane(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Intersection_pass(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Intersection_stop(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    double Compute_segment_s(const Vector& currentState);
    alglib::spline1dinterpolant spl_x;
    alglib::spline1dinterpolant spl_y;
    struct Params {
        double maxAcc = 3;
        double maxDec = -4;
        double maxVelocity = 50;
        double minGapBetweenCars = 6.5;
        double safeTimeHeadway = 1.8;
    };
protected:
    float IDMAcceleration(const Vector& egoState, Vector& aheadVehicleState);
    inline double V(double dx){ return tanh(dx - C)+tanh(C);}; /*!< calculate the optimal velocity */

    // x, y, yaw, x_v, y_v, yaw_rate, acceleration, desired velocity
    // const int dimState; /*!< the dimension of the FSM state */
    Vector nextState; /*!< next state which is calculated from model and waiting for apply */

    Params params;

    ConstLanelet currentLanelet_;

private:
    BehaviourType type;
    FSMMode mode;

    ConstLanelet startLanelet_;
    ConstLanelet destinationLanelet_;
    std::vector<ConstLanelet> allWayStopWaitingList;
    std::vector<Agent* > relatedAgents;
    routing::LaneletPath shortestPath_;

    routing::RoutingGraphPtr routingGraphPtr_;
    lanelet::LaneletMapPtr mapPtr_;

    double last_time;

    double s_;
    double globleS = 0;

    double last_merging_dis = DBL_MAX;

    Agent* opponentCar;
    Vector predictOpponentState;

    LineString2d purePursuitPath;
    bool first_ = true;
};


#endif //FSM_HPP
