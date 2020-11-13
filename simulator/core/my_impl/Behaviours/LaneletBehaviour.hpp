//
// Created by mscsim on 8/23/18.
//

#ifndef LANELETBEHAVIOUR_HPP
#define LANELETBEHAVIOUR_HPP

#include <vector>
#include <ctime>

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

#include "Behaviour.hpp"
#include "../threadPool/Task.hpp"
#include "../Planners/Planner.hpp"
#include "../Controllers/Controller.hpp"
#include "../Models/Model.hpp"
#include "../alglib/interpolation.h"


typedef std::vector<double> Vector;
class Simulator;

using namespace lanelet;
using namespace lanelet::matching;



/// The parent class of all Behaviours. An Behaviour has a state vector and an id.
/** You can get the current state, or set the next state followed by applying the next state.*/
class LaneletBehaviour : public Behaviour {

public:
    int mergingIndex = 0;
    std::vector<std::pair<double, double>> mergingTrajectory;
    LineString2d mergingPath;
    LineString2d wholePath;
    LaneletBehaviour(Agent* agent_ibt, BehaviourType t, LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr);
    void generateRoutingReferencePath();

    // explicit Behaviour(int id, Vector initialState, Planner *planner, Controller *controller, Model *model);
    void init();
    int getId() const;

    Vector getState() const;
    void setNextState(Vector state);
    void setPreState(Vector state);
    void applyNextState();
    void setPlanner(Planner *p);
    void setController(Controller *c);
    void setModel(Model *m);

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
    BehaviourType getType() {
        return type;
    }

    Mode getMode() {
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

    Vector update(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Following(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Merging(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Yielding(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Stopping(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);

    alglib::spline1dinterpolant spl_x;
    alglib::spline1dinterpolant spl_y;

    // For CILQR Planner
    alglib::spline1dinterpolant route_x, route_y;
    alglib::spline1dinterpolant left_lane_x, left_lane_y;
    alglib::spline1dinterpolant ego_lane_x, ego_lane_y;
    alglib::spline1dinterpolant right_lane_x, right_lane_y;
    LineString2d route_path, left_lane, ego_lane, right_lane;
    Vector route_path_each_len, left_lane_each_len, ego_lane_each_len, right_lane_each_len; // the total length from start node to each node .._len[i] on LineString2d
    double length_route_path, length_ego_lane, length_left_lane, length_right_lane; // the length of LineString2d
    double route_start_s, left_start_s, ego_start_s, right_start_s; // position of init point in each plan
    int route_last_index, left_last_index, ego_last_index, right_last_index;
    void getRoutingReferencePath(Vector & lane_change, double & total_length);
    void getEgoReferencePath(double stop_length, double & total_length);
    void getLeftReferencePath(double stop_length, double & total_length);
    void getRightReferencePath(double stop_length, double & total_length);
    std::pair<int, int> getLeftAndRightLanletId(ConstLanelet & curLanlet);
    bool updateCurrentLanelet(const BasicPoint2d & point);
    bool insideShortestPath(const ConstLanelet & checkLaneLet);
    routing::LaneletPath getShortestPathCILQR() {
        return shortestPath_;
    }
    ConstLanelet startLanelet_;
    ConstLanelet destinationLanelet_;
protected:
    float IDMAcceleration(const Vector& egoState, Vector& aheadVehicleState);
    inline double V(double dx){ return tanh(dx - C)+tanh(C);}; /*!< calculate the optimal velocity */
    float OVMAcceleration(const Vector& egoState, Vector& aheadVehicleState);
    float GFMAcceleration(const Vector& egoState, Vector& aheadVehicleState);

    // x, y, yaw, x_v, y_v, yaw_rate, acceleration, desired velocity
    // const int dimState; /*!< the dimension of the Behaviour state */
    Vector nextState; /*!< next state which is calculated from model and waiting for apply */

    Params params;

    ConstLanelet currentLanelet_;

private:
    BehaviourType type;
    Mode mode;


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


#endif //LANELETBEHAVIOUR_HPP
