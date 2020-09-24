//
// Created by LCR on 7/19/20.
//

#ifndef AGENTSIM_MAP_H
#define AGENTSIM_MAP_H
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

#include "TrafficInfo.hpp"
#include "../alglib/interpolation.h"
///
/// this class manage the agent routing info 
typedef std::vector<double> Vector;
class Agent;
using namespace lanelet;
using namespace lanelet::matching;


class MapInfo {
public:

    MapInfo(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr);

    ConstLanelet getCurrentLanelet() { return currentLanelet_;};
    int getCurrentLaneletId() { return currentLanelet_.id();};

    void setRoutingGraph(routing::RoutingGraphPtr& rgPtr);
    bool setRoutingPath(ConstLanelet& startLanelet, ConstLanelet& destinationLanelet);
    void setLaneletPath(ConstLanelets& lanelet_path);
    void init(int id, Vector initstate);

    void setCurrentLanelet(ConstLanelet& ll);
    ConstLanelet getNextLanelet();
    ConstLanelet findNextLanelet(ConstLanelet& ll);
    bool moveToLanelet(ConstLanelet& toLabelet);
    bool AsRoutingLineChangefirst(ConstLanelet& ll);

    Agent* findClosestPrecByLane(std::vector<Agent*> agents, double maxL);
    Agent* findClosestSuccByLane(std::vector<Agent*> agents);


    Vector getCurrentState() { return State;}
    double getS() { return s_;}
    int getId() {return self_id_;}
    LaneletSequence getShortestPath() {
        return shortestPath_.getRemainingLane(startLanelet_);
    }
    LaneletSequence getRemainingPath() {
        return shortestPath_.getRemainingLane(currentLanelet_);
    }

    LaneletSequence getRemainingPath(ConstLanelet& ll) {
        return shortestPath_.getRemainingLane(ll);
    }

    void update(Vector nextstate);

    bool HasArrivedDestination_ = false;
    bool RoutingLineChange_ = false;
    std::vector<std::pair<ConstLanelet, ConstLanelet>> RoutingLineChangePair_;
    std::pair<ConstLanelet, ConstLanelet> CurrentRoutingLineChangePair_;

    routing::RoutingGraphPtr routingGraphPtr_;
    lanelet::LaneletMapPtr mapPtr_;
    routing::LaneletPath shortestPath_;
    LineString2d mergingPath;
    bool marge_first = true;
    double original_merge_length = 0;
    //alglib::spline1dinterpolant spl_x;
    //alglib::spline1dinterpolant spl_y;

    //int mergingIndex = 0;
    //std::vector<std::pair<double, double>> mergingTrajectory;
    //LineString2d mergingPath;
    //LineString2d wholePath;

/*
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
*/
protected:
    // x, y, yaw, x_v, y_v, yaw_rate, acceleration, desired velocity
    // const int dimState; /*!< the dimension of the Behaviour state */
    Vector State; /*!< next state which is calculated from model and waiting for apply */

    ConstLanelet currentLanelet_;

private:

    ConstLanelet startLanelet_;
    ConstLanelet destinationLanelet_;

    

    double s_;
    int  self_id_;
    double last_merging_dis = DBL_MAX;

    bool first_ = true;
};


#endif //AGENTSIM_MAP_H
