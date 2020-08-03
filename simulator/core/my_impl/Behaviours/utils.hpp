#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>

#include <cstdio>
#include <vector>
#include <utility>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
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
#include "../Agents/Agent.hpp"

#define Pi 3.14159265

using namespace lanelet;
using namespace lanelet::matching;

// typedef std::vector<BasicPoint2d> TrajectoryPoints;
// typedef std::pair<int, TrajectoryPoints> Trajectory;
// typedef std::vector<double> Vector;



namespace mscUtils {
/*
std::vector<std::string> split(const std::string &str,const std::string &pattern)
    {
        //const char* convert to char*
        char * strc = new char[strlen(str.c_str())+1];
        strcpy(strc, str.c_str());
        std::vector<std::string> resultVec;
        char* tmpStr = strtok(strc, pattern.c_str());
        while (tmpStr != NULL)
        {
            resultVec.push_back(std::string(tmpStr));
            tmpStr = strtok(NULL, pattern.c_str());
        }

        delete[] strc;

        return resultVec;
    }

std::vector<Trajectory> readCSV(std::string filePath) {
    std::vector<Trajectory> res;
    std::ifstream infile;
    infile.open(filePath);
    std::string x, y, line;
    int id, lastId = 1;
    double xd, yd;
    std::vector<BasicPoint2d> trajectory_points;
    getline(infile, line); // ignore the first line
    while(getline(infile, line))
    {
        std::vector<std::string> values = split(line, ",");
        id = std::stoi(values[0].c_str());
        xd = atof(values[4].c_str());
        yd = atof(values[5].c_str());
        double co = std::cos(155 * Pi / 180.0);
        double si = std::sin(155 * Pi / 180.0);
        BasicPoint2d p(xd*co + yd*si, -xd*si + yd*co);

        if (lastId != id) {
            res.push_back(std::make_pair(lastId, trajectory_points));
            trajectory_points.clear();
        }
        trajectory_points.push_back(p);
        lastId = id;
    }
    return res;
}

ConstLanelets getTraveledLaneletPath (LaneletMap& map, Trajectory& trajectory) {
    ConstLanelets res;
    std::vector<BasicPoint2d>& points = trajectory.second;
    auto startPoint = points.begin();
    auto endPoint = points.end()-1;
    ConstLanelet startLanelet;
    ConstLanelet endLanelet;
    std::cout << " Tracking id : " << trajectory.first << std::endl;
    while (startLanelet.id() == 0 || endLanelet.id() == 0) {
        for (auto lanelet : map.laneletLayer) {
            if (startLanelet.id() == 0 && geometry::inside(lanelet, *startPoint)) {
                startLanelet = lanelet;
            }
            if (endLanelet.id() == 0 && geometry::inside(lanelet, *endPoint)) {
                endLanelet = lanelet;
            }        
        }
        if (startLanelet.id() == 0) startPoint+=10;
        if (endLanelet.id() == 0) endPoint-=10;
        if(startPoint + 10 > endPoint) {
            std::cout << "Cannot find point in map : id " << trajectory.first << std::endl;
            return res;
        }
    }

    res.push_back(startLanelet);
    if (startLanelet.id() == endLanelet.id()) {
        std::cout << " lanelet id : " << startLanelet.id() << std::endl;
        std::cout << "lase lanelet id : " << endLanelet.id() << std::endl;
        return res;
    }

    traffic_rules::TrafficRulesPtr trafficRules =
                traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    std::unique_ptr<routing::RoutingGraph> routingGraph  = routing::RoutingGraph::build(map, *trafficRules);
    Optional<routing::Route> route = routingGraph->getRoute(startLanelet, endLanelet, 0);

    if(!route) { 
        std::cout << "Routing error, : id " << trajectory.first << std::endl;
        return res;
    }
    for (std::vector<BasicPoint2d>::iterator iter = points.begin(); iter != points.end(); iter++) {
        BasicPoint2d p = *iter;
        auto closestLanelets = route->laneletMap()->laneletLayer.nearest(p, 2);
        if(!geometry::inside(res.back(), p)) {
            for (auto ll : closestLanelets) {
                if (geometry::intersects2d(ll, res.back()) && 
                    std::find(res.begin(), res.end(), ll) == res.end()) {
                    res.push_back(ll);
                    break;
                }
            }
        }
        // if (closestLanelet.id() != res.back().id() && closestLanelet.id()!=0) {
        //     res.push_back(closestLanelet);
        // }
    }
    // verify if the lanelet path is valid
    for (ConstLanelets::iterator iter = res.begin(); iter != res.end(); iter++) {
        // ConstLanelets::iterator nex = iter + 1;
        std::cout << "lanelet id :" << (*iter).id();
        // if (!geometry::intersects2d(*iter, *nex)) {
        //     std::cout << "Wrong path, : id " << trajectory.first << std::endl;
        //     std::cout << "lanelet id :" << (*iter).id() << "next : " << (*nex).id() <<  std::endl;
        // }
    }
    std::cout << "  lase lanelet id : " << endLanelet.id() << std::endl;
    std::cout << "------------------" << std::endl;

    return res;
}

std::vector<double> distanceToReferencePath(ConstLanelets& lanePath, Trajectory& trajectory) {
    std::vector<BasicPoint2d>& points = trajectory.second;
    std::vector<BasicPoint2d>::iterator point_iter = points.begin();
    ConstLanelets::iterator lanelet_iter = lanePath.begin();
    std::vector<double> res;
    while (point_iter != points.end()) {
        if (std::next(lanelet_iter) != lanePath.end() &&
            geometry::distanceToCenterline2d(*lanelet_iter, *point_iter) > 
            geometry::distanceToCenterline2d(*(std::next(lanelet_iter)), *point_iter)) {
                lanelet_iter++;       
        }
        double diff = geometry::distanceToCenterline2d(*lanelet_iter, *point_iter);
        // std::cout << "Diff : " << diff << std::endl;
        res.push_back(diff);
        point_iter++;
    }
    return res;
}

ConstLanelets reachableLanelets(const LaneletMap& map, int startLaneletId, float maxDistance) {
    ConstLanelet lanelet = map.laneletLayer.get(startLaneletId);
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    auto routingGraph  = routing::RoutingGraph::build(map, *trafficRules);
    ConstLanelets reachableSet = routingGraph->reachableSet(lanelet, maxDistance, 0);
    return reachableSet;
}

routing::LaneletPath getshortestRoutingPath(LaneletMap& map, int startLaneletId, int endLaneletId) {
    ConstLanelet lanelet = map.laneletLayer.get(startLaneletId);
    ConstLanelet toLanelet = map.laneletLayer.get(endLaneletId);
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    auto routingGraph  = routing::RoutingGraph::build(map, *trafficRules);
    Optional<routing::Route> route = routingGraph->getRoute(lanelet, toLanelet, 0);

    if(!route) std::cout << "No routing path from " << startLaneletId << " to " << endLaneletId << std::endl;
    routing::LaneletPath shortestPath = route->shortestPath();
    return shortestPath;
}
*/
std::vector<double> lineCurvatures (ConstLanelet& lanelet) ;

Optional<std::pair<bool, ConstLanelet> > findConflictLanelet (const routing::RoutingGraphConstPtr& routingGraphPtr, ConstLanelet& lanelet) ;

Agent* findAgentInLanelet(ConstLanelet& lanelet, std::vector<Agent*> agents) ;


std::vector<Agent* > findAgentsInLanelet(const Vector& egoState, ConstLanelet& lanelet, std::vector<Agent*> agents) ;

lanelet::ConstLanelet laneletFromLLSeq(const lanelet::LaneletSequence& llseq);

double getPt( double n1 , double n2 , double perc );


std::vector<std::pair<double, double>> BezierCurve(lanelet::BasicPoint2d p1, lanelet::BasicPoint2d p2, lanelet::BasicPoint2d p3, lanelet::BasicPoint2d p4);
 
std::vector<std::pair<double, double>> CILQRBezierCurve(lanelet::BasicPoint2d p1, lanelet::BasicPoint2d p2, lanelet::BasicPoint2d p3, 
                                                        lanelet::BasicPoint2d p4, double sampl_dis, double & start_dis, double & total_s); 

} //end namespace
