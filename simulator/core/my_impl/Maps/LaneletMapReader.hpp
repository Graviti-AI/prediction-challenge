//
// Created by LCR on 7/19/20.
//

#ifndef AGENTSIM_LANELETMAPREADER_H
#define AGENTSIM_LANELETMAPREADER_H
#include <vector>
#include <ctime>

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
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
using namespace lanelet;
using namespace lanelet::matching;
typedef std::map<int, std::vector<ConstLanelet>> ConflictLane;
class LaneletMapReader{
public:
    LaneletMapReader(std::string MapPath, double origin_x,double origin_y);

    LaneletMapPtr map;
    routing::RoutingGraphPtr routingGraph;
    ConflictLane ConflictLane_;
    std::vector<int> StartLaneletIds;
    std::vector<int> EndLaneletIds;
};





#endif //AGENTSIM_LANELETMAPREADER_H
