#pragma once

#include <vector>
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
#include <chrono>

using namespace lanelet;
using namespace std::chrono;

enum TrafficLightState {
    Green = 0,
    Red = 1,
    Yellow
};

class Branch {
public:
    void addLanelet(ConstLanelet& lanelet) {
        relatedLanelets.push_back(lanelet);
    }
private:
    ConstLanelets relatedLanelets;
};

class TrafficInfoLight {
public:
    void setLightState(TrafficLightState& s) {
        state = s;
    }
    TrafficLightState getLightState() {
        return state;
    }
// private:
    int id;
    double last_time;
    double duration = 8.0;
    std::vector<LaneletSequence> relatedLaneletSequences;
    ConstLanelets relatedLanelets;
    TrafficLightState state = TrafficLightState::Red;
};

class TrafficLightGroup {
public:
    std::vector<TrafficInfoLight* > trafficLights;
    double duration = 5.0;
};

typedef std::vector<TrafficLightGroup* > TrafficLightGroups;

class TrafficInfoManager {
public:

    TrafficInfoManager(std::string& mapPath);
    std::vector<TrafficInfoLight*> findConlictTrafficLights(TrafficInfoLight* egoTrafficLight);
    // return state and remaining time
    TrafficInfoLight* findTrafficLightById(int id);
    TrafficLightState requestTrafficStateById(int id);

    TrafficInfoManager(TrafficInfoManager const&) = delete;
    void operator=(TrafficInfoManager const&) = delete;
    void Update();
    int currentIndex = 0;
    
private:
    std::pair<TrafficLightState, double> requestTrafficLightStateById(int id);
    bool isTrafficGroupExist(int id);
    std::vector<TrafficInfoLight*> trafficLights;
    TrafficLightGroups trafficLightGroups;
    std::chrono::time_point<std::chrono::system_clock> last_time;
    double duration = 5.0;
    bool isAllRed = false;

};
