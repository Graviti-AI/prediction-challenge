#include "TrafficInfo.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <ctime>
#include <ratio>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
using namespace lanelet;


/// Constructor, load all traffic lights and then put them into different group
/// \param mapPath, the path of map file
TrafficInfoManager::TrafficInfoManager(std::string& mapPath) {
    // this line is for the rendering map
    //LaneletMapPtr map = load(mapPath, projection::UtmProjector(Origin({37.90030552764608, -122.30152928144717})));
    LaneletMapPtr map = load(mapPath, projection::UtmProjector(Origin({0.0, 0.0})));
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);

    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

    for (auto lanelet : map->laneletLayer) {
        auto  tmpTrafficLights = lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
        if (tmpTrafficLights.empty()) {
            continue;
        }

        // check if this group exists already
        int id = traits::getId(*tmpTrafficLights.front());
        if (isTrafficGroupExist(id)) {
            continue;
        }
        TrafficLightGroup* tmpGroup = new TrafficLightGroup();
        for (auto tL : tmpTrafficLights) {
            TrafficInfoLight* trafficLight = new TrafficInfoLight();
            trafficLight->id = traits::getId(*tL);
            ConstLanelets followingLanelets = routingGraph->following(lanelet);
            trafficLight->relatedLanelets.push_back(lanelet);

            for (auto ll : followingLanelets) {
                LaneletSequence llsq = LaneletSequence({lanelet, ll});
                ConstLanelet currLanelet = ll;
                while(llsq.size() < 4 && !routingGraph->following(currLanelet).empty()) {
                    currLanelet = routingGraph->following(currLanelet).at(0);
                    LaneletSequence newSequence = LaneletSequence({currLanelet});
                    
                    llsq = LaneletSequence({llsq, newSequence});
                };
                trafficLight->relatedLaneletSequences.push_back(llsq);
            }

            tmpGroup->trafficLights.push_back(trafficLight);
        }

        trafficLightGroups.push_back(tmpGroup);
    }
    // Initialize time stamp
    last_time = std::chrono::system_clock::now();
}

/// Find a traffic light by id
/// \return a traffic light
TrafficInfoLight* TrafficInfoManager::findTrafficLightById(int id) {
    for(auto trafficLight : trafficLights) {
        if(trafficLight->id == id) {
            return trafficLight;
        }
    }
    // if it does not exist, creat a new one
    TrafficInfoLight* newTrafficLight = new TrafficInfoLight();
    newTrafficLight->id = id;
    return newTrafficLight;
}

/// This function is supposed to find the traffic light who have conflict region with ego light. It means those light could not be green in the same time.
/// \return conflict traffic lights
std::vector<TrafficInfoLight*> TrafficInfoManager::findConlictTrafficLights(TrafficInfoLight* egoTrafficLight) {
    std::vector<TrafficInfoLight* > resVec;
    for(auto trafficLight : trafficLights) {
        if (trafficLight->id == egoTrafficLight->id)
            continue;
        for (LaneletSequence llsq : trafficLight->relatedLaneletSequences) {
            for (LaneletSequence egollsq : egoTrafficLight->relatedLaneletSequences) {
                if (geometry::intersects3d(llsq.centerline3d(), egollsq.centerline3d())){
                    resVec.push_back(trafficLight);
                }
            }
        }
    }
    return resVec;
}

/// if a traffic light exists

bool TrafficInfoManager::isTrafficGroupExist(int id) {
    for (auto group : trafficLightGroups) {
        for (auto trafficLight : group->trafficLights) {
            if (trafficLight->id == id) {
                return true;
            }
        }
    }
    return false;
}
/// Update traffic lights' status
void TrafficInfoManager::Update() {
    if (trafficLightGroups.size()==0) return;
    TrafficLightGroup* tmpGroup = trafficLightGroups[currentIndex];
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    double time_gap =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count()/1000.0;
    if (time_gap > tmpGroup->duration) {
        isAllRed = true;
    }

    if (time_gap > tmpGroup->duration + 2.0) {
        isAllRed = false;
        last_time = current_time;
        currentIndex = (++currentIndex) % trafficLightGroups.size();
    }
}

/// interface to agents, for requesting the current status of traffic light
TrafficLightState TrafficInfoManager::requestTrafficStateById(int id) {
    TrafficLightGroup* greenGroup = trafficLightGroups[currentIndex];
    for (auto trafficLight : greenGroup->trafficLights) {
            if (trafficLight->id == id) {
                return TrafficLightState::Green;
            }
    }
    return TrafficLightState::Red;
}
