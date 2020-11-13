#pragma once

#include <random>
#include <ctime>
#include <vector>
#include "LaneletBehaviour.hpp"
#include "FSM.hpp"

//std::vector<int> startLaneletIds{30228, 30255, 30377, 30020, 30426, 30423, 30220, 30229, 30086, 30224, 30076, 30415, 30365}; // 30419
//std::vector<int> endLaneletIds{30036, 30368, 30303, 30009, 30396, 30039, 30408, 30124, 30417, 30091, 30265, 30252, 30416, 30274};
namespace CarGenerator {

/* Old Version
/// generate a behavioral model
/// \param mapPtr the pointer of lanelet map
/// \param rgPth the pointer of routing map, get navigation info
/// \param type Behaviour model type
/// \return  behaviour model 
LaneletBehaviour* generateBehaviourModel(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr, BehaviourType type, std::vector<int>& startLaneletIds, std::vector<int>& endLaneletIds) {
    int id = random();
    int i = random()%startLaneletIds.size();
    int startLaneletId = startLaneletIds[i];
    i = random()%endLaneletIds.size();
    int endLaneletId = endLaneletIds[i];
    // std::cout << "New car generated " << std::endl;
    LaneletBehaviour* behaviour = new LaneletBehaviour(type, mapPtr, rgPtr);
    ConstLanelet fromLanelet = mapPtr->laneletLayer.get(startLaneletId);
    ConstLanelet toLanelet = mapPtr->laneletLayer.get(endLaneletId);
    while (!behaviour->setRoutingPath(fromLanelet, toLanelet)) {
        fromLanelet = mapPtr->laneletLayer.get(startLaneletIds[random()%startLaneletIds.size()]);
        toLanelet = mapPtr->laneletLayer.get(endLaneletIds[random()%endLaneletIds.size()]);
    }
    behaviour->setCurrentLanelet(fromLanelet);
    behaviour->generateRoutingReferencePath();
    return behaviour;
}
*/

 
/////////////////////////////////////////////
// Jinning Behavior model
/* Old Version
LaneletBehaviour* generate_Jinning_Obstacles_BehaviourModel(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr, BehaviourType type, std::vector<int>& startLaneletIds, std::vector<int>& endLaneletIds, int Obs_id) {
    int i = 0;
    int startLaneletId = 0;
    if (Obs_id==1){
        startLaneletId = startLaneletIds[1];
    }
    else
    {
        startLaneletId = startLaneletIds[0];
    }
    
    i = 0;
    int endLaneletId = endLaneletIds[i];
    // std::cout << "New car generated " << std::endl;
    LaneletBehaviour* behaviour = new LaneletBehaviour(type, mapPtr, rgPtr);
    ConstLanelet fromLanelet = mapPtr->laneletLayer.get(startLaneletId);
    ConstLanelet toLanelet = mapPtr->laneletLayer.get(endLaneletId);
    behaviour->setRoutingPath(fromLanelet, toLanelet);
    // while (!behaviour->setRoutingPath(fromLanelet, toLanelet)) {
    //     fromLanelet = mapPtr->laneletLayer.get(startLaneletIds[random()%startLaneletIds.size()]);
    //     toLanelet = mapPtr->laneletLayer.get(endLaneletIds[random()%endLaneletIds.size()]);
    // }
    behaviour->setCurrentLanelet(fromLanelet);
    behaviour->generateRoutingReferencePath();
    return behaviour;
}
*/
/////////////////////////////////////////////////

FSM* generateFSM(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr, BehaviourType type, std::vector<int>& startLaneletIds, std::vector<int>& endLaneletIds) {
    int id = random();
    int i = random()%startLaneletIds.size();
    int startLaneletId = startLaneletIds[i];
    i = random()%endLaneletIds.size();
    int endLaneletId = endLaneletIds[i];
    // std::cout << "New car generated " << std::endl;
    FSM* fsm = new FSM(type, mapPtr, rgPtr);
    ConstLanelet fromLanelet = mapPtr->laneletLayer.get(startLaneletId);
    ConstLanelet toLanelet = mapPtr->laneletLayer.get(endLaneletId);
    while (!fsm->setRoutingPath(fromLanelet, toLanelet)) {
        fromLanelet = mapPtr->laneletLayer.get(startLaneletIds[random()%startLaneletIds.size()]);
        toLanelet = mapPtr->laneletLayer.get(endLaneletIds[random()%endLaneletIds.size()]);
    }
    fsm->setCurrentLanelet(fromLanelet);
    fsm->generateRoutingReferencePath();
    return fsm;
}

MapInfo* generatemapinfo(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr, std::vector<int>& startLaneletIds, std::vector<int>& endLaneletIds) {
    int id = random();
    int i = random()%startLaneletIds.size();
    int startLaneletId = startLaneletIds[i];
    i = random()%endLaneletIds.size();
    int endLaneletId = endLaneletIds[i];
    // std::cout << "New car generated " << std::endl;
    MapInfo* mapinfo = new MapInfo(mapPtr, rgPtr);
    ConstLanelet fromLanelet = mapPtr->laneletLayer.get(startLaneletId);
    ConstLanelet toLanelet = mapPtr->laneletLayer.get(endLaneletId);
    while (!mapinfo->setRoutingPath(fromLanelet, toLanelet)) {
        fromLanelet = mapPtr->laneletLayer.get(startLaneletIds[random()%startLaneletIds.size()]);
        toLanelet = mapPtr->laneletLayer.get(endLaneletIds[random()%endLaneletIds.size()]);
    }
    return mapinfo;
}

}; // end of namespace
