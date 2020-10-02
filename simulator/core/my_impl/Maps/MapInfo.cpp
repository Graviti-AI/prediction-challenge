//
// Created by Fan on 2018/9/26.
//

#include "MapInfo.hpp"
#include "../Agents/Agent.hpp"
/// Constructor
/// \param mapPtr the pointer of map which is shared by simulator
/// \param rgPtr the pointer of routing map which contains navigation info.
MapInfo::MapInfo(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr) {
    routingGraphPtr_ = rgPtr;
    mapPtr_ = mapPtr;
    s_ = 0;
}

/// Set the navigation path
/// \param startLanelet the lanelet where ego vehicle starts
/// \param destinationLanelet the lanelet where ego vehicle ends
bool MapInfo::setRoutingPath(ConstLanelet& startLanelet, ConstLanelet& destinationLanelet) {
    startLanelet_ = startLanelet;
    destinationLanelet_ = destinationLanelet;
    Optional<routing::Route> route = routingGraphPtr_->getRoute(startLanelet, destinationLanelet, 0);
    if(!route) return false;
    shortestPath_ = route->shortestPath();
    printf("Mapinfo: ShortestPath Length: %d\n", int(shortestPath_.size()));
    //RoutingLineChangePair_.swap(std::vector<std::pair<ConstLanelet, ConstLanelet>>());
    //printf("## ShortestPath Length: %d\n", int(shortestPath_.size()));

    RoutingLineChangePair_.clear();
    for (ConstLanelets::iterator iter = shortestPath_.begin(); iter != shortestPath_.end(); iter++) {
        printf("lanelet_id: %d\n", int(iter->id()));

        Optional<ConstLanelet> leftLanelet = routingGraphPtr_->left(*iter);
        Optional<ConstLanelet> rithgLanelet = routingGraphPtr_->right(*iter);

        auto nextLanelet = std::next(iter, 1);
        if ((nextLanelet!=shortestPath_.end()) && (leftLanelet && leftLanelet->id() == nextLanelet->id() || rithgLanelet && rithgLanelet->id() == nextLanelet->id())) {
            std::pair<ConstLanelet, ConstLanelet> onechange;
            onechange.first = *iter;
            onechange.second = *nextLanelet;
            RoutingLineChangePair_.push_back(onechange);
        }
    }
    if (RoutingLineChangePair_.size()>0 && RoutingLineChangePair_[0].first.id() == startLanelet_.id() )
    {
        RoutingLineChange_ = true;
        CurrentRoutingLineChangePair_ = RoutingLineChangePair_[0];
    }
    setCurrentLanelet(startLanelet_);
    return true;
}


void MapInfo::setLaneletPath(ConstLanelets& lanelet_path){
    assert (lanelet_path.size() > 0);

    startLanelet_ = lanelet_path.front();
    destinationLanelet_ = lanelet_path.back();

    shortestPath_ = routing::LaneletPath(lanelet_path);
    printf("Mapinfo: ShortestPath Length: %d\n", int(shortestPath_.size()));

    RoutingLineChangePair_.clear();
    for (ConstLanelets::iterator iter = shortestPath_.begin(); iter != shortestPath_.end(); iter++) {
        printf("lanelet_id: %d\n", int(iter->id()));
        
        auto nextLanelet = std::next(iter, 1);
        if (nextLanelet!=shortestPath_.end()) {
            std::pair<ConstLanelet, ConstLanelet> onechange;
            onechange.first = *iter;
            onechange.second = *nextLanelet;
            RoutingLineChangePair_.push_back(onechange);
        }
    }
    if (RoutingLineChangePair_.size()>0 && RoutingLineChangePair_[0].first.id() == startLanelet_.id() )
    {
        RoutingLineChange_ = true;
        CurrentRoutingLineChangePair_ = RoutingLineChangePair_[0];
    }
    setCurrentLanelet(startLanelet_);
}


/// init ego car info
/// \param id id of ego car
/// \param initstate init state of ego car
void MapInfo::init(int id, Vector initstate){
    self_id_ = id;
    State = initstate;
    update(initstate);
}

/// reset routing graph
/// \param rgPtr the new routing graph where ego vehicle is
void MapInfo::setRoutingGraph(routing::RoutingGraphPtr& rgPtr) {
    routingGraphPtr_ = rgPtr;
}

/// current lanelet setter
/// \param ll the  current lanelet where ego vehicle is
void MapInfo::setCurrentLanelet(ConstLanelet& ll) {
        currentLanelet_ = ll;
}

/// next lanelet getter
/// \return next lanelet in routing.
ConstLanelet MapInfo::getNextLanelet(){
    for (ConstLanelets::iterator iter = shortestPath_.begin(); iter != shortestPath_.end(); iter++) {
        if (currentLanelet_.id() == iter->id() && iter != (shortestPath_.end()-1)) {
            return mapPtr_->laneletLayer.get((++iter)->id());
        }
    }
    return shortestPath_.back();
}

/// find next lanelet of input
/// \param ll the  current lanelet where ego vehicle is
/// \return next lanelet in routing.
ConstLanelet MapInfo::findNextLanelet(ConstLanelet& ll){
    for (ConstLanelets::iterator iter = shortestPath_.begin(); iter != shortestPath_.end(); iter++) {
        if (ll.id() == iter->id() && iter != (shortestPath_.end()-1)) {
            return mapPtr_->laneletLayer.get((++iter)->id());
        }
    }
    return shortestPath_.back();
}

/// This function is supposed to move vehicle to the next lanelet. As vehicle may go to the following lanelet or change lane, the next lanelet is not determined.
/// \return if the vehicle reaches the end, we should return true to simulator. Then simulator will destroy it.
bool MapInfo::moveToLanelet(ConstLanelet& toLabelet) {
    if (toLabelet.id() == currentLanelet_.id()) {
        return false; // Destroy this car
    }
    if (s_ > geometry::length2d(currentLanelet_))
        s_ -= geometry::length2d(currentLanelet_);
    setCurrentLanelet(toLabelet);
    return true;
}

/// check wheather the input lanelet is in lanechange pair
bool MapInfo::AsRoutingLineChangefirst(ConstLanelet& ll) {
    for( auto &onechange : RoutingLineChangePair_)
    {
        if (onechange.first.id()== ll.id()){
            return true;
        }
    } 
    return false;
}


/// Get the closest precedent vehicle in current routing
/// \param agents all agents
/// \param maxL (optional )the max distance you want to search 
Agent* MapInfo::findClosestPrecByLane(std::vector<Agent*> agents, double maxL) {
    float min_d = std::numeric_limits<float>::max();
    BasicPoint2d egoPos(State[0], State[1]);
    ConstLanelet currLanelet = currentLanelet_;
    double length = 0;
    // double egoS = geometry::toArcCoordinates(wholePath.centerline2d(), egoPos).length;
    Agent* result=nullptr;
    int index = -1;
    bool routinglinechange = RoutingLineChange_;
    std::pair<ConstLanelet, ConstLanelet> currentroutinglinechangepair = CurrentRoutingLineChangePair_;
    while(length < maxL) {
        // Agent* agent = mscUtils::findAgentInLanelet(currLanelet, agents);
        for (auto &agent : agents) {
            if (agent->getState().empty() || agent->getId() == self_id_) continue;
            lanelet::BasicPoint2d agentPos(agent->getState()[0], agent->getState()[1]);

            if (routinglinechange){
                if((!geometry::inside(currentroutinglinechangepair.first, agentPos)) && (!geometry::inside(currentroutinglinechangepair.second, agentPos))) continue;
                BasicPoint2d diff = agentPos - egoPos;
                double dis = sqrt(diff.transpose() * diff);
                double dOther = atan2(diff.y(), diff.x());
                BasicPoint2d egoFuturePoint = geometry::interpolatedPointAtDistance(currentLanelet_.centerline2d(), s_ + 0.5);
                BasicPoint2d diffEgo = egoFuturePoint - egoPos;
                double dEgo = atan2(diffEgo.y(), diffEgo.x());
                if (currentLanelet_.id() == currLanelet.id()) {
                    double egoS = geometry::toArcCoordinates(currentLanelet_.centerline2d(), egoPos).length;
                    double otherS = geometry::toArcCoordinates(currentLanelet_.centerline2d(), agentPos).length;
                    dis = otherS - egoS;
                    if (dis > 0.05 && dis < min_d) { // TODO
                        min_d = dis;
                        result = agent;
                    }
                }       
                else if (fmod(abs(dOther - dEgo), 3.1415) < 3.1415/2.0 && dis > 0.1 && dis < min_d) {
                    min_d = dis;
                    result = agent;
                }
                continue;
            }

            if(!geometry::inside(currLanelet, agentPos)) continue;
            // double otherS  = geometry::toArcCoordinates(wholePath.centerline2d(), agentPos).length;
            BasicPoint2d diff = agentPos - egoPos;
            double dis = sqrt(diff.transpose() * diff);
            double dOther = atan2(diff.y(), diff.x());
            if (dOther<0) dOther += 2*3.1415926535;
            BasicPoint2d egoFuturePoint = geometry::interpolatedPointAtDistance(currentLanelet_.centerline2d(), s_ + 0.5);
            BasicPoint2d diffEgo = egoFuturePoint - egoPos;
            double dEgo = atan2(diffEgo.y(), diffEgo.x());
            if (dEgo<0) dEgo += 2*3.1415926535;
            if (currentLanelet_.id() == currLanelet.id()) {
                double egoS = geometry::toArcCoordinates(currentLanelet_.centerline2d(), egoPos).length;
                double otherS = geometry::toArcCoordinates(currentLanelet_.centerline2d(), agentPos).length;
                dis = otherS - egoS;
                if (dis > 0.05 && dis < min_d) { // TODO
                    min_d = dis;
                    result = agent;
                }
            }
            else if (abs(dOther - dEgo)< 3.1415/2.0 && dis > 0.1 && dis < min_d) {
            // if (((otherS > egoS) || (fmod(abs(dOther - dEgo), 3.1415) < 3.1415/2.0 && dis > 0.1)) && dis < min_d) {
                min_d = dis;
                result = agent;
                // return agent->getState();
            }
            // if ((otherS > egoS) && (egoS > otherS -10)) return agent->getState();
        }
        if(result!=nullptr) return result;

        if (routinglinechange){
            length += geometry::length2d(currentroutinglinechangepair.second);
            ConstLanelet followingLanelet = findNextLanelet(currentroutinglinechangepair.second);
            if (followingLanelet.id()==currentroutinglinechangepair.second.id()) break;
            currLanelet = followingLanelet;
            routinglinechange = false;
            continue;
        }

        length += geometry::length2d(currLanelet);
        ConstLanelet followingLanelet = findNextLanelet(currLanelet);
        if (followingLanelet.id()==currLanelet.id()) break;
        for( auto &onechange : RoutingLineChangePair_)
        {
            if (onechange.first.id()== followingLanelet.id()){
                routinglinechange = true;
                currentroutinglinechangepair = onechange;
            }
        } 
        
    }
    return nullptr;
}

/// Get the closest following vehicle in current lanelet and one layer previous lanelet
/// \param agents all agents
Agent* MapInfo::findClosestSuccByLane(std::vector<Agent*> agents) {
    float min_d = std::numeric_limits<float>::max();
    BasicPoint2d egoPos(State[0], State[1]);
    ConstLanelet currLanelet = currentLanelet_;
    ConstLanelets previousLanelets = routingGraphPtr_->previous(currLanelet);
    Agent* result = nullptr;
    for (auto &agent : agents) {
        if (agent->getState().empty() || agent->getId() == self_id_) continue;
        lanelet::BasicPoint2d agentPos(agent->getState()[0], agent->getState()[1]);
        if(!geometry::inside(currLanelet, agentPos)) continue;
        double egoS = geometry::toArcCoordinates(currentLanelet_.centerline2d(), egoPos).length;
        double otherS = geometry::toArcCoordinates(currentLanelet_.centerline2d(), agentPos).length;
        double dis = egoS - otherS;
        if (dis > 0.05 && dis < min_d) { // TODO
            min_d = dis;
            result = agent;
        }
    }
    if(result!=nullptr) return result;
    if (previousLanelets.empty()) return nullptr;

    for (auto &previousLanelet: previousLanelets){
        currLanelet = previousLanelet;
        for (auto &agent : agents) {
            if (agent->getState().empty() || agent->getId() == self_id_) continue;
            lanelet::BasicPoint2d agentPos(agent->getState()[0], agent->getState()[1]);
            if(!geometry::inside(currLanelet, agentPos)) continue;
            BasicPoint2d diff = egoPos - agentPos;
            double dis = sqrt(diff.transpose() * diff);
            double dOther = atan2(diff.y(), diff.x());
            if (dOther<0) dOther += 2*3.1415926535;
            BasicPoint2d egoFuturePoint = geometry::interpolatedPointAtDistance(currentLanelet_.centerline2d(), s_ + 0.5);
            BasicPoint2d diffEgo = egoFuturePoint - egoPos;
            double dEgo = atan2(diffEgo.y(), diffEgo.x());
            if (dEgo<0) dEgo += 2*3.1415926535;
            if (abs(dEgo - dOther) > 3.1415/2.0 && dis > 0.1) {
                min_d = dis;
                result = agent;
            }
        }
    }
    return nullptr;
}

void MapInfo::update(Vector nextstate){
    BasicPoint2d currPos(nextstate[0], nextstate[1]);
    if (RoutingLineChange_){
        if (currentLanelet_.id()== CurrentRoutingLineChangePair_.first.id()){ 
            //std::cout<< "distence of first: "<< geometry::toArcCoordinates(currentLanelet_.centerline2d(), currPos).distance<<std::endl;
            //std::cout<< "distence of second: "<< geometry::toArcCoordinates(CurrentRoutingLineChangePair_.second.centerline2d(), currPos).distance<<std::endl;
            if (abs(geometry::toArcCoordinates(currentLanelet_.centerline2d(), currPos).distance) > 
            abs(geometry::toArcCoordinates(CurrentRoutingLineChangePair_.second.centerline2d(), currPos).distance)){
                ConstLanelet next_lanelet = getNextLanelet();
                setCurrentLanelet(next_lanelet);
                s_ = geometry::toArcCoordinates(next_lanelet.centerline2d(), currPos).length;
                
                /// for multiple change
                for( auto &onechange : RoutingLineChangePair_)
                {
                    if (onechange.first.id()== next_lanelet.id()){
                        RoutingLineChange_ = true;
                        CurrentRoutingLineChangePair_ = onechange;
                    }
                } 
            }
            else{
                s_ = geometry::toArcCoordinates(currentLanelet_.centerline2d(), currPos).length;
            }
        }  
        else {
            s_ = geometry::toArcCoordinates(currentLanelet_.centerline2d(), currPos).length;
            if (s_ - geometry::length2d(currentLanelet_)>=-0.001) {
                ConstLanelet next_lanelet = getNextLanelet();
                if (currentLanelet_.id() == next_lanelet.id()){
                    HasArrivedDestination_ = true;
                    return;
                }
                setCurrentLanelet(next_lanelet);
                s_ = geometry::toArcCoordinates(next_lanelet.centerline2d(), currPos).length;

                RoutingLineChange_ = false;
                marge_first = true;

                /// for continuing change
                for( auto &onechange : RoutingLineChangePair_)
                {
                    if (onechange.first.id()== next_lanelet.id()){
                        RoutingLineChange_ = true;
                        CurrentRoutingLineChangePair_ = onechange;
                    }
                } 
            }
        }  
        return;
    }
   
    s_ = geometry::toArcCoordinates(currentLanelet_.centerline2d(), currPos).length;
    //cout<<"s_ : "<<s_<<" length2d: "<<geometry::length2d(currentLanelet_)<<" "<<s_ - geometry::length2d(currentLanelet_)<<endl;
    if (s_ - geometry::length2d(currentLanelet_)>=-0.001) {
        ConstLanelet next_lanelet = getNextLanelet();
        if (currentLanelet_.id() == next_lanelet.id()){
            std::cout<<"Mapinfo: set HasArrivedDestination_ as true!"<<std::endl;
            HasArrivedDestination_ = true;
            return;
        }
        setCurrentLanelet(next_lanelet);
        s_ = geometry::toArcCoordinates(next_lanelet.centerline2d(), currPos).length;
        for( auto &onechange : RoutingLineChangePair_)
        {
            if (onechange.first.id()== next_lanelet.id()){
                RoutingLineChange_ = true;
                CurrentRoutingLineChangePair_ = onechange;
            }
        } 
    }
    this->State = nextstate;
}