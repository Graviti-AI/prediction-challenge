#include "LaneletMapReader.hpp"

LaneletMapReader::LaneletMapReader(std::string MapPath, double origin_x,double origin_y){
    map = load(MapPath, projection::UtmProjector(Origin({origin_x, origin_y})));
    traffic_rules::TrafficRulesPtr trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    routingGraph  = routing::RoutingGraph::build(*map, *trafficRules);

    for (auto lanelet : map->laneletLayer) {
        if (routingGraph->following(lanelet).size()==0) {
            EndLaneletIds.push_back(lanelet.id());
        }
        if (routingGraph->previous(lanelet).size()==0) {
            StartLaneletIds.push_back(lanelet.id());
        }
    }

    for (auto lanelet : map->laneletLayer){
        Object2d obj;
        obj.pose.translation() = BasicPoint2d{0, 0};
        obj.pose.linear() = Eigen::Rotation2D<double>(0).matrix();
        auto centerline_= lanelet.centerline2d();
        BasicPoint2d p1 = centerline_[1];
        BasicPoint2d p2 = centerline_[centerline_.size()-2];
        obj.absoluteHull = matching::Hull2d{p1,p2};
        std::vector<LaneletMatch> Match_result = getDeterministicMatches(*map,obj,0.0);
        //std::cout<<"matching: "<<lanelet.id()<<std::endl;
        //std::cout<<"secuess: "<<routingGraph->following(lanelet)<<std::endl;
        //std::cout<<"previous: "<<routingGraph->previous(lanelet)<<std::endl;
        std::vector<ConstLanelet> conflictset;
        for (auto one_match: Match_result){
            if (one_match.lanelet.inverted()) continue;
            //std::cout<<one_match.lanelet<<" dis: "<<one_match.distance<<std::endl;
            conflictset.push_back(ConstLanelet(one_match.lanelet));
            

        }
        auto one_map = std::pair<int, std::vector<ConstLanelet>>(lanelet.id(), conflictset);
        ConflictLane_.insert(one_map);
    }

}