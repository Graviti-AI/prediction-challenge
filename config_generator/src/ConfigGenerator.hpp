//
// Created by lcr on 9/23/20.
//

#ifndef CONFIGGENERATOR_H
#define CONFIGGENERATOR_H

#include "RecordRead.hpp"
#include <cmath>
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
using namespace std;
using namespace lanelet;
using namespace lanelet::matching;
struct RobotCarInitStates{
    int id = 0;
    int EnterupdateTimes = 0;
    double x_ = 0;
    double y_ = 0;
    double yaw_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double length_ = 0;
    double width_ = 0;
    int Start_lanlet_ID = 10001;
    int End_lanlet_ID = 10002;
    std::string PlannerName;
    std::string PlannerPara = "N";
    std::string PridictorName = "GroundTruth";
    double PredictorDt = 0.1;
    double PredictorHorizon = 3; //TODO: the future horizon in the challenge is 3 not 5
    bool isego = true;
    bool ex_Predictor = false;
};

struct ReplayCarInitStates
{
    int id = 0;
    std::string PridictorName = "GroundTruth";
    double PredictorDt = 0.1;
    double PredictorHorizon = 3;  //TODO: the future horizon in the challenge is 3 not 5
    bool ex_Predictor = false;
};


struct ConfigDetiles{
    std::string MapName;
    std::string TrackNumber = "000";
    int RobotCarNum = 0;
    std::vector<RobotCarInitStates> RobotCarInitStates_;
    int MaxUpdateTime = 5000;//TODO: 30000 is too large
    int Startframe_;
    int Endframe_;
    int ReplayCarNum = 0;
    double EgoEndPositionX_;
    double EgoEndPositionY_;
    std::vector<ReplayCarInitStates> ReplayCarInitStates_;
    std::vector<int> RightofWayIDs_;
};

class ConfigGenerator {

public:
    ConfigGenerator(std::string CSV_file);
    void Read(std::string CSV_file);
    void Generating();
    std::vector<ConfigDetiles> Configs;

    template <class Type>
    Type stringToNum(const std::string& str)
    {
        istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }

};

namespace HelperFunction{
    std::pair<int, std::string> xy2laneid(double x, double y, double yaw, lanelet::LaneletMapPtr map_ptr);
}


#endif //CONFIGGENERATOR_H