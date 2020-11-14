#include "ConfigGenerator.hpp"
# define PI (3.1415926535)
ConfigGenerator::ConfigGenerator(std::string CSV_file){
    Read(CSV_file);
    Generating();
}

void ConfigGenerator::Read(std::string CSV_file){
    cout<<"Reading..."<<endl;
    Configs.clear();
    ifstream track_file(CSV_file, ios::in);
    string str;
    getline(track_file, str);

    string Map, Trackid, Startframe, EgoID, ReactiveIDs, IDMs, external_predictorIDs,internal_predictorIDs,Endframe,RightofWayIDs;
    while (getline(track_file, Map, ',')) {
        getline(track_file, Trackid, ',');
        getline(track_file, Startframe, ',');
        getline(track_file, EgoID, ',');
        getline(track_file, ReactiveIDs, ',');
        getline(track_file, IDMs, ',');
        getline(track_file, external_predictorIDs, ',');
        getline(track_file, internal_predictorIDs, ',');
        getline(track_file, Endframe, ',');
        getline(track_file, RightofWayIDs, '\n');
        ConfigDetiles onefile;
        /// process MAP
        if(Map =="ZS") onefile.MapName = "DR_CHN_Merging_ZS";
        else if (Map =="LN") onefile.MapName = "DR_CHN_Roundabout_LN";
        else if (Map =="MT") onefile.MapName = "DR_DEU_Merging_MT";
        else if (Map =="OF") onefile.MapName = "DR_DEU_Roundabout_OF";
        else if (Map =="EP0") onefile.MapName = "DR_USA_Intersection_EP0";
        else if (Map =="EP1") onefile.MapName = "DR_USA_Intersection_EP1";
        else if (Map =="GL") onefile.MapName = "DR_USA_Intersection_GL";
        else if (Map =="MA") onefile.MapName = "DR_USA_Intersection_MA";
        else if (Map =="EP") onefile.MapName = "DR_USA_Roundabout_EP";
        else if (Map =="FT") onefile.MapName = "DR_USA_Roundabout_FT";
        else if (Map =="SR") onefile.MapName = "DR_USA_Roundabout_SR";

        /// process Track number
        onefile.TrackNumber = Trackid;
        while (onefile.TrackNumber.size()<3)
        {
            onefile.TrackNumber = '0'+onefile.TrackNumber;
        }

        /// process Start frame
        onefile.Startframe_= 100*stringToNum<int>(Startframe);

        /// process Ego robot car
        RobotCarInitStates NewOne;
        NewOne.id = stringToNum<int>(EgoID);
        NewOne.PlannerName = "Astar";
        onefile.RobotCarInitStates_.push_back(NewOne);
        NewOne.isego = false;

        /// process Reactive robot car
        istringstream ReactiveIDsProcess(ReactiveIDs);
        string OneRCarID;
        NewOne.PlannerName = "ReactivePlanner"; //TODO: where is the reactive planner
        while (getline(ReactiveIDsProcess,OneRCarID,' '))
        {
            NewOne.id = stringToNum<int>(OneRCarID);
            onefile.RobotCarInitStates_.push_back(NewOne);
        }
        

        /// process IDM car
        istringstream IDMsProcess(IDMs);
        string OneIDMsID;
        NewOne.PlannerName = "IDM";
        while (getline(IDMsProcess,OneIDMsID,' '))
        {
            NewOne.id = stringToNum<int>(OneIDMsID);
            onefile.RobotCarInitStates_.push_back(NewOne);
        }
        onefile.RobotCarNum = onefile.RobotCarInitStates_.size();
        /// process internal predictor car
        istringstream InternalPredictorIDsProcess(internal_predictorIDs);
        string OneReplayID;
        while (getline(InternalPredictorIDsProcess,OneReplayID,' ')){
            bool find_flag = false;
            for(int i = 0;i<onefile.RobotCarInitStates_.size();i++){
                if (stringToNum<int>(OneReplayID)==onefile.RobotCarInitStates_[i].id){
                    find_flag = true;
                    break;
                }
            }
            if (!find_flag){
                ReplayCarInitStates NewOne;
                NewOne.id = stringToNum<int>(OneReplayID);
                onefile.ReplayCarInitStates_.push_back(NewOne);
            }
        }

        /// process external predictor car
        istringstream ExternalPredictorIDsProcess(external_predictorIDs);
        while (getline(ExternalPredictorIDsProcess,OneReplayID,' ')){
            bool find_flag = false;
            for(int i = 0;i<onefile.RobotCarInitStates_.size();i++){
                if (stringToNum<int>(OneReplayID)==onefile.RobotCarInitStates_[i].id){
                    onefile.RobotCarInitStates_[i].ex_Predictor = true;
                    find_flag = true;
                    break;
                }
            }
            for(int i = 0;i<onefile.ReplayCarInitStates_.size();i++){
                if (stringToNum<int>(OneReplayID)==onefile.ReplayCarInitStates_[i].id){
                    onefile.ReplayCarInitStates_[i].ex_Predictor = true;
                    find_flag = true;
                    break;
                }
            }
            if (!find_flag){
                ReplayCarInitStates NewOne;
                NewOne.id = stringToNum<int>(OneReplayID);
                NewOne.ex_Predictor = true;
                onefile.ReplayCarInitStates_.push_back(NewOne);
            }
        }
        onefile.ReplayCarNum = onefile.ReplayCarInitStates_.size();

        /// process End frame
        onefile.Endframe_= 100*stringToNum<int>(Endframe);

        /// process RightofWay car
        istringstream RightofWayIDsProcess(RightofWayIDs);
        string RightofWayID;
        while (getline(RightofWayIDsProcess,RightofWayID,' ')){
            onefile.RightofWayIDs_.push_back(stringToNum<int>(RightofWayID));
        }


        Configs.push_back(onefile);
    }
    cout<<"Finish Reading"<<endl;

    RecordRead RecordProcesser(Configs[0].MapName,Configs[0].TrackNumber);
    LaneletMapPtr map;
    routing::RoutingGraphPtr routingGraph;

    for(auto &one_file: Configs){
        cout<<"processing: "<<one_file.MapName<<" Track: "<<one_file.TrackNumber<<endl;
        std::string MapPath = "../maps/"+one_file.MapName+".osm";
        map = load(MapPath, projection::UtmProjector(Origin({0, 0})));
        traffic_rules::TrafficRulesPtr trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
        routingGraph  = routing::RoutingGraph::build(*map, *trafficRules);

        if (one_file.MapName != RecordProcesser.MapName_||one_file.TrackNumber!=RecordProcesser.TrackNumber_){
            RecordProcesser = RecordRead(one_file.MapName,one_file.TrackNumber);
        }

        for(int i = 0;i<one_file.RobotCarInitStates_.size();i++){
            cout<<"processing: "<<one_file.RobotCarInitStates_[i].id<<endl;
            car* this_car;
            for(auto one:RecordProcesser.data_car){
                if (one->id_==one_file.RobotCarInitStates_[i].id){
                    this_car = one;
                    break;
                }
            }
            state adding_states = this_car->getstate(one_file.Startframe_);
            int delaytimes = 0;
            while(adding_states.time_==0){
                delaytimes++;
                adding_states = this_car->getstate(one_file.Startframe_+(delaytimes)*100);
            }

            cout<<"state find! "<<endl;
            one_file.RobotCarInitStates_[i].EnterupdateTimes = delaytimes*10;
            one_file.RobotCarInitStates_[i].x_ = adding_states.x_;
            one_file.RobotCarInitStates_[i].y_ = adding_states.y_;
            one_file.RobotCarInitStates_[i].yaw_ = adding_states.psi_rad_;
            one_file.RobotCarInitStates_[i].vx_ = adding_states.vx_;
            one_file.RobotCarInitStates_[i].vy_ = adding_states.vy_;
            one_file.RobotCarInitStates_[i].length_ = adding_states.length_;
            one_file.RobotCarInitStates_[i].width_ = adding_states.width_;

            cout<<"begin matching"<<endl;

            /*
            Object2d obj;
            obj.pose.translation() = BasicPoint2d{0, 0};
            obj.pose.linear() = Eigen::Rotation2D<double>(0).matrix();
            BasicPoint2d currPos(one_file.RobotCarInitStates_[i].x_ ,one_file.RobotCarInitStates_[i].y_);
            obj.absoluteHull = matching::Hull2d{currPos};
            std::vector<LaneletMatch> Match_result = getDeterministicMatches(*map,obj,0.0);
            double min_angel =  2*PI;
            int lanelet_id = 0;
            for (auto one_match: Match_result){
                if (one_match.lanelet.inverted()) continue;
                double s_ = geometry::toArcCoordinates(one_match.lanelet.centerline2d(), currPos).length;
                BasicPoint2d p_front = geometry::interpolatedPointAtDistance(one_match.lanelet.centerline2d(), s_);
                BasicPoint2d p_back = geometry::interpolatedPointAtDistance(one_match.lanelet.centerline2d(), s_+0.1);
                BasicPoint2d p_diff = p_back-p_front;
                double Lane_direction = atan2(p_diff.y(),p_diff.x());
                double angle_diff = abs(Lane_direction - one_file.RobotCarInitStates_[i].yaw_);
                if (angle_diff>=PI)  angle_diff = 2*PI - angle_diff;
                if (angle_diff<min_angel){
                    min_angel = angle_diff;
                    lanelet_id =one_match.lanelet.id();
                }
            }
            */
           int lanelet_id = HelperFunction::xy2laneid(one_file.RobotCarInitStates_[i].x_ ,one_file.RobotCarInitStates_[i].y_, one_file.RobotCarInitStates_[i].yaw_, map).first;
            one_file.RobotCarInitStates_[i].Start_lanlet_ID = lanelet_id;
            
            state ending_states = this_car->car_state.back();

            /*
            currPos=BasicPoint2d(ending_states.x_ ,ending_states.y_);
            obj.absoluteHull = matching::Hull2d{currPos};
            Match_result = getDeterministicMatches(*map,obj,0.0);
            min_angel =  2*PI;
            lanelet_id = 0;

            for (auto one_match: Match_result){
                if (one_match.lanelet.inverted()) continue;
                double s_ = geometry::toArcCoordinates(one_match.lanelet.centerline2d(), currPos).length;
                BasicPoint2d p_front = geometry::interpolatedPointAtDistance(one_match.lanelet.centerline2d(), s_);
                BasicPoint2d p_back = geometry::interpolatedPointAtDistance(one_match.lanelet.centerline2d(), s_+0.1);
                BasicPoint2d p_diff = p_back-p_front;
                double Lane_direction = atan2(p_diff.y(),p_diff.x());
                double angle_diff = abs(Lane_direction - one_file.RobotCarInitStates_[i].yaw_);
                if (angle_diff>=PI)  angle_diff = 2*PI - angle_diff;
                if (angle_diff<min_angel){
                    min_angel = angle_diff;
                    lanelet_id =one_match.lanelet.id();
                }
            }
            */

           lanelet_id = HelperFunction::xy2laneid(ending_states.x_ ,ending_states.y_, ending_states.psi_rad_, map).first;
            one_file.RobotCarInitStates_[i].End_lanlet_ID = lanelet_id;

            /*
            ConstLanelet fromLanelet = map->laneletLayer.get(one_file.RobotCarInitStates_[i].Start_lanlet_ID);
            ConstLanelet toLanelet = map->laneletLayer.get(one_file.RobotCarInitStates_[i].End_lanlet_ID);

            Optional<routing::Route> route = routingGraph->getRoute(fromLanelet, toLanelet, 0);
            if(!route) assert(false);
            */
            
            if (one_file.RobotCarInitStates_[i].isego){
                state endframe_states =this_car->getstate(one_file.Endframe_);
                one_file.EgoEndPositionX_ = endframe_states.x_;
                one_file.EgoEndPositionY_ = endframe_states.y_;
            }
                    
        }
    }

    cout<<"Finish processing"<<endl;



}

void ConfigGenerator::Generating(){
    int i =-1;
    cout<<"Generating..."<<endl;
    for(auto &one_file: Configs){
        i++;
        cout<<"process: "<<i<<endl;
        char write_file_name[100];
        sprintf(write_file_name,"../output/test_config_%s.txt",to_string(i).c_str());
        ofstream File_creat(write_file_name);
        File_creat.close();
        std::ofstream out;
        out.open(write_file_name,std::ios::app);
        if(out.is_open()) {
            out << "Map:"<<one_file.MapName<<endl;
            out << "Track:"<<one_file.TrackNumber<<endl;
            out <<"ReplayStartTimestamp(ms):"<<to_string(one_file.Startframe_)<<endl;
            out <<"MaxUpdateTimes:"<<to_string((one_file.Endframe_ - one_file.Startframe_) / 10)<<endl;
            out << "RobotCarNum:"<<to_string(one_file.RobotCarNum)<<endl;
            out << "InitState:track_id,Planner,Planner.Para,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon,ego_car"<<endl;
            for(int i=0;i<one_file.RobotCarInitStates_.size();i++){
                auto initstate = one_file.RobotCarInitStates_[i];

                if (initstate.Start_lanlet_ID == 0 || initstate.End_lanlet_ID == 0){
                    printf("orz\n");
                }

                out <<to_string(initstate.id) <<" ";
                //out <<to_string(initstate.EnterupdateTimes) <<" "<<to_string(initstate.x_)<<" "<<to_string(initstate.y_)<<" ";
                //out <<to_string(initstate.yaw_)<<" "<<to_string(sqrt(initstate.vx_*initstate.vx_+initstate.vy_*initstate.vy_))<<" ";
                //out <<"0.00"<<" "<<"0.00"<<" "<<to_string(initstate.length_)<<" "<<to_string(initstate.width_)<<" ";
                //out <<to_string(initstate.Start_lanlet_ID)<<" "<<to_string(initstate.End_lanlet_ID)<<" ";
                out <<initstate.PlannerName<<" "<<initstate.PlannerPara<<" "<<initstate.PridictorName<<" ";
                //out <<to_string(initstate.PredictorDt)<<" "<<to_string(initstate.PredictorHorizon)<<" ";
                out <<"0.1"<<" "<<"3.0"<<" ";
                if (initstate.ex_Predictor){
                    out <<"yes "<<"0.1"<<" "<<"3.0"<<" ";
                }
                else
                {
                    out <<"no "<<"0.1"<<" "<<"3.0"<<" ";
                }
                if (initstate.isego) out <<"yes"<<endl;
                else out <<"no"<<endl;
            } 
            out <<"ReplayCarWithPredictor:"<<to_string(one_file.ReplayCarNum)<<endl;
            out <<"InitState:track_id,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon"<<endl;
            for(auto one: one_file.ReplayCarInitStates_){
                out <<to_string(one.id)<<" "<<one.PridictorName<<" "<<"0.1"<<" "<<"3.0"<<" ";
                if (one.ex_Predictor){
                    out <<"yes "<<"0.1"<<" "<<"3.0"<<endl;
                }
                else
                {
                    out <<"no "<<"0.1"<<" "<<"3.0"<<endl;
                }
                
            }
            out <<"EndframeTimestamp(ms):"<<to_string(one_file.Endframe_)<<endl;
            out <<"EgoEndPosition:"<<to_string(one_file.EgoEndPositionX_)<<" "<<to_string(one_file.EgoEndPositionY_)<<endl;
            out <<"TargetRightofWay:";
            for(auto one: one_file.RightofWayIDs_){
                out<<to_string(one)<<" ";
            }
            out <<endl;
            out.close();
        }
        else
        {
            cout << "no such file" << endl;
        }
    }

}


std::pair<int, std::string> HelperFunction::xy2laneid(double x, double y, double yaw, lanelet::LaneletMapPtr map_ptr){
    
    // calculate s, d, lane
    Object2d obj;
    BasicPoint2d pp(x, y);
    obj.pose.translation() = BasicPoint2d{0, 0};
    obj.pose.linear() = Eigen::Rotation2D<double>(0).matrix();
    obj.absoluteHull = matching::Hull2d{pp};
    
    const double MIN_ALLOWED_DIS = 0.0;
    //const double PI = 3.1415926535;
    std::vector<LaneletMatch> Match_result = getDeterministicMatches(*map_ptr, obj, MIN_ALLOWED_DIS);
    //Find all the lanelets whose distance to pp is smaller than MIN_ALLOWED_DIS

    if (Match_result.size() == 0){
        return std::make_pair(0, "closest_lane");

        int min_dis_lanelet_id = -1;
        double min_dis = 1e10;

        //printf("# DEBUG | x: %.3lf, y: %.3lf, yaw: %.3lf, match size = 0\n", x, y, yaw);

        for (auto tmplanelet : map_ptr->laneletLayer){
            ConstLanelet lanelet = map_ptr->laneletLayer.get(tmplanelet.id());
            auto centerline = lanelet.centerline2d();
            double dis = abs(geometry::toArcCoordinates(centerline, BasicPoint2d(x, y)).distance);

            //printf("# DEBUG | lane_id: %d, dis: %.3lf\n", int(lanelet.id()), dis);

            if (min_dis_lanelet_id == -1 || dis < min_dis){
                min_dis_lanelet_id = tmplanelet.id();
                min_dis = dis;
            }
        }
        //printf("# DEBUG | min_lane_id: %d\n\n", min_dis_lanelet_id);
        //exit(0);

        assert(min_dis_lanelet_id != -1);
        return std::make_pair(min_dis_lanelet_id, "closest_lane");
    }
    else {
        int min_yaw_gap_lanelet_id = -1;
        double min_yaw_gap = 1e10;

        //printf("# DEBUG | x: %.3lf, y: %.3lf, yaw: %.3lf, match size > 0\n", x, y, yaw);

        // Find the lanelets whose direction is the closest to the yaw_angle 
        for (auto one_match: Match_result){
            assert(one_match.distance <= MIN_ALLOWED_DIS);
            if (one_match.lanelet.inverted()) continue;

            ConstLanelet lanelet = map_ptr->laneletLayer.get(one_match.lanelet.id());
            auto centerline = lanelet.centerline2d();

            double s_now = geometry::toArcCoordinates(centerline, BasicPoint2d(x, y)).length;
            BasicPoint2d pinit = geometry::interpolatedPointAtDistance(centerline, s_now);
            BasicPoint2d pinit_f = geometry::interpolatedPointAtDistance(centerline, s_now + 0.1);
            BasicPoint2d pDirection = pinit_f - pinit;
            double direction = std::atan2(pDirection.y(),pDirection.x());
            double angle_diff = abs(direction - yaw);
            if (angle_diff >= PI)  angle_diff = 2*PI - angle_diff;

            //printf("# DEBUG | lane_id: %d, angle_diff: %.3lf\n", int(lanelet.id()), angle_diff);

            if (min_yaw_gap_lanelet_id == -1 || angle_diff < min_yaw_gap){
                min_yaw_gap = angle_diff;
                min_yaw_gap_lanelet_id = one_match.lanelet.id();
            }
        }
        //printf("# DEBUG | min_lane_id: %d\n\n", min_yaw_gap_lanelet_id);

        assert(min_yaw_gap_lanelet_id != -1);
        return std::make_pair(min_yaw_gap_lanelet_id, "matches");
    }
    assert(false);
}
