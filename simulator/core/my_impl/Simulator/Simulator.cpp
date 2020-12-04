//
// Created by mscsim on 8/23/18.
//
#include <zconf.h>
#include <assert.h>
#include "Simulator.hpp"

#include "../Controllers/Controller.hpp"
#include "../Models/Model.hpp"
#include "../Planners/Planner.hpp"
#include "../Planners/VirtualCarPlanner.hpp"
#include "../Controllers/VirtualCarController.hpp"
#include "../Behaviours/LaneletBehaviour.hpp"
#include "../Models/VirtualCarModel.hpp"
#include "../Behaviours/CarGenerator.hpp"
#include "../Planners/AstarPlanner.hpp"
#include "../Planners/CILQRPlanner.hpp"
#include "../Planners/EBPlanner.hpp"
//#include "../Planners/GameTreePlanner.hpp"
#include "../Behaviours/FSM.hpp"
#include "../Behaviours/AoBehaviour.hpp"
#include "../Predictors/Predictor.hpp"

#include <stdio.h>      /* printf */
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string.h>
#include <iostream>
#include <algorithm>


char write_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";
char collision_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";
std::ofstream out;


#define Car_Num 10

namespace {
    //std::string exampleMapPath = "/home/mscsim/ao/framework/Maps/test.osm";
    //std::string exampleMapPath = "/home/mscsim/ao/framework/Maps/with_negative_xy/DR_USA_Roundabout_SR.osm";

    std::string examplemap = "DR_USA_Intersection_MA";
    std::string exampleMapPath = "../core/my_impl/Maps/with_negative_xy/"+examplemap+".osm";

    // DR_CHN_Merging_ZS.osm    DR_CHN_Roundabout_LN.osm    DR_DEU_Merging_MT.osm   DR_DEU_Roundabout_OF.osm
    // DR_USA_Intersection_EP0.osm  DR_USA_Intersection_EP1.osm DR_USA_Intersection_GL.osm  DR_USA_Intersection_MA.osm
    // DR_USA_Roundabout_EP.osm DR_USA_Roundabout_FT.osm    DR_USA_Roundabout_SR.osm
}

using namespace std;
using namespace lanelet;
using namespace lanelet::matching;
using namespace collision_or_not;

///
/// \param simulatorState reference to the simulator state, an enumerator.
/// \param humanInputs reference to a map from agent to pertaining vector human input.
/// \param agentDictionary reference to a map from agent to pertaining controller, model, and planner.
/// \param mutex reference to a global mutex lock, which avoids thread conflicts.
Simulator::Simulator(int rviz_port):myThreadPool(Car_Num){ //, total_car_num(1){
    simulatorState = Running; // will be shared by simulator thread and server thread
    MapName_ = examplemap;

    //generateVirtualCar();
    //ReplayGeneratorPtr->loadCSV("/home/sunyaofeng/Desktop/CSV/DR_USA_Intersection_MA/vehicle_tracks_001.csv");
    //cout<<"EndLaneletIds: "<<mapreader->EndLaneletIds.size()<<" StartLaneletIds: "<<mapreader->StartLaneletIds.size()<<endl;
    //CILQR_car_flag = false;//true;

    //rviz viualization
    if (rviz_port > 0){
        myClientPool = new MyClientPool(Car_Num, mutex, agentDictionary);
        server = new Server(rviz_port, simulatorState, humanInputs, agentDictionary, pAgentDictionary, mutex);
        printf("# Rviz port: %d ......\n", rviz_port);
    }
}

void Simulator::InitSimulation(std::string scenario_id, std::string Config_Path, std::string log_folder, const bool verbose){
    verbose_ = verbose;
    // if verbose_ is ture, then output log in the console
    
    std::ifstream Config_ifstream;
    Config_Path_ = Config_Path;
    Config_ifstream.open(Config_Path);

    if (! Config_ifstream.good()){
        throw std::runtime_error("Error: The congfig file is not exsited!");
    }
    std::string temp;

    // read map
    getline(Config_ifstream, temp, ':');
    assert(temp == "Map");

    getline(Config_ifstream, temp, '\n');
    MapName_ = temp;

    std::string MapPath_="../core/my_impl/Maps/with_negative_xy/" + MapName_ + ".osm";
    trafficInfoManagerPtr = new TrafficInfoManager(MapPath_);
    mapreader = new LaneletMapReader(MapPath_,0.0,0.0);
    if (verbose_) printf("# MapPath_: %s, StartLaneletIds: %d, EndLaneletIds: %d\n", MapPath_.c_str(), int(mapreader->StartLaneletIds.size()), int(mapreader->EndLaneletIds.size()));

    // read Track
    getline(Config_ifstream, temp, ':');
    assert(temp == "Track");

    getline(Config_ifstream, temp, '\n');
    std::string TrackPath_="../core/my_impl/CSV/" + MapName_ + "/vehicle_tracks_" + temp + ".csv";
    ReplayGeneratorPtr->loadCSV(TrackPath_);
    if (verbose_) printf("# TrackPath: %s\n", TrackPath_.c_str());

    // ReplayStartTimestamp(ms)
    getline(Config_ifstream, temp, ':');
    assert(temp == "ReplayStartTimestamp(ms)");

    getline(Config_ifstream, temp, '\n');
    int ReplayStartTimestamp_ms = stringToNum<int>(temp);
    if (verbose_) printf("# ReplayStartTimestamp_ms: %d\n", ReplayStartTimestamp_ms);

    // MaxUpdateTimes
    getline(Config_ifstream, temp, ':');
    assert(temp == "MaxUpdateTimes");

    getline(Config_ifstream, temp, '\n');
    MaxUpdateTimes_=stringToNum<int >(temp);
    if (verbose_) printf("# MaxUpdateTimes: %d\n", MaxUpdateTimes_);

    // filter all the replay cars.
    int ReplayEndTimestamp_ms = MaxUpdateTimes_ * 10 + ReplayStartTimestamp_ms; // each tick in the simulator is 0.01s (10ms)
    auto replay_info_pool = ReplayGeneratorPtr->filter_replay_car(ReplayStartTimestamp_ms, ReplayEndTimestamp_ms);
    if (verbose_) printf("# ReplayStartTimestamp_ms: %d, ReplayEndTimestamp_ms: %d, replay_info_pool_size: %d\n", ReplayStartTimestamp_ms, ReplayEndTimestamp_ms, int(replay_info_pool.size()));

    // read robot cars
    getline(Config_ifstream, temp, ':');
    assert(temp == "RobotCarNum");

    getline(Config_ifstream, temp, '\n');
    int RobotCarNum = stringToNum<int >(temp);
    if (verbose_) printf("# RobotCarNum: %d\n", RobotCarNum);

    getline(Config_ifstream, temp, '\n');
    assert(temp == "InitState:track_id,Planner,Planner.Para,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon,ego_car");

    for (int i = 0; i < RobotCarNum; i ++){
        getline(Config_ifstream, temp, ' ');
        int track_id = stringToNum<int >(temp);

        getline(Config_ifstream, temp, '\n');

        std::vector<int> info;
        for (int i = 0; i < replay_info_pool.size(); i ++)
            if (replay_info_pool[i][0] == track_id) {
                info = replay_info_pool[i];
                replay_info_pool.erase(replay_info_pool.begin() + i);
                break;
            }
        
        if (info.size() != 3){
            printf("ERROR: cannot find robot car (%d) in replay list\n", track_id);
            exit(-1);
        }
        if (verbose_) printf("track_id (%d) is a robot car, remove from the replay list\n", track_id);
        
        ReplayCarInfo behave_info = std::make_tuple(info[0], info[1], info[2], temp);
        //this replay car will appear at (info[1] - ReplayStartTimestamp_ms) / 10

        assert((info[1] - ReplayStartTimestamp_ms) % 10 == 0);
        int init_update = (info[1] - ReplayStartTimestamp_ms) / 10;

        BehaveCarWaitList[init_update].push_back(behave_info);
        if (verbose_) printf("Add Behave Car (%d, %d, %d, %s) to WaitList[%d]\n", std::get<0>(behave_info), std::get<1>(behave_info), std::get<2>(behave_info), std::get<3>(behave_info).c_str(), init_update);
    }
    if (verbose_) printf("# After removing robot car, replay_info_pool_size: %d\n", int(replay_info_pool.size()));

    // read replay car
    getline(Config_ifstream, temp, ':');
    assert(temp == "ReplayCarWithPredictor");

    getline(Config_ifstream, temp, '\n');
    int ReplayCarWithPredictor = stringToNum<int>(temp);
    if (verbose_) printf("ReplayCarWithPredictor: %d\n", ReplayCarWithPredictor);

    assert(RobotCarNum + ReplayCarWithPredictor <= Car_Num);

    getline(Config_ifstream, temp, '\n');
    assert(temp == "InitState:track_id,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon");
    
    for (int i = 0; i < ReplayCarWithPredictor; i ++){
        getline(Config_ifstream, temp, ' ');
        int track_id = stringToNum<int >(temp);

        getline(Config_ifstream, temp, '\n');
        
        std::vector<int> info;
        for (int i = 0; i < replay_info_pool.size(); i ++)
            if (replay_info_pool[i][0] == track_id) {
                info = replay_info_pool[i];
                replay_info_pool.erase(replay_info_pool.begin() + i);
                break;
            }
        
        if (info.size() != 3){
            printf("ERROR: cannot find replay car (%d) in replay list\n", track_id);
            exit(-1);
        }

        ReplayCarInfo replay_info = std::make_tuple(info[0], info[1], info[2], temp);
        //this replay car will appear at (info[1] - ReplayStartTimestamp_ms) / 10

        assert((info[1] - ReplayStartTimestamp_ms) % 10 == 0);
        int init_update = (info[1] - ReplayStartTimestamp_ms) / 10;

        ReplayCarWaitList[init_update].push_back(replay_info);
        if (verbose_) printf("Add Replay Car (%d, %d, %d, %s) to WaitList[%d]\n", std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info), std::get<3>(replay_info).c_str(), init_update);
    }

    // add the remaining replay cars to wait list
    for (int i = 0; i < replay_info_pool.size(); i ++){
        auto info = replay_info_pool[i];
        ReplayCarInfo replay_info = std::make_tuple(info[0], info[1], info[2], "");
        
        assert((info[1] - ReplayStartTimestamp_ms) % 10 == 0);
        int init_update = (info[1] - ReplayStartTimestamp_ms) / 10;

        ReplayCarWaitList[init_update].push_back(replay_info);
        if (verbose_) printf("Add Replay Car (%d, %d, %d, %s) to WaitList[%d]\n", std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info), std::get<3>(replay_info).c_str(), init_update);
    }

    // read tails
    getline(Config_ifstream, temp, ':');
    assert(temp == "EndframeTimestamp(ms)");
    getline(Config_ifstream, temp, '\n');
    assert(ReplayEndTimestamp_ms >= stringToNum<int >(temp));

    getline(Config_ifstream, temp, ':');
    assert(temp == "EgoEndPosition");
    getline(Config_ifstream, temp, '\n');

    getline(Config_ifstream, temp, ':');
    assert(temp == "TargetRightofWay");
    getline(Config_ifstream, temp, '\n');

    Config_ifstream.close();

    // build logs
    timeval T_now;
    tm *area;
    gettimeofday(&T_now,NULL);
    area=localtime(&(T_now.tv_sec));

    char* format_area = asctime(area);
    format_area[strcspn(format_area, "\n")] = '\0';

    for (int i = 0; i < strlen(format_area); i ++)
        if (format_area[i] == ' ') format_area[i] = '_';

    sprintf(write_file_name,"%s/scenario%s_test_%s.txt", log_folder.c_str(), scenario_id.c_str(), format_area);
    ofstream File_creat(write_file_name);
    File_creat.close();
    if (verbose_) printf("# write_file_name: %s\n", write_file_name);

    sprintf(collision_file_name,"%s/scenario%s_Collision_test_%s.txt", log_folder.c_str(), scenario_id.c_str(), format_area);
    ofstream Collition_File_creat(collision_file_name);
    Collition_File_creat.close();
    if (verbose_) printf("# collision_file_name: %s\n", collision_file_name);

    out.open(write_file_name,std::ios::app);
    if(out.is_open()) {
        out <<"Simulation Begin Time:"<<format_area<<endl;
        out <<"Config Path: ";
        out << Config_Path_<<endl;
        out << "id,x,y,yaw,v_lon,v_lat,v_yaw,length,width,lane_id,centerline,agent_type,is_ego_car\n";
        out.close();
    }
    else
    {
        std::cout << "no such file" << std::endl;
        exit(-1);
    }

    out.open(collision_file_name,std::ios::app);
    if(out.is_open()) {
        out <<"Simulation Begin Time:"<<format_area<<endl;
        out <<"Config Path: ";
        out << Config_Path_<<endl;
        out.close();
    }
    else
    {
        std::cout << "no such file" << std::endl;
        exit(-1);
    }

    mutex.lock();
    simulatorState = SimulatorState::Running;
    mutex.unlock();
}


void Simulator::generateReplayCar(ReplayCarInfo replay_info) {
    if (verbose_) printf("\nNew Replay Car Generated, (%d, %d, %d, %s)\n", std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info), std::get<3>(replay_info).c_str());

    // set replay car
    auto traj = ReplayGeneratorPtr->elicit_trajectory(std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info));
    auto first_point = traj.second.front().second;
    assert(first_point.size() == 8);

    Vector initstate(6, 0.0);
    for (int i = 0; i < 6; i ++) initstate[i] = first_point[i];

    if (verbose_) {
        printf("initstate: ");
        for (int i = 0; i < 6; i ++) printf("%.3lf ", initstate[i]);
        printf("\n");
    }

    ReplayAgent* virtualCar = new ReplayAgent(std::get<0>(replay_info), initstate);
    virtualCar->length_ = first_point[6];
    virtualCar->width_ = first_point[7];
    if (verbose_) printf("length: %.3lf, width: %.3lf\n", virtualCar->length_, virtualCar->width_);

    ReplayGeneratorPtr->pre_load_state(virtualCar, std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info));
    virtualCar->setTrajectory(traj);
    
    // set mapinfo
    MapInfo* mapinfo = new MapInfo(mapreader->map, mapreader->routingGraph);
    ConstLanelets lanelet_path;

    for (auto t : traj.second){
        auto x = t.second[0], y = t.second[1], yaw = t.second[2];
        auto xy2laneid_res = HelperFunction::xy2laneid(x, y, yaw, mapreader->map);
        
        //if (xy2laneid_res.second == "matches"){   //TODO:
            auto candidate_lanelet_id = xy2laneid_res.first;
            ConstLanelet candidate_lanelet = mapreader->map->laneletLayer.get(candidate_lanelet_id);
            
            bool repeat_flag = false;
            for (auto l : lanelet_path)
                if (l.id() == candidate_lanelet_id) repeat_flag = true;
            
            if (! repeat_flag){
                lanelet_path.push_back(candidate_lanelet);
            }   
        //}
    }
    mapinfo->setLaneletPath(lanelet_path);
    virtualCar->setMapinfo(mapinfo);

    // set smooth reference
    if (std::get<3>(replay_info) != ""){
        alglib::real_1d_array y_ref;
        alglib::real_1d_array x_ref;
        alglib::real_1d_array s_ref;
        if (verbose_) printf("traj size: %d\n", int(traj.second.size()));

        y_ref.setlength(traj.second.size());
        x_ref.setlength(traj.second.size());
        s_ref.setlength(traj.second.size());

        int ref_size = -1;
        for (int i = 0; i < traj.second.size(); i ++){
            double x = traj.second[i].second[0];
            double y = traj.second[i].second[1];

            if (i == 0 || sqrt((x - x_ref[ref_size])*(x - x_ref[ref_size]) + (y - y_ref[ref_size])*(y - y_ref[ref_size])) > 0.00001){
                ref_size ++;
                y_ref[ref_size] = y;
                x_ref[ref_size] = x;
                s_ref[ref_size] = ref_size;
            }
        }
        ref_size ++;
        if (verbose_) printf("ref_size: %d\n", ref_size);

        {
            alglib::real_1d_array y_ref_new;
            alglib::real_1d_array x_ref_new;
            alglib::real_1d_array s_ref_new;

            y_ref_new.setlength(ref_size);
            x_ref_new.setlength(ref_size);
            s_ref_new.setlength(ref_size);

            for (int i = 0; i < ref_size; i ++){
                y_ref_new[i] = y_ref[i];
                x_ref_new[i] = x_ref[i];
                s_ref_new[i] = s_ref[i];
            }

            alglib::spline1dbuildcubic(s_ref_new, x_ref_new, mapinfo->spl_ref_xs_);
            alglib::spline1dbuildcubic(s_ref_new, y_ref_new, mapinfo->spl_ref_ys_);
        }

        y_ref.setlength(10 * ref_size);
        x_ref.setlength(10 * ref_size);
        s_ref.setlength(10 * ref_size);
        s_ref[0]=0;
        mapinfo->reference_.clear();

        for (int i = 0; i < 10 * ref_size; i ++){
            double xx = 0, yy = 0, dx = 0, dy = 0, d2x = 0, d2y = 0;
            alglib::spline1ddiff(mapinfo->spl_ref_xs_, i*0.1, xx, dx, d2x);
            alglib::spline1ddiff(mapinfo->spl_ref_ys_, i*0.1, yy, dy, d2y);

            x_ref[i] = xx;
            y_ref[i] = yy;
            Point2d p(lanelet::utils::getId(),xx,yy);
            mapinfo->reference_.push_back(p);

            if (i>0) s_ref[i] = s_ref[i-1] + sqrt((xx-x_ref[i-1])*(xx-x_ref[i-1])+(yy-y_ref[i-1])*(yy-y_ref[i-1]));
        }

        //alglib::spline1dbuildcubic(s_ref, x_ref, 10 * ref_size, 2, 0.0, 2, 0.0, mapinfo->spl_ref_xs_);
        alglib::spline1dbuildcubic(s_ref, x_ref, mapinfo->spl_ref_xs_);
        //alglib::spline1dbuildcubic(s_ref, y_ref, 10 * ref_size, 2, 0.0, 2, 0.0, mapinfo->spl_ref_ys_);
        alglib::spline1dbuildcubic(s_ref, y_ref, mapinfo->spl_ref_ys_);
        
        mapinfo->total_ref_length = s_ref[10 * ref_size - 1];
    }
    mapinfo->init(std::get<0>(replay_info), initstate);

    // set predictor & push to Dictionary
    if (std::get<3>(replay_info) == ""){
        auto *no_predictor = new class NoPredictor(virtualCar, 0.0, 0.0);
        virtualCar->setInPredictor(no_predictor);

        assert(virtualCar->getExPredictor() == nullptr);
        if (verbose_) printf("in_predictor info: no_predictor\n");
        if (verbose_) printf("ex_predictor info: nullptr\n");
        
        mutex.lock();
        replayAgentDictionary.push_back(virtualCar);
        mutex.unlock();
        if (verbose_) printf("Add into replayAgentDictionary\n");
    }
    else {
        std::istringstream iss(std::get<3>(replay_info));

        //set in_predictor (compulsory)
        string in_predictor_type;
        double in_predictor_dt, in_predictor_horizon;

        iss >> in_predictor_type >> in_predictor_dt >> in_predictor_horizon;
        if (verbose_) printf("in_predictor info: %s, %.3lf, %.3lf\n", in_predictor_type.c_str(), in_predictor_dt, in_predictor_horizon);
        
        if (in_predictor_type == "ConstantSpeed"){
            auto *conspre = new class ConstantSpeedPredictor(virtualCar, in_predictor_dt, in_predictor_horizon);
            virtualCar->setInPredictor(conspre);
        }
        else if (in_predictor_type == "GroundTruth"){
            auto *gt_predictor = new class GroundTruthPredictor(virtualCar, in_predictor_dt, in_predictor_horizon);
            virtualCar->setInPredictor(gt_predictor);
        }
        else throw std::runtime_error("Invalid in_predictor_type!");

        //set ex_predictor (optional)
        string ex_predictor_type;
        double ex_predictor_dt, ex_predictor_horizon;

        iss >> ex_predictor_type >> ex_predictor_dt >> ex_predictor_horizon;
        if (verbose_) printf("ex_predictor info: %s, %.3lf, %.3lf\n", ex_predictor_type.c_str(), ex_predictor_dt, ex_predictor_horizon);

        if (ex_predictor_type == "yes"){
            auto *py_predictor = new class PyPredictor(virtualCar, ex_predictor_dt, ex_predictor_horizon);
            virtualCar->setExPredictor(py_predictor);
        }
        else {
            assert(ex_predictor_type == "no");
            assert(virtualCar->getExPredictor() == nullptr);
        }

        std::tuple<Controller*, Model*, Planner*> temp(
                new VirtualCarController(), // create corresponding controller
                new VirtualCarModel(), // create corresponding model
                new AstarPlanner(virtualCar, mapinfo) // create corresponding plannerh
        );
        auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(virtualCar, temp);

        mutex.lock();
        this->agentDictionary.insert(pair); // add the car to the simulator
        this->humanInputs.insert(std::pair<Agent*, Vector>(virtualCar, Vector(3))); // add the car to the simulator
        mutex.unlock();
        if (verbose_) printf("Add into agentDictionary\n");
    }
}

void Simulator::generateBehaveCar(ReplayCarInfo behave_info) {
    if (verbose_) printf("\nNew Behave Car Generated, (%d, %d, %d, %s)\n", std::get<0>(behave_info), std::get<1>(behave_info), std::get<2>(behave_info), std::get<3>(behave_info).c_str());
    
    // set behave car
    auto traj = ReplayGeneratorPtr->elicit_trajectory(std::get<0>(behave_info), std::get<1>(behave_info), std::get<2>(behave_info));
    auto first_point = traj.second.front().second;
    assert(first_point.size() == 8);

    Vector initstate(6, 0.0);
    for (int i = 0; i < 6; i ++) initstate[i] = first_point[i];

    if (verbose_) {
        printf("initstate: ");
        for (int i = 0; i < 6; i ++) printf("%.3lf ", initstate[i]);
        printf("\n");
    }

    auto virtualCar = new class BehaveCar(std::get<0>(behave_info), initstate);
    virtualCar->length_ = first_point[6];
    virtualCar->width_ = first_point[7];
    if (verbose_) printf("length: %.3lf, width: %.3lf\n", virtualCar->length_, virtualCar->width_);

    ReplayGeneratorPtr->pre_load_state(virtualCar, std::get<0>(behave_info), std::get<1>(behave_info), std::get<2>(behave_info));
    
    // set mapinfo
    MapInfo* mapinfo = new MapInfo(mapreader->map, mapreader->routingGraph);
    ConstLanelets lanelet_path;

    for (auto t : traj.second){
        auto x = t.second[0], y = t.second[1], yaw = t.second[2];
        auto xy2laneid_res = HelperFunction::xy2laneid(x, y, yaw, mapreader->map);
        
        //if (xy2laneid_res.second == "matches"){   //TODO:
            auto candidate_lanelet_id = xy2laneid_res.first;
            ConstLanelet candidate_lanelet = mapreader->map->laneletLayer.get(candidate_lanelet_id);
            
            bool repeat_flag = false;
            for (auto l : lanelet_path)
                if (l.id() == candidate_lanelet_id) repeat_flag = true;
            
            if (! repeat_flag){
                lanelet_path.push_back(candidate_lanelet);
            }
        //}
    }
    mapinfo->setLaneletPath(lanelet_path);
    /*
    int startLaneletId = lanelet_path.front().id(), endLaneletId = lanelet_path.back().id();
    if (verbose_) printf("routing info: %d %d\n", startLaneletId, endLaneletId);

    ConstLanelet fromLanelet = mapreader->map->laneletLayer.get(startLaneletId);
    ConstLanelet toLanelet = mapreader->map->laneletLayer.get(endLaneletId);

    if (!mapinfo->setRoutingPath(fromLanelet, toLanelet)) 
        throw std::runtime_error("No route exist for "+std::to_string(startLaneletId)+" and "+std::to_string(endLaneletId));
    */

    virtualCar->setMapinfo(mapinfo);
    
    // set smooth reference
    alglib::real_1d_array y_ref;
    alglib::real_1d_array x_ref;
    alglib::real_1d_array s_ref;
    if (verbose_) printf("traj size: %d\n", int(traj.second.size()));

    y_ref.setlength(traj.second.size());
    x_ref.setlength(traj.second.size());
    s_ref.setlength(traj.second.size());

    int ref_size = -1;
    for (int i = 0; i < traj.second.size(); i ++){
        double x = traj.second[i].second[0];
        double y = traj.second[i].second[1];

        if (i == 0 || sqrt((x - x_ref[ref_size])*(x - x_ref[ref_size]) + (y - y_ref[ref_size])*(y - y_ref[ref_size])) > 0.00001){
            ref_size ++;
            y_ref[ref_size] = y;
            x_ref[ref_size] = x;
            s_ref[ref_size] = ref_size;
        }
    }
    ref_size ++;
    if (verbose_) printf("ref_size: %d\n", ref_size);

    {
        alglib::real_1d_array y_ref_new;
        alglib::real_1d_array x_ref_new;
        alglib::real_1d_array s_ref_new;

        y_ref_new.setlength(ref_size);
        x_ref_new.setlength(ref_size);
        s_ref_new.setlength(ref_size);

        for (int i = 0; i < ref_size; i ++){
            y_ref_new[i] = y_ref[i];
            x_ref_new[i] = x_ref[i];
            s_ref_new[i] = s_ref[i];
        }

        alglib::spline1dbuildcubic(s_ref_new, x_ref_new, mapinfo->spl_ref_xs_);
        alglib::spline1dbuildcubic(s_ref_new, y_ref_new, mapinfo->spl_ref_ys_);
        //alglib::spline1dbuildcubic(s_ref, x_ref, ref_size, 2, 0.0, 2, 0.0, mapinfo->spl_ref_xs_);
        //alglib::spline1dbuildcubic(s_ref, y_ref, ref_size, 2, 0.0, 2, 0.0, mapinfo->spl_ref_ys_);
    }
    //printf("# DEBUG | BP 1\n");

    y_ref.setlength(10 * ref_size);
    x_ref.setlength(10 * ref_size);
    s_ref.setlength(10 * ref_size);
    s_ref[0]=0;
    mapinfo->reference_.clear();

    for (int i = 0; i < 10 * ref_size; i ++){
        double xx = 0, yy = 0, dx = 0, dy = 0, d2x = 0, d2y = 0;
        alglib::spline1ddiff(mapinfo->spl_ref_xs_, i*0.1, xx, dx, d2x);
        alglib::spline1ddiff(mapinfo->spl_ref_ys_, i*0.1, yy, dy, d2y);

        x_ref[i] = xx;
        y_ref[i] = yy;
        Point2d p(lanelet::utils::getId(),xx,yy);
        mapinfo->reference_.push_back(p);

        if (i>0) s_ref[i] = s_ref[i-1] + sqrt((xx-x_ref[i-1])*(xx-x_ref[i-1])+(yy-y_ref[i-1])*(yy-y_ref[i-1]));
    }
    //printf("# DEBUG | BP 2\n");
    //printf("%d %d\n", int(mapinfo->reference_.size()), int(s_ref.length()));
    //printf("ID: %d\n", virtualCar->getId());

    //alglib::spline1dbuildcubic(s_ref, x_ref, 10 * ref_size, 2, 0.0, 2, 0.0, mapinfo->spl_ref_xs_);
    alglib::spline1dbuildcubic(s_ref, x_ref, mapinfo->spl_ref_xs_);
    //printf("# DEBUG | BP 2-1\n");
    //alglib::spline1dbuildcubic(s_ref, y_ref, 10 * ref_size, 2, 0.0, 2, 0.0, mapinfo->spl_ref_ys_);
    alglib::spline1dbuildcubic(s_ref, y_ref, mapinfo->spl_ref_ys_);
    //printf("# DEBUG | BP 2-2\n");
    mapinfo->total_ref_length = s_ref[10 * ref_size - 1];
    
    //printf("# DEBUG | BP 3\n"); 
    mapinfo->init(std::get<0>(behave_info), initstate);
    //printf("# DEBUG | BP 4\n");
    
    // set planner
    std::istringstream iss(std::get<3>(behave_info));

    string planner_type, planner_para;
    iss >> planner_type >> planner_para;
    if (verbose_) printf("planner info: %s %s\n", planner_type.c_str(), planner_para.c_str());

    if (planner_type == "IDM"){
        AoBehaviour *aobehave = new class AoBehaviour(virtualCar, BehaviourType::IDM);
        aobehave -> mapinfo_=mapinfo;
        virtualCar->setBehaviour(aobehave);
        virtualCar->IDM_ = true;
    }
    else if (planner_type == "Astar" || planner_type == "ReactivePlanner"){ //TODO:
        AoBehaviour *aobehave = new class AoBehaviour(virtualCar, BehaviourType::IDM);
        aobehave -> mapinfo_=mapinfo;
        virtualCar->setBehaviour(aobehave);
        virtualCar->setfollowingPlanner(new AstarPlanner(virtualCar, mapinfo)); // new CILQRPlanner
        virtualCar->IDM_ = false;
    }
/*    else if (planner_type == "GameTree"){
        AoBehaviour *aobehave = new class AoBehaviour(virtualCar, BehaviourType::IDM);
        aobehave -> mapinfo_=mapinfo;
        virtualCar->setBehaviour(aobehave);
        virtualCar->setfollowingPlanner(new GameTreePlanner(virtualCar, mapinfo)); // new CILQRPlanner
        virtualCar->IDM_ = false;
    }
*/
    else if (planner_type == "EB"){
        AoBehaviour *aobehave = new class AoBehaviour(virtualCar, BehaviourType::IDM);
        aobehave -> mapinfo_=mapinfo;
        virtualCar->setBehaviour(aobehave);
        virtualCar->setfollowingPlanner(new EBPlanner(virtualCar, mapinfo)); // new CILQRPlanner
        virtualCar->IDM_ = false;
    }
    else throw std::runtime_error("Invalid planner_type");

    //set in_predictor (compulsory)
    string in_predictor_type;
    double in_predictor_dt, in_predictor_horizon;

    iss >> in_predictor_type >> in_predictor_dt >> in_predictor_horizon;
    if (verbose_) printf("in_predictor info: %s, %.3lf, %.3lf\n", in_predictor_type.c_str(), in_predictor_dt, in_predictor_horizon);
    
    if (in_predictor_type == "ConstantSpeed"){
        auto *conspre = new class ConstantSpeedPredictor(virtualCar, in_predictor_dt, in_predictor_horizon);
        virtualCar->setInPredictor(conspre);
    }
    else if (in_predictor_type == "GroundTruth"){
        auto *gt_predictor = new class GroundTruthPredictor(virtualCar, in_predictor_dt, in_predictor_horizon);
        virtualCar->setInPredictor(gt_predictor);
    }
    else throw std::runtime_error("Invalid in_predictor_type!");

    //set ex_predictor (optional)
    string ex_predictor_type;
    double ex_predictor_dt, ex_predictor_horizon;

    iss >> ex_predictor_type >> ex_predictor_dt >> ex_predictor_horizon;
    if (verbose_) printf("ex_predictor info: %s, %.3lf, %.3lf\n", ex_predictor_type.c_str(), ex_predictor_dt, ex_predictor_horizon);

    if (ex_predictor_type == "yes"){
        auto *py_predictor = new class PyPredictor(virtualCar, ex_predictor_dt, ex_predictor_horizon);
        virtualCar->setExPredictor(py_predictor);
    }
    else {
        assert(ex_predictor_type == "no");
        assert(virtualCar->getExPredictor() == nullptr);
    }

    //set ego_car
    string is_ego_car;
    iss >> is_ego_car;
    if (verbose_) printf("is_ego_car: %s\n", is_ego_car.c_str());

    if (is_ego_car == "yes"){
        virtualCar->setEgoCar();
        assert(virtualCar->isEgoCar() == true);
    }
    else{
        assert(is_ego_car == "no");
        assert(virtualCar->isEgoCar() == false);
    }

    // add to agentDictionary
    std::tuple<Controller*, Model*, Planner*> temp(
            new VirtualCarController(), // create corresponding controller
            new VirtualCarModel(), // create corresponding model
            new AstarPlanner(virtualCar, mapinfo) // create corresponding plannerh
    );
    auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(virtualCar, temp);

    mutex.lock();
    this->agentDictionary.insert(pair); // add the car to the simulator
    this->humanInputs.insert(std::pair<Agent*, Vector>(virtualCar, Vector(3))); // add the car to the simulator
    mutex.unlock();
    if (verbose_) printf("Add into agentDictionary\n");

    /* Old version
    int id = random() % 1000; //total_car_num ++;
    printf("************* New Behave Car ID: %d ***********\n", id);

    class BehaveCar *virtualCar = new class BehaveCar(id, Vector{
            5000.0,
            5000.0,
            0.0,
            5.0,
            0.0,
            0.0
    }); // create a new car

    Agent* tmpAgent = virtualCar;
    MapInfo* mapinfo = CarGenerator::generatemapinfo(mapreader->map, mapreader->routingGraph, mapreader->StartLaneletIds,mapreader->EndLaneletIds);

    while(tmpAgent != nullptr) {
        delete mapinfo;
        mapinfo = CarGenerator::generatemapinfo(mapreader->map, mapreader->routingGraph, mapreader->StartLaneletIds,mapreader->EndLaneletIds);
        ConstLanelet tmpLl = mapinfo->getCurrentLanelet();
        //mutex.lock();
        //std::vector<Agent *> agents = Simulator::agentsForThread;
        //mutex.unlock();
        tmpAgent = nullptr;
        for (auto pair : agentDictionary) {
            auto agent = pair.first;

            if (!agent->getState().empty()) {
                lanelet::BasicPoint2d pos(agent->getState()[0], agent->getState()[1]);
                if(geometry::inside(tmpLl, pos)) {
                    tmpAgent = agent;
                    break;
                }
            }
        }
    }
    BasicPoint2d pinit = geometry::interpolatedPointAtDistance(mapinfo->getCurrentLanelet().centerline2d(), 0);
    BasicPoint2d pinit_f = geometry::interpolatedPointAtDistance(mapinfo->getCurrentLanelet().centerline2d(), 0.01);
    BasicPoint2d pDirection = pinit_f - pinit;
    virtualCar->setState({
            pinit.x(),
            pinit.y(),
            std::atan2(pDirection.y(),pDirection.x()),
            5.0,
            0.0,
            0.0
    }); // reset the init position

    mapinfo->init(id, {
            pinit.x(),
            pinit.y(),
            std::atan2(pDirection.y(),pDirection.x()),
            5.0,
            0.0,
            0.0
    });

    AoBehaviour *aobehave = new class AoBehaviour(BehaviourType::IDM);
    aobehave -> mapinfo_=mapinfo;
    //ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,0.2,2);
    PyPredictor *py_predictor = new PyPredictor(virtualCar,0.2,2);

    virtualCar->setfollowingPlanner(new AstarPlanner(mapinfo));
    // for CILQR planner
    ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,0.2,5);
    CILQRPlanner * p = new CILQRPlanner(mapinfo);
    p->setId(id); // Car Id
    virtualCar->setfollowingPlanner(p);
    virtualCar->setlinechangePlanner(p);

    virtualCar->setBehaviour(aobehave);
    virtualCar->setMapinfo(mapinfo);
    virtualCar->setPredictor(py_predictor);

    std::tuple<Controller*, Model*, Planner*> temp(
            new VirtualCarController(), // create corresponding controller
            new VirtualCarModel(), // create corresponding model
            new AstarPlanner(mapinfo) // create corresponding plannerh
    );
    auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(virtualCar, temp);
    mutex.lock();
    this->agentDictionary.insert(pair); // add the car to the simulator
    this->humanInputs.insert(std::pair<Agent*, Vector>(virtualCar, Vector(3))); // add the car to the simulator
    mutex.unlock();
    */
}

int Simulator::collisionWithLane(double x[], double y[],  ConstLineString2d left, ConstLineString2d right, double xx, double yy, double Thre)
{

    BasicPoint2d egoPos(xx, yy);
    double S1 = geometry::toArcCoordinates(left, egoPos).distance;
    double S2 = geometry::toArcCoordinates(right, egoPos).distance;
    if(S1 * S2 > 0){
        return 1;
    }
    return 0;
}

void Simulator::isThereCollision(){
	double xx = 0;
	double yy = 0;
	double angle = 0;
	double ob_xx = 0;
	double ob_yy = 0;
	double ob_angle = 0;
	double length = 0;
	double width = 0;
	double ob_length = 0;
	double ob_width = 0;
	double Thre = 30;
	double thre = 10;
	double x[8];
	double y[8];
    double numb = 0;
	/*mutex.lock();
	std::vector<Agent *> agentss = Simulator::agentsForThread;
	mutex.unlock();*/

    if (verbose_) printf("\n");
	for (auto pair : this->agentDictionary){
		Agent* agent = pair.first;
		xx = agent->getState()[0];
		yy = agent->getState()[1];
		angle = agent->getState()[2];
		length = agent->length_ - 0.05;     //give a small tolerance
		width = agent->width_ - 0.05;
		x[0] = xx + 0.5 * length * cos(angle) - 0.5 * width * sin(angle);
		x[1] = xx + 0.5 * length * cos(angle) + 0.5 * width * sin(angle);
		x[2] = xx - 0.5 * length * cos(angle) + 0.5 * width * sin(angle);
		x[3] = xx - 0.5 * width * sin(angle) - 0.5 * length * cos(angle);
		y[0] = yy + 0.5 * length * sin(angle) + 0.5 * width * cos(angle);
		y[1] = yy + 0.5 * length * sin(angle) - 0.5 * width * cos(angle);
		y[2] = yy - 0.5 * width * cos(angle) - 0.5 * length * sin(angle);
		y[3] = yy - 0.5 * length * sin(angle) + 0.5 * width * cos(angle);
		for (auto pair1 : this->agentDictionary){
			Agent* agen = pair1.first;
			if((agent != agen) && ((xx-agen->getState()[0])*(xx-agen->getState()[0])+(yy-agen->getState()[1])*(yy-agen->getState()[1])<Thre)){
				ob_xx = agen->getState()[0];
				ob_yy = agen->getState()[1];
				ob_angle = agen->getState()[2];
				ob_length = agen->length_ - 0.05;
				ob_width = agen->width_ - 0.05;
				x[4] = ob_xx + 0.5 * ob_length * cos(ob_angle) - 0.5 * ob_width * sin(ob_angle);
				x[5] = ob_xx + 0.5 * ob_length * cos(ob_angle) + 0.5 * ob_width * sin(ob_angle);
				x[6] = ob_xx - 0.5 * ob_length * cos(ob_angle) + 0.5 * ob_width * sin(ob_angle);
				x[7] = ob_xx - 0.5 * ob_width * sin(ob_angle) - 0.5 * ob_length * cos(ob_angle);
				y[4] = ob_yy + 0.5 * ob_length * sin(ob_angle) + 0.5 * ob_width * cos(ob_angle);
				y[5] = ob_yy + 0.5 * ob_length * sin(ob_angle) - 0.5 * ob_width * cos(ob_angle);
				y[6] = ob_yy - 0.5 * ob_width * cos(ob_angle) - 0.5 * ob_length * sin(ob_angle);
				y[7] = ob_yy - 0.5 * ob_length * sin(ob_angle) + 0.5 * ob_width * cos(ob_angle);
				if (collisionBetweenCar(x, y) == 1){
                    numb++;
					if (verbose_)cout<<"There is a collision between car "<<agent->getId()<<" and car "<<agen->getId()<<endl;
                    out.open(collision_file_name,std::ios::app);
                    if(out.is_open()) {
                        out <<to_string(updateTimes)<<" "<<"collision between cars(CarID CarID):"<<to_string(agent->getId())<<" "<<to_string(agen->getId())<<endl;
                        out.close();
                    }
                    else
                    {
                        std::cout << "no such file" << std::endl;
                        exit(-1);
                    }
				}
			}
		}
        if(numb == 0){
            if (verbose_) cout<<"There are no collisions between cars!"<<endl;;
        }

        if (agent->getType() == AgentType::ReplayCar)
            continue;

		Agent* tmpAgent = agent;
        MapInfo* this_map_info = agent->mapinfo;
		ConstLanelet tmpLl = this_map_info->getCurrentLanelet();
		ConstLineString2d LEFT = tmpLl.leftBound2d();
		ConstLineString2d RIGHT = tmpLl.rightBound2d();
		if(collisionWithLane(x,y,LEFT,RIGHT,xx,yy,thre) == 1){
		    if (verbose_) cout<<"The car "<<agent->getId()<<" collides with the lane "<<this_map_info->getCurrentLanelet().id()<<"!"<<endl;
            out.open(collision_file_name,std::ios::app);
            if(out.is_open()) {
                out <<to_string(updateTimes)<<" "<<"collision with lane(CarID laneID):"<<to_string(agent->getId())<<" "<<to_string(this_map_info->getCurrentLanelet().id())<<endl;
                out.close();
            }
            else
            {
                std::cout << "no such file" << std::endl;
                exit(-1);
            }
		}
	}
}

bool Simulator::removeAgentIfNeeded() {
    mutex.lock();
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;
        if (agent->hasReachedDestinaiton || (agent->getState()).size() == 0) {
            while(agent->isRunning) {
                usleep(1e6 * SIM_TICK);
                continue;
            }
            this->agentDictionary.erase(agent);
            if (verbose_) printf("# Remove Agent (%d) From agentDictionary \n", agent->getId());
            mutex.unlock();
            return true;
        }
    }

    for(int i = 0; i < replayAgentDictionary.size(); i++){
        Agent* agent = replayAgentDictionary[i];

        if (agent->hasReachedDestinaiton || (agent->getState()).size() == 0) {
            while(agent->isRunning) {
                usleep(1e6 * SIM_TICK);
                continue;
            }
            replayAgentDictionary.erase(replayAgentDictionary.begin() + i);
            if (verbose_) printf("# Remove Agent (%d) From replayAgentDictionary \n", agent->getId());
            mutex.unlock();
            return true;
        }
    }
    mutex.unlock();
    return false;
}

///
/// add agent if need
void Simulator::Agentmanager(){
    for (auto behave_info : BehaveCarWaitList[updateTimes]){
        generateBehaveCar(behave_info);
    }

    for (auto replay_info : ReplayCarWaitList[updateTimes]){
        if (std::get<3>(replay_info) != ""){
            generateReplayCar(replay_info);
        }
        else {
            //if (replayAgentDictionary.size() < Car_Num)
                generateReplayCar(replay_info);
            //else 
            //    if (verbose_) printf("\nGenerating ReplayCar (%d) to replayAgentDictionary Failed, since Car_Num is too small!!!\n", std::get<0>(replay_info));
        }
    }

    /* Old version
    int JinningCar_id = 0;
    int generate_time = 1;
    int last_updateTimes = -1;  // Add replay car since update time >= 0

    while (true)
    {
        if (updateTimes != last_updateTimes){
            last_updateTimes = updateTimes;

            for (auto replay_info : ReplayCarWaitList[updateTimes])
                if (std::get<std::string>(replay_info) != ""){
                    if (this->agentDictionary.size() < Car_Num)
                         generateReplayCar(replay_info);
                    else
                        printf("\nGenerating ReplayCar (%d) to agentDictionary Failed, since Car_Num is too small!!!\n", std::get<0>(replay_info));
                }
                else {
                    if (replayAgentDictionary.size() < Car_Num)
                        generateReplayCar(replay_info);
                    else
                        printf("\nGenerating ReplayCar (%d) to replayAgentDictionary Failed, since Car_Num is too small!!!\n", std::get<0>(replay_info));
                }
        }

        if (this -> updateTimes%10 == 0 && this->agentDictionary.size() == Car_Num-1) { // generate a CILQR car
            bool exist_CILQR = false;
            for (auto pair : this->agentDictionary)
                if (pair.first->getType() == 1) {
                    exist_CILQR = true;
                    break;
                }
            if (! exist_CILQR) generateVirtualCar();
        } else if (this -> updateTimes%10 == 0 && this->agentDictionary.size() < Car_Num) {
            generateReplayCar();
        }
    }
    */
}

///
/// Start the simulator. This is an infinite loop.
void Simulator::run() {
    // set initial state for the robot cars
    assert(agentDictionary.size() == 0);
    assert(replayAgentDictionary.size() == 0);

    while (true){

        // check all the cars have finished updates from PREDICTOR
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;
            auto in_predictor = agent->getInPredictor();
            auto ex_predictor = agent->getExPredictor();

            while (
                agent->isRunning ||
                in_predictor->get_state() != SubprocessState::wait4update ||
                (ex_predictor != nullptr && ex_predictor->get_state() != SubprocessState::wait4update)
            ){
                usleep(1e6 * SIM_TICK);
            }
        }

        for (int i = 0; i < replayAgentDictionary.size(); i++){
            Agent* agent = replayAgentDictionary[i];
            auto in_predictor = agent->getInPredictor();
            assert(in_predictor->getType() == PredictorType::NoPredictor);

            while(agent->isRunning || in_predictor->get_state() != SubprocessState::wait4update){
                usleep(1e6 * SIM_TICK);
            }
        }

        // check that all cars finish update from PLANNER
        for (auto pair: this->agentDictionary) {
            Agent *agent = pair.first;
            auto planner = agent->getPlanner();
            while (agent->isRunning || planner->get_state() != SubprocessState::wait4update) {
                usleep(1e6 * SIM_TICK);
            }
        }

        //set predictor_state as fine, and prepare for a tick
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;
            auto in_predictor = agent->getInPredictor();
            auto ex_predictor = agent->getExPredictor();

            assert(in_predictor->get_state() == SubprocessState::wait4update);
            in_predictor->set_state(SubprocessState::fine);

            if (ex_predictor != nullptr){
                assert(ex_predictor->get_state() == SubprocessState::wait4update);
                ex_predictor->set_state(SubprocessState::fine);
            }
        }
        for (int i = 0; i < replayAgentDictionary.size(); i++){
            Agent* agent = replayAgentDictionary[i];
            auto in_predictor = agent->getInPredictor();

            assert(in_predictor->get_state() == SubprocessState::wait4update);
            in_predictor->set_state(SubprocessState::fine);
        }
    
        //set planner_state as fine, and prepare for a tick
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;
            auto planner = agent->getPlanner();
            assert(planner->get_state() == SubprocessState::wait4update);
            planner->set_state(SubprocessState::fine);
        }

        // one update has finished, remove the unnecessary cars
        while (removeAgentIfNeeded()){}

        // print progress bar to console
        {
            int barWidth = 50;
            double progress = 1.0 * updateTimes / MaxUpdateTimes_;

            std::cout << "# UpdateTimes: [";
            int pos = int(barWidth * progress);
            for (int i = 0; i < barWidth; ++i) {
                if (i < pos) std::cout << "=";
                else if (i == pos) std::cout << ">";
                else std::cout << " ";
            }
            std::cout << "] " << int(progress * 100.0) << "% (" << updateTimes << " / " << MaxUpdateTimes_ << ")\r";
            std::cout.flush();

            //printf("# updateTimes (%d / %d)\n", updateTimes, MaxUpdateTimes_);
        }

        // add new cars at this `updateTimes`
        Agentmanager();

        //Log this moment
        LogTick();
        isThereCollision();

        // check whether updateTimes==MaxUpdateTimes_
        if (updateTimes==MaxUpdateTimes_) {
            mutex.lock();
            simulatorState = Paused;
            mutex.unlock();
            if (verbose_) printf("\n# updateTimes == MaxUpdateTimes_, set simulatorState as Paused!\n");

            out.open(write_file_name, std::ios::app);
            if (out.is_open()) {
                out << "no crash" << endl;
                out << "FinalState:id,s_now,s_tot" << endl;

                for (auto pair : this->agentDictionary){
                    auto agent = pair.first;
                    if (agent->getType() != BehaveCar) continue;

                    auto s_now = geometry::toArcCoordinates(agent->mapinfo->reference_, BasicPoint2d(agent->getState()[0], agent->getState()[1])).length;
                    auto s_tot = agent->mapinfo->total_ref_length;

                    out << agent->getId() << "," << s_now << "," << s_tot << endl;
                }

                out.close();
            }
            else {
                cout << "no such file" << endl;
                exit(-1);
            }
        }

        if (simulatorState == Paused){
            if (verbose_) printf("wait for the client to close the simulator...\n");
            usleep(1e6 * 3);

            printf("# No client, closed by simulator itself\n");
            exit(0);
        }

        // A New Tick
        if (verbose_) printf("\n# UpdateTime: %d, agentDictionary size: %d, replayAgentDictionary size: %d\n", updateTimes, int(agentDictionary.size()), int(replayAgentDictionary.size()));
        if (verbose_) printf("# Tick.....\n\n");

        mutex.lock();
        SimulatorState simulatorState = this->simulatorState; // get the current simulator state
        mutex.unlock();

        switch (simulatorState) {
            case Running:
                gettimeofday(&t1, NULL);
                this->updateTick(); // advance the simulation by updating all agents.
                updateTimes ++;
                gettimeofday(&t2, NULL);
                break;
            case Reset:
                assert(false);  //TODO: don't support this function
                this->reset();
                mutex.lock();
                this->simulatorState = Running;
                mutex.unlock();
                break;
            case Paused:
                assert(false);  //TODO: simulator will be closed in advance
                break;
        }
    }
}

void Simulator::LogTick() {
    std::string writebuf = to_string(updateTimes)+" ";

    std::vector<Agent*> agent_for_log;
    for (auto pair : this->agentDictionary)
        if (pair.first->getType() != AgentType::RealCar)
            agent_for_log.push_back(pair.first);

    for (int i = 0; i < replayAgentDictionary.size(); i ++)
        if (replayAgentDictionary[i]->getType() != AgentType::RealCar)
            agent_for_log.push_back(replayAgentDictionary[i]);

    writebuf += to_string(agent_for_log.size()) + "\n";

    for (auto agent : agent_for_log){
        Vector vehstate;

        int i = agent->getId();
        vehstate = agent->getState();

        //calc center_line_direction
        auto currentlanelet = agent->mapinfo->getCurrentLanelet();
        double s_now = agent->mapinfo->getS();

        if (abs(s_now - geometry::toArcCoordinates(currentlanelet.centerline2d(), BasicPoint2d(vehstate[0], vehstate[1])).length) > 1e-3){
            printf("#### DEBUG | s_now: %.3lf, toArcCoordinates: %.3lf\n", s_now, geometry::toArcCoordinates(currentlanelet.centerline2d(), BasicPoint2d(vehstate[0], vehstate[1])).length);
            exit(-1);
        }

        auto p_now = geometry::interpolatedPointAtDistance(currentlanelet.centerline2d(), s_now);
        auto p_after = geometry::interpolatedPointAtDistance(currentlanelet.centerline2d(), s_now + 0.1);
        auto p_direction = p_after - p_now;
        double center_line_direction = atan2(p_direction.y(), p_direction.x());

        //getType
        AgentType agent_type_id = agent->getType();
        std::string agent_type_str = "";

        if (agent_type_id == AgentType::ReplayCar){
            agent_type_str = "ReplayCar";
        }   else
        if (agent_type_id == AgentType::BehaveCar){
            agent_type_str = "BehaveCar";
        }
        else assert(false);

        std::string is_ego_car = "";
        if (agent->isEgoCar())
            is_ego_car = "yes";
        else
            is_ego_car = "no";

        writebuf += std::to_string(i) + ',' + std::to_string(vehstate[0]) + ',' + std::to_string(vehstate[1]) + ',' +
                    std::to_string(vehstate[2]) + ',' + std::to_string(vehstate[3]) + ',' + std::to_string(vehstate[4])
                    +',' + std::to_string(vehstate[5]) + ','  + std::to_string(agent->length_) + ',' + std::to_string(agent->width_)
                    + ','+ std::to_string(agent->mapinfo->getCurrentLaneletId()) + ',' + std::to_string(center_line_direction)
                    + ',' + agent_type_str + ',' + is_ego_car + '\n';
    }
    writebuf = "-----------------\n" + writebuf;
    out.open(write_file_name,std::ios::app);
    if(out.is_open()) {
        out << writebuf;
        out.close();
    }
    else
    {
        cout << "no such file" << endl;
        exit(-1);
    }
}


///
/// Record the agents states. Update the agents in the simulation. This is called for each iteration step.
void Simulator::updateTick() {
    mutex.lock();

    // Update traffic info
    Simulator::trafficInfoManagerPtr->Update();

    humanInputsForThread = humanInputs;
    /*
    //virtual cars update
    // if (flagForVirtualCar == 1 && managerForVirtualCar == 1){
    //     this->lineNumber ++;
    //     vector<string> tempInfo;
    //     string file = "/home/mscsim/mkz-mpc-control/output.txt";
    //     infile.open(file.data());
    //     string s;
    //     for (int j = 0; j < this->lineNumber; j ++)
    //     {
    //         getline(infile,s);
    //     }
    //     tempInfo = split(s, "&");
    //     for (int i = 0; i < tempInfo.size(); i++){
    //         if ((i - 1) % 7 == 4){

    //             if (atoi(tempInfo[i - 3].c_str()) == 0 && atoi(tempInfo[i - 2].c_str()) == 0 && atoi(tempInfo[i - 1].c_str()) == 0){
    //                 AgentDictionary::iterator itor;
    //                 for (itor = this->agentDictionary.begin(); itor != this->agentDictionary.end();) {
    //                     if(itor->first->getId() == atoi(tempInfo[i - 4].c_str())){
    //                         itor = this->agentDictionary.erase(itor);
    //                     }
    //                     else{
    //                         ++itor;
    //                     }
    //                 }
    //             }
    //             for (auto pair : this->agentDictionary) {
    //                 if(pair.first->getId() == atoi(tempInfo[i - 4].c_str())){
    //                     pair.first->setNextState(Vector{
    //                             atof(tempInfo[i - 3].c_str()) * 100 - 10000,
    //                             -atof(tempInfo[i - 2].c_str()) * 100,
    //                             -atof(tempInfo[i - 1].c_str()) * 3.14/180.0,
    //                             0,
    //                             0,
    //                             0
    //                     });
    //                     pair.first->applyNextState();
    //                 }
    //             }
    //             class VirtualCar *virtualCar = new class VirtualCar(atoi(tempInfo[i - 4].c_str()), Vector{
    //                     atof(tempInfo[i - 3].c_str()) * 100 - 10000,
    //                     -atof(tempInfo[i - 2].c_str()) * 100,
    //                     -atof(tempInfo[i - 1].c_str()) * 3.14/180.0,
    //                     0,
    //                     0,
    //                     0
    //             }); // create a new car
    //             std::tuple<Controller*, Model*, Planner*> temp(
    //                     new VirtualCarController(), // create corresponding controller
    //                     new VirtualCarModel(), // create corresponding model
    //                     new VirtualCarPlanner() // create corresponding planner
    //             );
    //             auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(virtualCar, temp);
    //             this->agentDictionary.insert(pair); // add the car to the simulator
    //             this->humanInputs.insert(std::pair<Agent*, Vector>(virtualCar, Vector(3))); // add the car to the simulator
    //         }
    //     }

    //     infile.close();
    // }
    //std::chrono::time_point<std::chrono::system_clock> in_time = std::chrono::system_clock::now();
    */

    vector<Agent*> agents;
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; // for each agent
        /*
        if (agent->getType()== VirtualCar) {
            class VirtualCar *agentinfo = new class VirtualCar(agent->getId(), agent->getState());
            agents.push_back(agentinfo);
        }
        if (agent->getType()== HumanCar) {
            class HumanCar *agentinfo = new class HumanCar(agent->getId(), agent->getState());
            agents.push_back(agentinfo);
        }
        if (agent->getType()== RealCar){
            class RealCar *agentinfo = new class RealCar(agent->getId(), agent->getState());
            agents.push_back(agentinfo);
        }

        if (agent->getType()== ReplayCar) { // For CILQR ??
            class ReplayAgent *agentinfo = new class ReplayAgent(agent->getId(), agent->getState());
            agents.push_back(agentinfo);
        }
        */

        class BehaveCar *agentinfo = new class BehaveCar(agent->getId(), agent->getState());

        agentinfo->in_PredictTra_.Trajs.swap(agent->in_PredictTra_.Trajs);
        agentinfo->ex_PredictTra_.Trajs.swap(agent->ex_PredictTra_.Trajs);
        agentinfo->mapinfo = agent->mapinfo;
        agents.push_back(agentinfo);
    }

    for(int i = 0; i < replayAgentDictionary.size(); i++){
        Agent* agent = replayAgentDictionary[i];

        if (agent->getState().empty()) {
            assert(false);
            continue;
        }
        agent->Run();
        //this->myThreadPool.AddTask(agent, 10); // multiThreads
    }

    agentsForThread = agents;   //Agent::Run will use agentsForThread
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; // for each agent
        auto tuple = this->agentDictionary.at(agent);
        Model *model = std::get<Model *>(tuple); // get corresponding model
        Controller *controller = std::get<Controller *>(tuple); // get corresponding controller
        Planner *planner = std::get<Planner *>(tuple); // get corresponding planner
        // if (agent->getId() < 50){
        agent->setPlanner(planner);
        agent->setController(controller);
        agent->setModel(model);
        if (agent->getState().empty()) {
            assert(false);
            continue;
        }
        this->myThreadPool.AddTask(agent, 10); // multiThreads
    }


    /*
    for (auto pair : this->pAgentDictionary) {
        PedestrianAgent *pedestrianAgent = pair.first; // for each agent
        // std::cout << " Ped id : " << pedestrianAgent->getId() << std::endl;
        auto tuple = this->pAgentDictionary.at(pedestrianAgent);
        Model *model = std::get<Model *>(tuple); // get corresponding model
        Controller *controller = std::get<Controller *>(tuple); // get corresponding controller
        Planner *planner = std::get<Planner *>(tuple); // get corresponding planner
        if (pedestrianAgent->getId() < 50) {
            pedestrianAgent->setPlanner(planner);
            pedestrianAgent->setController(controller);
            pedestrianAgent->setModel(model);
            this->myThreadPool.AddTask(pedestrianAgent, 5); // multiThreads
        }
    }
    */
    //std::chrono::time_point<std::chrono::system_clock> out_time = std::chrono::system_clock::now();
    //cout<<"before calculate time: "<< double (std::chrono::duration_cast<std::chrono::milliseconds>(out_time - in_time).count()) << " ms"<<endl;
    /*
    for (Agent *agent : agents) {
    agent->applyNextState(); // Apply the next state of the agent.
    }*/
    mutex.unlock();
}


core::Trajectory Simulator::ToTraj(Agent* agent){
    assert(agent->getInPredictor()->get_state() != SubprocessState::fine);
    assert(agent->getExPredictor() == nullptr || agent->getExPredictor()->get_state() != SubprocessState::fine);
    // must be wait4fetch, wait4upload, or wait4update, which means the nextstate has already been applied

    auto prestate = agent->get_preState();
    auto curstate = agent->getState();
    core::Trajectory traj;

    Vector laststate;
    for (int t = 0; t < 10; t++){
        if (t == 0){
            laststate = curstate;
        }
        else if (prestate.size() >= 1){
            laststate = prestate[max(0, int(prestate.size()) - t * 10)];
            // since each tick in simulator is 0.01s, but each step in predictor is 0.1s
        }

        auto state = new core::State();
        state->track_id=agent->getId();
        state->frame_id=max(0, int(prestate.size()) - t * 10);
        state->timestamp_ms=state->frame_id * 10;
        state->agent_type="car";

        state->x=laststate[0];
        state->y=laststate[1];
        state->vx=laststate[3] * cos(laststate[2]);
        state->vy=laststate[3] * sin(laststate[2]);
        state->psi_rad=laststate[2];
        state->length=agent->length_;
        state->width=agent->width_;
        //TODO: s, d, lanelet_id, jerk if the predictor needs them

        traj.emplace_back(state);
    }
    reverse(traj.begin(), traj.end());

    for (int i = 1; i < traj.size(); i ++)
        assert(traj[i-1]->frame_id <= traj[i]->frame_id);

    assert(traj.size() == 10);
    return traj;
}


core::SimulationEnv Simulator::fetch_history(){
    if (verbose_) printf("\n### Fetch Env: There are %d cars now\n", int(this->agentDictionary.size()));
    core::SimulationEnv env;
    env.paused = false;

    bool isfine = false;
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;

        // fine means this car's nextstate has not been applied
        if (agent->getInPredictor()->get_state() == SubprocessState::fine){
            isfine = true;
            break;
        }
        if (agent->getExPredictor() != nullptr && agent->getExPredictor()->get_state() == SubprocessState::fine){
            isfine = true;
            break;
        }
    }

    if (! isfine){
        mutex.lock();
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;
            auto ex_predictor = agent->getExPredictor();

            if (ex_predictor != nullptr && ex_predictor->get_state() == SubprocessState::wait4fetch){
                ex_predictor->set_state(SubprocessState::wait4upload);

                if (verbose_) printf("# Find car_id %d, historical length = %d\n", agent->getId(), int(agent->get_preState().size()));
                
                env.map_name = MapName_;        //map
                env.my_traj = ToTraj(agent);     //my_traj

                for (auto p2 : this->agentDictionary)
                    if (p2.first != agent)
                        env.other_trajs.push_back(ToTraj(p2.first));
                
                for (auto agent : replayAgentDictionary)
                    env.other_trajs.push_back(ToTraj(agent));
                
                if (verbose_) printf("# size of other_trajs: %d\n", (int)env.other_trajs.size());

                mutex.unlock();
                return env;
            }
        }
        mutex.unlock();
    }

    env.map_name = MapName_;        //map
    for (int t = 0; t < 10; t ++){
        auto state = new core::State();
        state->track_id=0;
        state->frame_id=0;
        state->timestamp_ms=0;
        state->agent_type="233";
        state->x=0;
        state->y=0;
        state->vx=0;
        state->vy=0;
        state->psi_rad=0;
        state->length=0;
        state->width=0;

        env.my_traj.emplace_back(state);
    }

    if (simulatorState == Paused){
        env.paused = true;
        if (verbose_) printf("# simulatorState == Paused\n");   
    }
    else{
        if (verbose_) printf("# Did not find available car\n");
    }

    return env;
}

core::SimulationEnv Simulator::fetch_info_planner() 
{
    if (verbose_) printf("\n### Fetch Info Planner: There are %d cars now\n", int(this->agentDictionary.size()));
    core::SimulationEnv env;
    env.paused = false;
    env.map_name = MapName_;        //map

    mutex.lock();
    // TODO - set reference points and obstacle info for planner
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;
        auto my_planner = agent->getPlanner();

        if (my_planner != nullptr && my_planner->get_state() == SubprocessState::wait4fetch){
            my_planner->set_state(SubprocessState::wait4upload);

            if (verbose_) printf("# Find car_id %d\n", agent->getId());

            env.my_traj = ToTraj(agent);     //my_traj

            // The code below will give us the history for the other cars, 
            // which is not what we want.
            // for (auto p2 : this->agentDictionary)
            //     if (p2.first != agent)
            //         env.other_trajs.push_back(ToTraj(p2.first));
            // The correct code (below) gives us the predicted trajs from previous timestep
            auto my_pred_trajs = (this->prevPredictedTrajs).find(agent->getId());
            if (my_pred_trajs != this->prevPredictedTrajs.end()) {
                for (auto p3 : my_pred_trajs->second){
                    env.other_trajs.push_back(p3);
                }
            }

            if (verbose_) printf("# size of other_trajs: %d\n", (int)env.other_trajs.size());

            mutex.unlock();
            return env;
        }
    }
    mutex.unlock();


    for (int t = 0; t < 10; t ++){
        auto state = new core::State();
        state->track_id=0;
        state->frame_id=0;
        state->timestamp_ms=0;
        state->agent_type="233";
        state->x=0;
        state->y=0;
        state->vx=0;
        state->vy=0;
        state->psi_rad=0;
        state->length=0;
        state->width=0;

        env.my_traj.emplace_back(state);
    }

    if (simulatorState == Paused){
        env.paused = true;
        if (verbose_) printf("# simulatorState == Paused\n");
    } else {
        if (verbose_) printf("# Did not find available car\n");
    }

    return env;

}

void Simulator::upload_traj_predictor(int car_id, std::vector<core::Trajectory> pred_trajs, std::vector<double> probability){
    this->prevPredictedTrajs[car_id] = pred_trajs;
    if (car_id == 0) {
        if (verbose_) printf("# uploading traj failed, car_id = 0\n");
    }
    else {
        bool found = false;
        mutex.lock();

        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;

            if (agent->getId() == car_id){
                assert(found == false);
                if (verbose_) printf("# Find car_id %d, Upload the PredictTraj\n", car_id);

                PredictTra result;
                for (int i = 0; i < pred_trajs.size(); i ++){
                    OneTra inittraj;
                    result.Trajs.push_back(inittraj);

                    {   // put current state
                        TraPoints initpoint;
                        
                        initpoint.t = 0;
                        initpoint.x = agent->getState()[0];
                        initpoint.y = agent->getState()[1];
                        initpoint.theta = agent->getState()[2];
                        initpoint.v = std::sqrt(agent->getState()[3] * agent->getState()[3] + agent->getState()[4] * agent->getState()[4]);
                        
                        initpoint.delta_theta = 0.0;
                        initpoint.a = 0.0;
                        initpoint.jerk = 0.0;

                        auto xy2laneid_res = HelperFunction::xy2laneid(initpoint.x, initpoint.y, initpoint.theta, mapreader->map);
                        auto candidate_lanelet_id = xy2laneid_res.first;

                        initpoint.current_lanelet = mapreader->map->laneletLayer.get(candidate_lanelet_id);
                        initpoint.s_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).length;
                        initpoint.d_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).distance;

                        result.Trajs[i].Traj.push_back(initpoint);
                    }

                    for (auto state: pred_trajs[i]){
                        TraPoints initpoint;

                        //TODO: change data type from core::state to TraPoints
                        initpoint.t = 10 * SIM_TICK * (result.Trajs[i].Traj.size() + 1);  //state->timestamp_ms;
                        initpoint.x = state->x;
                        initpoint.y = state->y;
                        initpoint.theta = state->psi_rad;
                        initpoint.v = std::sqrt(state->vx * state->vx + state->vy * state->vy);

                        initpoint.delta_theta = 0.0;
                        initpoint.a = 0.0;
                        initpoint.jerk = 0.0;

                        auto xy2laneid_res = HelperFunction::xy2laneid(initpoint.x, initpoint.y, initpoint.theta, mapreader->map);
                        auto candidate_lanelet_id = xy2laneid_res.first;

                        initpoint.current_lanelet = mapreader->map->laneletLayer.get(candidate_lanelet_id);
                        initpoint.s_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).length;
                        initpoint.d_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).distance;

                        result.Trajs[i].Traj.push_back(initpoint);
                    }
                    assert(result.Trajs[i].Traj.size() == 31);

                    result.Trajs[i].Probability = probability[i];

                    for (int j = 0; j < result.Trajs[i].Traj.size(); j ++)
                        if (j == 0 || result.Trajs[i].Traj[j].current_lanelet.id() != result.Trajs[i].Traj[j-1].current_lanelet.id()){
                            for (auto &ll: mapreader->ConflictLane_[result.Trajs[i].Traj[j].current_lanelet.id()]){
                                    result.Trajs[i].confilictlanes.push_back(ll);
                            }
                        }
                }

                agent->getExPredictor()->set_traj(result);
                found = true;
            }
        }
        assert(found == true);
        mutex.unlock();
    }
}

void Simulator::upload_traj_planner(int car_id, core::Trajectory planned_traj) {
    if (car_id == 0) {
        printf("# uploading traj from planner failed, car_id = 0\n");
    }
    else {
        bool found = false;
        mutex.lock();
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;

            if (agent->getId() == car_id) {
                assert(found == false);
                printf("# Find car_id %d, Upload the PlannedTraj\n", car_id);

                // TODO - set the agent's planned trajectory here.
                std::vector<TraPoints> inittraj;

                for (auto state: planned_traj){
                    TraPoints initpoint;

                    initpoint.t = SIM_TICK * planned_traj.size();  //state->timestamp_ms;
                    initpoint.x = state->x;
                    initpoint.y = state->y;
                    initpoint.theta = state->psi_rad;
                    initpoint.v = std::sqrt(state->vx * state->vx + state->vy * state->vy);

                    initpoint.delta_theta = 0.0;
                    initpoint.a = 0.0;
                    initpoint.jerk = 0.0;

                    auto xy2laneid_res = HelperFunction::xy2laneid(initpoint.x, initpoint.y, initpoint.theta, mapreader->map);
                    auto candidate_lanelet_id = xy2laneid_res.first;

                    initpoint.current_lanelet = mapreader->map->laneletLayer.get(candidate_lanelet_id);
                    initpoint.s_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).length;
                    initpoint.d_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).distance;

                    inittraj.push_back(initpoint);
                }
                agent->getPlanner()->set_traj(inittraj);
                found = true;
                // Send results to PyPlanner
            }
        }
        assert(found == true);
        mutex.unlock();
    }
}

///
/// reset the simulator status
void Simulator::reset() {
    assert(false);  // Old Version

    /*
    printf("reset simulator ......\n");

    mutex.lock();
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; // for each agent

        for(int i = 0; i < replayAgentDictionary.size(); i++)
            if(replayAgentDictionary[i]->getId() == agent->getId()) {
                this->replayAgentDictionary.erase(replayAgentDictionary.begin() + i);
                break;
            }

        this->agentDictionary.erase(agent);
        printf("####### remove agent %d\n", agent->getId());
    }
    simulatorState = SimulatorState::Paused;
    mutex.unlock();
    */
}

vector<Agent*> Simulator::agentsForThread = vector<Agent*>();
InputDictionary Simulator::humanInputsForThread = InputDictionary();
//int Simulator::flagForVirtualCar = 0;
int Simulator::managerForVirtualCar = 0;
ifstream Simulator::infile;
int Simulator::updateTimes = 0;
double Simulator::time = 0.0;
TrafficInfoManager* Simulator::trafficInfoManagerPtr = new TrafficInfoManager(exampleMapPath);
ReplayGenerator* Simulator::ReplayGeneratorPtr = new ReplayGenerator();
vector<ReplayAgent*> Simulator::replayAgentDictionary  = vector<ReplayAgent*>();
LaneletMapReader* Simulator::mapreader = new LaneletMapReader(exampleMapPath,0.0,0.0);
bool Simulator::verbose_ = false;

//map = load(exampleMapPath, projection::UtmProjector(Origin({37.8997956297, -122.29974290381})));


//////////////////////////////////////////////////////////////////////

std::pair<int, std::string> HelperFunction::xy2laneid(double x, double y, double yaw, lanelet::LaneletMapPtr map_ptr){

    // calculate s, d, lane
    Object2d obj;
    BasicPoint2d pp(x, y);
    obj.pose.translation() = BasicPoint2d{0, 0};
    obj.pose.linear() = Eigen::Rotation2D<double>(0).matrix();
    obj.absoluteHull = matching::Hull2d{pp};
    
    const double MIN_ALLOWED_DIS = 0.0;
    const double PI = 3.1415926535;
    std::vector<LaneletMatch> Match_result = getDeterministicMatches(*map_ptr, obj, MIN_ALLOWED_DIS);
    //Find all the lanelets whose distance to pp is smaller than MIN_ALLOWED_DIS

    if (Match_result.size() == 0){
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


bool HelperFunction::inside_ob(double xx, double yy, Vector ob_state, double ob_length, double ob_width){
    return  yy > tan(ob_state[2]) * (xx-ob_state[0]) + ob_state[1] - ob_width*0.5 / abs(cos(ob_state[2]))
        &&  yy < tan(ob_state[2]) * (xx-ob_state[0]) + ob_state[1] + ob_width*0.5 / abs(cos(ob_state[2]))
        &&  yy > -1*(xx-ob_state[0])/ tan(ob_state[2]) + ob_state[1]- ob_length*0.5 /abs(sin(ob_state[2]))
        &&  yy < -1*(xx-ob_state[0])/ tan(ob_state[2]) + ob_state[1]+ ob_length*0.5 /abs(sin(ob_state[2]));
}