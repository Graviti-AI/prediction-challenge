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
        printf("Waiting rviz on %d ......\n", rviz_port);
    }
}

void Simulator::InitSimulation(std::string scenario_id, std::string Config_Path, std::string log_folder){
    std::ifstream Config_ifstream;
    /// counting Num of ref points
    Config_Path_ = Config_Path;
    Config_ifstream.open(Config_Path);

    if (! Config_ifstream.good()){
        throw std::runtime_error("Error: The congfig file is not exsited!");
    }

    std::string temp;
    int CarNumber = 0;
    bool mapreaded_= false;
    while(getline(Config_ifstream, temp, ':')){
        if (temp == "Map"){
            getline(Config_ifstream, temp, '\n');
            MapName_ = temp;
            std::string MapPath_="../core/my_impl/Maps/with_negative_xy/";
            MapPath_+=MapName_;
            MapPath_+=".osm";
            TrafficInfoManager* trafficInfoManagerPtr = new TrafficInfoManager(MapPath_);
            LaneletMapReader* mapreader = new LaneletMapReader(MapPath_,0.0,0.0);
            mapreaded_=true;
            cout<<"EndLaneletIds: "<<mapreader->EndLaneletIds.size()<<" StartLaneletIds: "<<mapreader->StartLaneletIds.size()<<endl;
            cout<<"MAP INIT! "<<MapPath_<<endl;
        }
        else if (temp == "Track"){
            getline(Config_ifstream, temp, '\n');
            std::string TrackPath_="../core/my_impl/CSV/";
            TrackPath_+=MapName_;
            TrackPath_+="/vehicle_tracks_";
            TrackPath_+=temp;
            TrackPath_+=".csv";
            ReplayGeneratorPtr->loadCSV(TrackPath_);
            cout<<"TRACK INIT! "<<TrackPath_<<endl;
        }
        else if (temp == "MaxUpdateTimes"){
            getline(Config_ifstream, temp, '\n');
            MaxUpdateTimes_=stringToNum<int >(temp);
            cout<<"MaxUpdateTimes INIT! "<<MaxUpdateTimes_<<endl;
        }
        else if(temp == "CarNum"){
            getline(Config_ifstream, temp, '\n');
            CarNumber=stringToNum<int >(temp);
            cout<<"CarNum INIT! "<<CarNumber<<endl;
            //total_car_num = CarNumber + 1;
        }
        else if(temp == "InitState"){
            if (CarNumber==0 || (!mapreaded_)) {
                throw std::runtime_error("Bad Congfig File! CarNum and Map should be loaded before initState");
                break;
            }
            getline(Config_ifstream, temp, '\n');
            for (int j = 0; j < CarNumber; ++j) {
                getline(Config_ifstream, temp, ' ');
                int id = stringToNum<int >(temp);

                Vector initstate(6);
                for(int k = 0;k<6;k++){
                    getline(Config_ifstream, temp, ' ');
                    initstate[k]= stringToNum<double >(temp);
                }
                class BehaveCar *virtualCar = new class BehaveCar(id, initstate); 
                printf("\nAdd BehaveCar, id = %d\n", id);

                getline(Config_ifstream, temp, ' ');
                virtualCar->length_= stringToNum<double >(temp);
                getline(Config_ifstream, temp, ' ');
                virtualCar->width_= stringToNum<double >(temp);

                getline(Config_ifstream, temp, ' ');
                int startLaneletId= stringToNum<int >(temp);
                getline(Config_ifstream, temp, ' ');
                int endLaneletId= stringToNum<int >(temp);
                MapInfo* mapinfo = new MapInfo(mapreader->map, mapreader->routingGraph);
                ConstLanelet fromLanelet = mapreader->map->laneletLayer.get(startLaneletId);
                ConstLanelet toLanelet = mapreader->map->laneletLayer.get(endLaneletId);
                if (!mapinfo->setRoutingPath(fromLanelet, toLanelet)) 
                    throw std::runtime_error("Bad Congfig File! No route exist for "+std::to_string(startLaneletId)+" and "+std::to_string(endLaneletId));
                mapinfo->init(id,initstate);
                getline(Config_ifstream, temp, ' ');
                if (temp =="IDM"){
                    AoBehaviour *aobehave = new class AoBehaviour(BehaviourType::IDM);
                    aobehave -> mapinfo_=mapinfo;
                    //ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,0.2,2);
                    virtualCar->setBehaviour(aobehave);
                    virtualCar->setMapinfo(mapinfo);
                    //virtualCar->setPredictor(conspre);
                    virtualCar->IDM_ = true;
                    getline(Config_ifstream, temp, ' ');
                }
                else if (temp =="Astar"){
                    AoBehaviour *aobehave = new class AoBehaviour(BehaviourType::IDM);
                    aobehave -> mapinfo_=mapinfo;
                    //ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,0.2,2);
                    virtualCar->setBehaviour(aobehave);
                    virtualCar->setMapinfo(mapinfo);
                    //virtualCar->setPredictor(conspre);
                    virtualCar->setfollowingPlanner(new AstarPlanner(mapinfo)); // new CILQRPlanner
                    virtualCar->IDM_ = false;
                    getline(Config_ifstream, temp, ' ');
                }
                else if (temp =="EB"){
                    AoBehaviour *aobehave = new class AoBehaviour(BehaviourType::IDM);
                    aobehave -> mapinfo_=mapinfo;
                    //ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,0.2,2);
                    virtualCar->setBehaviour(aobehave);
                    virtualCar->setMapinfo(mapinfo);
                    //virtualCar->setPredictor(conspre);
                    virtualCar->setfollowingPlanner(new EBPlanner(mapinfo)); // new CILQRPlanner
                    virtualCar->IDM_ = false;
                    getline(Config_ifstream, temp, ' ');
                }
                else throw std::runtime_error("Bad Planner");

                getline(Config_ifstream, temp, ' ');
                if (temp =="Constant_Speed"){
                    getline(Config_ifstream, temp, ' ');
                    double dt = stringToNum<double>(temp);
                    getline(Config_ifstream, temp, '\n');
                    double Hor = stringToNum<double>(temp);
                    ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(virtualCar,dt,Hor);
                    virtualCar->setPredictor(conspre);
                }
                else if (temp == "py_predictor"){
                    getline(Config_ifstream, temp, ' ');
                    double dt = stringToNum<double>(temp);
                    getline(Config_ifstream, temp, '\n');
                    double Hor = stringToNum<double>(temp);

                    PyPredictor *py_predictor = new PyPredictor(virtualCar,dt,Hor);
                    virtualCar->setPredictor(py_predictor);
                }
                else throw std::runtime_error("Bad Predictor");

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
                printf("Add to agentDictionary\n\n");
            }

            cout<<"InitState INIT!"<<endl;
        }
        else if (temp == "ReplayStartTimestamp(ms)"){ // deal with replay cars
            getline(Config_ifstream, temp, '\n');
            int ReplayStartTimestamp_ms = stringToNum<int>(temp);

            if (ReplayStartTimestamp_ms < 0){
                throw std::runtime_error("ReplayStartTimestamp < 0, no replay car survived\n");
            }
            // each tick in the simulator is 0.01s (10ms)
            int ReplayEndTimestamp_ms = MaxUpdateTimes_ * 10 + ReplayStartTimestamp_ms;

            auto replay_info_pool = ReplayGeneratorPtr->filter_replay_car(ReplayStartTimestamp_ms, ReplayEndTimestamp_ms);
            printf("ReplayStartTimestamp_ms: %d, ReplayEndTimestamp_ms: %d, replay_info_pool_size: %d\n", ReplayStartTimestamp_ms, ReplayEndTimestamp_ms, int(replay_info_pool.size()));

            // remove robot car from replay list
            for (int i = 0; i < replay_info_pool.size(); ){
                int track_id = replay_info_pool[i][0];
                bool is_robot_car = false;

                for (auto pair : agentDictionary) {
                    auto agent = pair.first;
                    if (agent->getId() == track_id){
                        printf("track_id (%d) is a robot car, remove from the replay list\n", track_id);
                        is_robot_car = true;
                        break;
                    }
                }

                if (is_robot_car){
                    replay_info_pool.erase(replay_info_pool.begin() + i);
                }
                else i++;
            }
            printf("After removing robot car, replay_info_pool_size: %d\n", int(replay_info_pool.size()));

            getline(Config_ifstream, temp, ':');
            assert(temp == "ReplayCarWithPredictor");

            getline(Config_ifstream, temp, '\n');
            int ReplayCarWithPredictor = stringToNum<int>(temp);
            printf("ReplayCarWithPredictor INIT: %d\n", ReplayCarWithPredictor);

            getline(Config_ifstream, temp, '\n');
            assert(temp == "track_id,Predictor,Predictor.dt,Predictor.horizon");
            
            // read information for replay car(with predictor)
            for (int j = 0; j < ReplayCarWithPredictor; j ++){
                getline(Config_ifstream, temp, ' ');
                int track_id = stringToNum<int >(temp);

                getline(Config_ifstream, temp, ' ');
                string predictor_type = temp;

                getline(Config_ifstream, temp, ' ');
                double dt = stringToNum<double>(temp);

                getline(Config_ifstream, temp, '\n');
                double Hor = stringToNum<double>(temp);
                
                std::vector<int> info;
                for (int i = 0; i < replay_info_pool.size(); i ++)
                    if (replay_info_pool[i][0] == track_id) {
                        info = replay_info_pool[i];
                        replay_info_pool.erase(replay_info_pool.begin() + i);
                        break;
                    }
                
                if (info.size() != 3){
                    printf("ERROR: cannot find replay car (%d) in replay list\n", track_id);
                    throw std::runtime_error("cannot find replay car in replay list\n");
                }

                ReplayInfo replay_info = std::make_tuple(info[0], info[1], info[2], predictor_type, dt, Hor);
                assert((info[1] - ReplayStartTimestamp_ms) % 10 == 0);
                
                // the update time in which this replay car will occur is (info[1] - ReplayStartTimestamp_ms) / 10
                ReplayCarWaitList[(info[1] - ReplayStartTimestamp_ms) / 10].push_back(replay_info);

                printf("Add Replay Car (%d, %d, %d, %s, %.2lf, %.2lf) to WaitList\n", std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info), std::get<3>(replay_info).c_str(), std::get<4>(replay_info), std::get<5>(replay_info));
            }

            // add the remaining replay cars to wait list
            for (int i = 0; i < replay_info_pool.size(); i ++){
                auto info = replay_info_pool[i];
                ReplayInfo replay_info = std::make_tuple(info[0], info[1], info[2], "", -1, -1);
                
                assert((info[1] - ReplayStartTimestamp_ms) % 10 == 0);
                ReplayCarWaitList[(info[1] - ReplayStartTimestamp_ms) / 10].push_back(replay_info);

                printf("Add Replay Car (%d, %d, %d, %s, %.2lf, %.2lf) to WaitList\n", std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info), std::get<3>(replay_info).c_str(), std::get<4>(replay_info), std::get<5>(replay_info));
            }
        }
        else throw std::runtime_error("Bad Label in Congfig File");        
    } 
    Config_ifstream.close();

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
    std::cout<<"Record Create! "<<write_file_name<<std::endl;

    sprintf(collision_file_name,"%s/scenario%s_Collision_test_%s.txt", log_folder.c_str(), scenario_id.c_str(), format_area);
    ofstream Collition_File_creat(collision_file_name);
    Collition_File_creat.close();
    std::cout<<"Collision Create! "<<collision_file_name<<std::endl;

    out.open(write_file_name,std::ios::app);
    if(out.is_open()) {
        out <<"Simulation Begin Time:"<<format_area<<endl;
        out <<"Config Path: ";
        out << Config_Path_<<endl;
        out << "id,x,y,yaw,v_lon,v_lat,psi_rad,length,width,lane_id,centerline,agent_type\n";
        out.close();
    }
    else
    {
        std::cout << "no such file" << std::endl;
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
    }

    mutex.lock();
    simulatorState = SimulatorState::Running;
    mutex.unlock();
}


void Simulator::generateReplayCar(ReplayInfo replay_info) {
    //int car_id = total_car_num ++;
    printf("\nNew Replay Car Generated, (%d, %d, %d, %s, %.2lf, %.2lf)\n", std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info), std::get<3>(replay_info).c_str(), std::get<4>(replay_info), std::get<5>(replay_info));

    ReplayAgent* virtualCar = ReplayGeneratorPtr->generateReplayAgent(std::get<0>(replay_info), std::get<1>(replay_info), std::get<2>(replay_info));
    MapInfo* mapinfo = new MapInfo(mapreader->map, mapreader->routingGraph);
    ConstLanelets lanelet_path;

    for (auto t : virtualCar->getTrajectory().second){
        auto x = t.second[0], y = t.second[1], yaw = t.second[2];
        auto xy2laneid_res = HelperFunction::xy2laneid(x, y, yaw, mapreader->map);
        
        if (xy2laneid_res.second == "matches"){
            auto candidate_lanelet_id = xy2laneid_res.first;
            ConstLanelet candidate_lanelet = mapreader->map->laneletLayer.get(candidate_lanelet_id);
            
            if (lanelet_path.empty() || lanelet_path.back().id() != candidate_lanelet_id)
                lanelet_path.push_back(candidate_lanelet);
        }
        
        /* //For debug
        printf("x: %.3lf, y: %.3lf, yaw: %.3lf, lanelet_id: %d, yaw_gap: %.3lf\n", x, y, yaw, min_yaw_gap_lanelet_id, min_yaw_gap);

        auto following_lanelets = mapreader->routingGraph->following(min_yaw_gap_lanelet);
        for (auto ll : following_lanelets){
            printf("%d ", int(ll.id()));
        }
        printf("\n");
        */
   }
   
   mapinfo->setLaneletPath(lanelet_path);

    Vector tmpState = virtualCar->Update();
    assert(! tmpState.empty());

    Vector nextState = virtualCar->getState();
    for (int i = 0;i<6;i++) {
        nextState[i] = tmpState[i];
    }

    virtualCar->setState(nextState);
    mapinfo->init(std::get<0>(replay_info), nextState);
    virtualCar->setMapinfo(mapinfo);

    if (std::get<string>(replay_info) == "Constant_Speed"){
        ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(virtualCar, std::get<4>(replay_info), std::get<5>(replay_info));
        virtualCar->setPredictor(conspre);
    }
    else if (std::get<string>(replay_info) == "py_predictor"){
        PyPredictor *py_predictor = new PyPredictor(virtualCar, std::get<4>(replay_info), std::get<5>(replay_info));
        virtualCar->setPredictor(py_predictor);
    }
    else{
        assert(virtualCar->getPredictor() == nullptr);
    }

    std::tuple<Controller*, Model*, Planner*> temp(
            new VirtualCarController(), // create corresponding controller
            new VirtualCarModel(), // create corresponding model
            new AstarPlanner(mapinfo) // create corresponding plannerh
    );

    auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(virtualCar, temp);

    if (virtualCar->getPredictor() != nullptr){
        printf("Add into agentDictionary\n");

        mutex.lock();
        this->agentDictionary.insert(pair); // add the car to the simulator
        this->humanInputs.insert(std::pair<Agent*, Vector>(virtualCar, Vector(3))); // add the car to the simulator
        mutex.unlock();
    }
    else {
        printf("Add into replayAgentDictionary\n");

        mutex.lock();
        replayAgentDictionary.push_back(virtualCar);
        mutex.unlock();
    }
}

void Simulator::generateBehaveCar() {
    assert(false);  // this function is obseleted

    int id = random() % 1000; //total_car_num ++;
    // the initial value of tot_car_num is 1, ID 0 is for not finding.
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
    /*// for CILQR planner
    ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,0.2,5);
    CILQRPlanner * p = new CILQRPlanner(mapinfo);
    p->setId(id); // Car Id
    virtualCar->setfollowingPlanner(p);
    virtualCar->setlinechangePlanner(p);*/

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

    printf("\n");
	for (auto pair : this->agentDictionary){
		Agent* agent = pair.first;
		xx = agent->getState()[0];
		yy = agent->getState()[1];
		angle = agent->getState()[2];
		length = agent->length_;
		width = agent->width_;
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
				ob_length = agen->length_;
				ob_width = agen->width_;
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
					cout<<"There is a collision between car "<<agent->getId()<<" and car "<<agen->getId()<<endl;
                    out.open(collision_file_name,std::ios::app);
                    if(out.is_open()) {
                        out <<to_string(updateTimes)<<" "<<"collision between cars(CarID CarID):"<<to_string(agent->getId())<<" "<<to_string(agen->getId())<<endl;
                        out.close();
                    }
                    else
                    {
                        std::cout << "no such file" << std::endl;
                    }
				}
			}
         
		}
        if(numb == 0){
            cout<<"There are no collisions between cars!"<<endl;;
        }

		Agent* tmpAgent = agent;
        MapInfo* this_map_info = agent->mapinfo;
		ConstLanelet tmpLl = this_map_info->getCurrentLanelet();
		ConstLineString2d LEFT = tmpLl.leftBound2d();
		ConstLineString2d RIGHT = tmpLl.rightBound2d();
		if(collisionWithLane(x,y,LEFT,RIGHT,xx,yy,thre) == 1){
		    cout<<"The car "<<agent->getId()<<" collides with the lane "<<this_map_info->getCurrentLanelet().id()<<"!"<<endl;
            out.open(collision_file_name,std::ios::app);
            if(out.is_open()) {
                out <<to_string(updateTimes)<<" "<<"collision with lane(CarID laneID):"<<to_string(agent->getId())<<" "<<to_string(this_map_info->getCurrentLanelet().id())<<endl;
                out.close();
            }
            else
            {
                std::cout << "no such file" << std::endl;
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
            printf("# Remove Agent (%d) From agentDictionary \n", agent->getId());
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
            printf("# Remove Agent (%d) From replayAgentDictionary \n", agent->getId());
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
    //int JinningCar_id = 0;
    //int generate_time = 1;

    assert(false);  // this function is obseleted
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
        /*
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
        */
    }
}

///
/// Start the simulator. This is an infinite loop.
void Simulator::run() {
    printf("Simulator::run begins!\n");

    // set initial state for the robot cars
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; 
        agent->getPredictor()->set_state(PredictorState::wait4update);
    }

    while (true){
        // check all the cars have finished updates
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first; 
            auto predictor = agent->getPredictor();

            while(agent->isRunning || predictor->get_state() != PredictorState::wait4update){
                usleep(1e6 * SIM_TICK);
            }
        }
        for (int i = 0; i < replayAgentDictionary.size(); i++){
            Agent* agent = replayAgentDictionary[i];

            while(agent->isRunning){
                usleep(1e6 * SIM_TICK);
            }
        }

        //set predictor_state as fine, and prepare for a tick
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first;
            auto predictor = agent->getPredictor();

            assert(predictor->get_state() == PredictorState::wait4update);
            predictor->set_state(PredictorState::fine);
        }

        // one update has finished, remove the unnecessary cars
        while (removeAgentIfNeeded()){}

        // add new cars at this `updateTimes`
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

        //Log this moment
        LogTick();
        isThereCollision();

        // set pause when no agent existed
        if (this->agentDictionary.size() == 0){
            mutex.lock();
            simulatorState = Paused;
            mutex.unlock();
            printf("\nagentDictionary size = 0, set simulatorState as Paused!\n");
        }

        // check whether updateTimes==MaxUpdateTimes_
        if (updateTimes==MaxUpdateTimes_) {
            mutex.lock();
            simulatorState = Paused;
            mutex.unlock();
            printf("\nupdateTimes == MaxUpdateTimes_, set simulatorState as Paused!\n");
        }

        if (simulatorState == Paused){
            printf("wait for the client to close the simulator...\n");
            usleep(1e6 * 3);

            printf("\nNo client, closed by simulator itself\n");
            exit(0);
        }

        // A New Tick
        printf("\n# UpdateTime: %d, agentDictionary size: %d, replayAgentDictionary size: %d\n", updateTimes, int(agentDictionary.size()), int(replayAgentDictionary.size()));
        printf("# Tick.....\n\n");

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
    std::string writebuf = to_string(updateTimes)+"\n";

    std::vector<Agent*> agent_for_log;
    for (auto pair : this->agentDictionary)
        agent_for_log.push_back(pair.first);
    
    for (int i = 0; i < replayAgentDictionary.size(); i ++)
        agent_for_log.push_back(replayAgentDictionary[i]);
    
    for (auto agent : agent_for_log){
        Vector vehstate;
        if (agent->getType() != AgentType::RealCar) {
            int i = agent->getId();
            vehstate = agent->getState();

            //calc center_line_direction
            auto currentlanelet = agent->mapinfo->getCurrentLanelet();
            double s_now = agent->mapinfo->getS();

            if (abs(s_now - geometry::toArcCoordinates(currentlanelet.centerline2d(), BasicPoint2d(vehstate[0], vehstate[1])).length) > 1e-3){
                printf("#### DEBUG | s_now: %.3lf, toArcCoordinates: %.3lf\n", s_now, geometry::toArcCoordinates(currentlanelet.centerline2d(), BasicPoint2d(vehstate[0], vehstate[1])).length);
                assert(false);
            }

            auto p_now = geometry::interpolatedPointAtDistance(currentlanelet.centerline2d(), s_now);
            auto p_after = geometry::interpolatedPointAtDistance(currentlanelet.centerline2d(), s_now + 0.01);
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

            writebuf += std::to_string(i) + ',' + std::to_string(vehstate[0]) + ',' + std::to_string(vehstate[1]) + ',' +
                        std::to_string(vehstate[2]) + ',' + std::to_string(vehstate[3]) + ',' + std::to_string(vehstate[4]) 
                        +',' + std::to_string(vehstate[5]) + ','  + std::to_string(agent->length_) + ',' + std::to_string(agent->width_) 
                        + ','+ std::to_string(agent->mapinfo->getCurrentLaneletId()) + ',' + std::to_string(center_line_direction)
                        + ',' + agent_type_str + '\n';
        }
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
        //for(auto one : agent->PredictTra_.Trajs)
        agentinfo->PredictTra_.Trajs.swap(agent->PredictTra_.Trajs);
        agents.push_back(agentinfo);
    }

    // remove agent if needed
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

    for(int i = 0; i < replayAgentDictionary.size(); i++){
        Agent* agent = replayAgentDictionary[i];

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
    assert(agent->getPredictor()->get_state() != PredictorState::fine);
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
        state->vx=laststate[3];
        state->vy=laststate[4];
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
    printf("\n### Fetch Env: There are %d cars now\n", int(this->agentDictionary.size()));
    core::SimulationEnv env;
    env.paused = false;

    bool isfine = false;
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;

        if (agent->getPredictor()->get_state() == PredictorState::fine){
            // fine means this car's nextstate has not been applied
            isfine = true;
            break;
        }
    }

    if (! isfine){
        mutex.lock();
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first; 

            if (agent->getPredictor()->get_state() == PredictorState::wait4fetch){
                agent->getPredictor()->set_state(PredictorState::wait4upload);

                printf("# Find car_id %d, historical length = %d\n", agent->getId(), int(agent->get_preState().size()));
                
                env.map_name = MapName_;        //map
                env.my_traj = ToTraj(agent);     //my_traj

                for (auto p2 : this->agentDictionary)
                    if (p2.first != agent)
                        env.other_trajs.push_back(ToTraj(p2.first));
                
                printf("# size of other_trajs: %d\n", (int)env.other_trajs.size());

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
        printf("# simulatorState == Paused\n");   
    }
    else{
        printf("# Did not find available car\n");
    }
    
    return env;
}


void Simulator::upload_traj(int car_id, std::vector<core::Trajectory> pred_trajs, std::vector<double> probability){
    if (car_id == 0) {
        printf("# uploading traj failed, car_id = 0\n");
    }
    else {
        bool found = false;
        mutex.lock();
        
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first; 

            if (agent->getId() == car_id){
                assert(found == false);
                printf("# Find car_id %d, Upload the PredictTraj\n", car_id);

                PredictTra result;
                for (int i = 0; i < pred_trajs.size(); i ++){
                    OneTra inittraj;
                    result.Trajs.push_back(inittraj);

                    for (auto state: pred_trajs[i]){
                        TraPoints initpoint;

                        //TODO: change data type from core::state to TraPoints
                        initpoint.t = SIM_TICK * result.Trajs[i].Traj.size();  //state->timestamp_ms;
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

                    result.Trajs[i].Probability = probability[i];

                    for (int j = 0; j < result.Trajs[i].Traj.size(); j ++)
                        if (j == 0 || result.Trajs[i].Traj[j].current_lanelet.id() != result.Trajs[i].Traj[j-1].current_lanelet.id()){
                            for (auto &ll: mapreader->ConflictLane_[result.Trajs[i].Traj[j].current_lanelet.id()]){
                                    result.Trajs[i].confilictlanes.push_back(ll);
                            }
                        }
                }

                agent->getPredictor()->set_traj(result);
                found = true;
            }
        }
        assert(found == true);
        mutex.unlock();
    }
}


///
/// reset the simulator status
void Simulator::reset() {
    assert(false);  // this function is obseleted
    printf("reset simulator ......\n");

    mutex.lock();
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; // for each agent

        for(int i = 0; i < replayAgentDictionary.size(); i++)
            if(replayAgentDictionary[i]->getId() == agent->getId()){
                this->replayAgentDictionary.erase(replayAgentDictionary.begin() + i);
                break;
            }
        
        this->agentDictionary.erase(agent);
        printf("####### remove agent %d\n", agent->getId());
    }
    simulatorState = SimulatorState::Paused;
    mutex.unlock();
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
//map = load(exampleMapPath, projection::UtmProjector(Origin({37.8997956297, -122.29974290381})));


//////////////////////////////////////////////////////////////////////

std::pair<int, std::string> HelperFunction::xy2laneid(double x, double y, double yaw, lanelet::LaneletMapPtr map_ptr){
    
    // calculate s, d, lane
    Object2d obj;
    BasicPoint2d pp(x, y);
    obj.absoluteHull = matching::Hull2d{pp};
    
    double MIN_ALLOWED_DIS = 0.0;
    std::vector<LaneletMatch> Match_result = getDeterministicMatches(*map_ptr, obj, MIN_ALLOWED_DIS);
    //Find all the lanelets whose distance to pp is smaller than MIN_ALLOWED_DIS

    if (Match_result.size() == 0){
        int min_dis_lanelet_id = -1;
        double min_dis = 1e10;

        for (auto tmplanelet : map_ptr->laneletLayer){
            ConstLanelet lanelet = map_ptr->laneletLayer.get(tmplanelet.id());
            auto centerline = lanelet.centerline2d();
            double dis = abs(geometry::toArcCoordinates(centerline, BasicPoint2d(x, y)).distance);

            if (min_dis_lanelet_id == -1 || dis < min_dis){
                min_dis_lanelet_id = tmplanelet.id();
                min_dis = dis;
            }
        }
        assert(min_dis_lanelet_id != -1);
        return std::make_pair(min_dis_lanelet_id, "closest_lane");
    }
    else {
        int min_yaw_gap_lanelet_id = -1;
        double min_yaw_gap = 1e10;

        // Find the lanelets whose direction is the closest to the yaw_angle 
        for (auto one_match: Match_result){
            assert(one_match.distance <= MIN_ALLOWED_DIS);
            //if (one_match.lanelet.inverted()) continue;

            ConstLanelet lanelet = map_ptr->laneletLayer.get(one_match.lanelet.id());
            auto centerline = lanelet.centerline2d();

            double s_now = geometry::toArcCoordinates(centerline, BasicPoint2d(x, y)).length;
            BasicPoint2d pinit = geometry::interpolatedPointAtDistance(centerline, s_now);
            BasicPoint2d pinit_f = geometry::interpolatedPointAtDistance(centerline, s_now + 0.01);
            BasicPoint2d pDirection = pinit_f - pinit;
            double direction = std::atan2(pDirection.y(),pDirection.x());

            if (min_yaw_gap_lanelet_id == -1 || abs(direction - yaw) < min_yaw_gap){
                min_yaw_gap = abs(direction - yaw);
                min_yaw_gap_lanelet_id = one_match.lanelet.id();
            }
        }
        assert(min_yaw_gap_lanelet_id != -1);
        return std::make_pair(min_yaw_gap_lanelet_id, "matches");
    }
    assert(false);
}
