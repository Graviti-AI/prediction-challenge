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


char write_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";  //TODO:
char collision_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";
std::ofstream out;


#define Car_Num 3       //TODO: the number of cars generated at first

namespace {
    //std::string exampleMapPath = "/home/mscsim/ao/framework/Maps/test.osm";
    //std::string exampleMapPath = "/home/mscsim/ao/framework/Maps/with_negative_xy/DR_USA_Roundabout_SR.osm";
    
    //TODO:
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
Simulator::Simulator(int rviz_port):myThreadPool(Car_Num), total_car_num(1){
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

void Simulator::InitSimulation(std::string Config_Path){
    std::ifstream Config_ifstream;
    /// counting Num of ref points
    Config_Path_ = Config_Path;
    Config_ifstream.open(Config_Path);
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
            ReplayGeneratorPtr->loadCSV(TrackPath_);    //TODO:
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
            total_car_num = CarNumber + 1;
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
                printf("id = %d\n", id);

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
                if (temp =="Constent_Speed"){
                    getline(Config_ifstream, temp, ' ');
                    double dt = stringToNum<double>(temp);
                    getline(Config_ifstream, temp, '\n');
                    double Hor = stringToNum<double>(temp);
                    ConstantSpeedPredictor *conspre = new class ConstantSpeedPredictor(mapinfo,dt,Hor);
                    virtualCar->setPredictor(conspre);
                }
                else if (temp == "py_predictor"){
                    getline(Config_ifstream, temp, ' ');
                    getline(Config_ifstream, temp, '\n');

                    PyPredictor *py_predictor = new PyPredictor(mapinfo,0.2,2);
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
            }

            cout<<"InitState INIT!"<<endl;
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
    
    sprintf(write_file_name,"../Log/test_%s.txt", format_area);
    ofstream File_creat(write_file_name);
    File_creat.close();
    std::cout<<"Record Create! "<<write_file_name<<std::endl;

    sprintf(collision_file_name,"../Log/Collision_test_%s.txt", format_area);
    ofstream Collition_File_creat(collision_file_name);
    Collition_File_creat.close();
    std::cout<<"Collision Create! "<<collision_file_name<<std::endl;

    out.open(write_file_name,std::ios::app);
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
}


void Simulator::generateBehaveCar() {
    int id = total_car_num ++; //random() % 1000;   //TODO:
    // the initial value of tot_car_num is 1, ID 0 is for not finding.
    printf("************* New car ID: %d ***********\n", id);

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
    PyPredictor *py_predictor = new PyPredictor(mapinfo,0.2,2);

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
        Agent *agent = pair.first; // for each agent
        if (agent->hasReachedDestinaiton or (agent->getState()).size() == 0) {
        //if (agent->hasReachedDestinaiton) {
            while(agent->isRunning) {
                usleep(1e6 * SIM_TICK);
                continue;
            }
            for(int i = 0; i < replayAgentDictionary.size(); i++)
                if(replayAgentDictionary[i]->getId() == agent->getId()){
                    this->replayAgentDictionary.erase(replayAgentDictionary.begin() + i);
                    break;
                }
            this->agentDictionary.erase(agent);
            printf("# remove agent %d\n", agent->getId());
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
    int JinningCar_id = 0;
    int generate_time = 20;
    while (true)
    {   
        // /*
        //generateReplayCar();
        if (this -> updateTimes%generate_time == 10 && this->agentDictionary.size() < Car_Num) {
        //if (this->agentDictionary.size() < Car_Num){ 
            /*
            if (JinningCar_id%Car_Num == 0)
            {
                JinningCar_id = 0;
            }
            if (JinningCar_id==1){
                generate_time = 100;
            }
            else{
                generate_time = 50;
            }
            generateJinningCar_Obstacles(JinningCar_id);
            JinningCar_id++;
            */

            // generateVirtualCar();
            // generateFSMVirtualCar();
            generateBehaveCar();
            std::cout<<"\n******* add car, update times = " << updateTimes <<"******"<<endl;
        }
        
        // */
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

    std::thread manager_thread;
    manager_thread = thread(&Simulator::Agentmanager, this);
    manager_thread.detach();

    /*
    while (true) {
        // std::cout << "simulatorState " << simulatorState << std::endl; 
        usleep(1e6 * SIM_TICK); // sleep before a new iteration begins
        while(removeAgentIfNeeded()){
            // Do nothing
            // std::cout << "Curremt agent num : " << simulatorState << std::endl;
        }
        mutex.lock();
        SimulatorState simulatorState = this->simulatorState; // get the current simulator state
        mutex.unlock();

        switch (simulatorState) {
            case Running:
                gettimeofday(&t1, NULL);
                //generateReplayCar();
                //if (updateTimes%300 == 0 && this->agentDictionary.size()<25) {
                //    generateVirtualCar();
                //}
                this->updateTick(); // advance the simulation by updating all agents.
                updateTimes ++;
                gettimeofday(&t2, NULL);
                break;
            case Reset:
                this->reset();
                mutex.lock();
                this->simulatorState = Running;
                mutex.unlock();
                break;
            case Paused:
                break;
        }
        time = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0;
        this->timeuse += time;
        if (this->timeuse < SIM_TICK) {
            Simulator::flagForVirtualCar = 0;
        }
        else {
            Simulator::flagForVirtualCar = 1;
            this->timeuse = 0;
        }
        //printf ("It took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);
    }
    */
}

core::Trajectory Simulator::ToTraj(Agent* agent){
    auto prestate = agent->get_preState();
    auto curstate = agent->getState();

    core::Trajectory traj;

    Vector laststate;
    for (int t = 0; t < 10; t++){
        if (t == 0){
            laststate = curstate;
        }
        else if (prestate.size() >= 1){
            laststate = prestate[max(0, int(prestate.size()) - t)];
        }

        auto state = new core::State();
        state->track_id=agent->getId();
        state->frame_id=max(0, int(prestate.size()) - t);
        state->timestamp_ms=state->frame_id * 100;
        state->agent_type="car";

        state->x=laststate[0];      
        state->y=laststate[1];
        state->vx=laststate[3];
        state->vy=laststate[4];
        state->psi_rad=laststate[2];
        state->length=agent->length_;
        state->width=agent->width_;
        //TODO:

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

    bool isRunning = false;
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;

        if (agent->isRunning == true){
            isRunning = true;
            break;
        }
    }

    if (! isRunning){
        mutex.lock();
        for (auto pair : this->agentDictionary) {
            Agent *agent = pair.first; 

            if (agent->getPredictor()->get_state() == 0){
                agent->getPredictor()->set_state(1);

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

    printf("# Did not find available car\n");
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
    return env;
}


void Simulator::upload_traj(int car_id, std::vector<core::Trajectory> pred_trajs, std::vector<double> probability){
    if (car_id == 0) {
        printf("# uploading traj failed, car_id = 0\n");

        if (this->agentDictionary.size() == 0)
            this->updateTimes ++;

        return;
    }

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

                    initpoint.t = state->timestamp_ms;
                    initpoint.x = state->x;
                    initpoint.y = state->y;
                    initpoint.theta = state->psi_rad;
                    initpoint.v = std::sqrt(state->vx * state->vx + state->vy * state->vy);
                    //TODO:

                    result.Trajs[i].Traj.push_back(initpoint);
                }
                result.Trajs[i].Probability = probability[i];
            }

            agent->getPredictor()->set_client_traj(result);
            found = true;
        }
    }
    assert(found == true);
    mutex.unlock();

    ////////////// check whether all the predictors have received Trajs.
    mutex.lock();

    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; 

        if (agent->getPredictor()->get_state() != 2){
            mutex.unlock();
            return;
        }
    }
    /////////////////////////// Tick()
    //COPY FROM Simulator::run()

    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;
        agent->getPredictor()->set_state(3);
    }
    mutex.unlock();

    printf("\nTick.....\n");

    while(removeAgentIfNeeded()){
        // Do nothing
        // std::cout << "Curremt agent num : " << simulatorState << std::endl;
    }
    
    SimulatorState simulatorState = this->simulatorState; // get the current simulator state
    
    if (updateTimes==MaxUpdateTimes_) {
        cout<<"finish this simu"<<endl;
        mutex.lock();
        this->simulatorState = Paused;
        mutex.unlock();
    }

    switch (simulatorState) {
        case Running:
            gettimeofday(&t1, NULL);
            //generateReplayCar();
            //if (updateTimes%300 == 0 && this->agentDictionary.size()<25) {
            //    generateVirtualCar();
            //}
            //isThereCollision();
            this->updateTick(); // advance the simulation by updating all agents.
            updateTimes ++;
            gettimeofday(&t2, NULL);
            break;
        case Reset:
            this->reset();
            mutex.lock();
            this->simulatorState = Running;
            mutex.unlock();
            break;
        case Paused:
            break;
    }

    //isThereCollision();
    time = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0;
    this->timeuse += time;
    if (this->timeuse < SIM_TICK) { //TODO: 0.01
        Simulator::flagForVirtualCar = 0;
    }
    else {
        Simulator::flagForVirtualCar = 1;
        this->timeuse = 0;
    }

    //usleep(1e6 * SIM_TICK); // sleep before a new iteration begins
}


///
/// Record the agents states. Update the agents in the simulation. This is called for each iteration step.
void Simulator::updateTick() {
    mutex.lock();
    // Update traffic info
    Simulator::trafficInfoManagerPtr->Update();

    vector<Agent*> agents;
    //Record data of cars in file "/home/mscsim/state_write_test/test_[CURRENT_TIME].txt";
    int num = 0;

    std::string writebuf = to_string(updateTimes)+"\n";
    for (auto pair : this->agentDictionary) {
        agents.push_back(pair.first); // get pointers to all agents
        int i;
        Vector vehstate;
        if (pair.first->getType() != AgentType::RealCar) {
            i = pair.first->getId();
            vehstate = pair.first->getState();
            num++;
            writebuf += std::to_string(i) + ',' + std::to_string(vehstate[0]) + ',' + std::to_string(vehstate[1]) + ',' +
                        std::to_string(vehstate[2]) + ',' + std::to_string(vehstate[3]) + ',' + std::to_string(vehstate[4]) 
                        +',' + std::to_string(vehstate[5]) + ','  + std::to_string(pair.first->length_) + ',' + std::to_string(pair.first->width_) + ','+ std::to_string(pair.first->mapinfo->getCurrentLaneletId())  + '\n';
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

    /*
    out.open("/home/mscsim/mkz-mpc-control/b.txt",std::ios::trunc|std::ios::out);
    if(out.is_open()) {
        out << writebuf;
        out.close();
    }
    else
    {
        cout <<"no such file" << endl;
    }
    */

    humanInputsForThread = humanInputs;
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
    std::chrono::time_point<std::chrono::system_clock> in_time = std::chrono::system_clock::now();

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
    agentsForThread = agents;
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
            continue;
        }
        this->myThreadPool.AddTask(agent, 10); // multiThreads
    }

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
    std::chrono::time_point<std::chrono::system_clock> out_time = std::chrono::system_clock::now();
    //cout<<"before calculate time: "<< double (std::chrono::duration_cast<std::chrono::milliseconds>(out_time - in_time).count()) << " ms"<<endl;
    /*
    for (Agent *agent : agents) {
    agent->applyNextState(); // Apply the next state of the agent.
    }*/
    mutex.unlock();
}

///
/// reset the simulator status
void Simulator::reset() {

}

vector<Agent*> Simulator::agentsForThread = vector<Agent*>();
InputDictionary Simulator::humanInputsForThread = InputDictionary();
int Simulator::flagForVirtualCar = 0;
int Simulator::managerForVirtualCar = 0;
ifstream Simulator::infile;
int Simulator::updateTimes = 0;
double Simulator::time = 0.0;
TrafficInfoManager* Simulator::trafficInfoManagerPtr = new TrafficInfoManager(exampleMapPath);
ReplayGenerator* Simulator::ReplayGeneratorPtr = new ReplayGenerator(); 
vector<ReplayAgent*> Simulator::replayAgentDictionary  = vector<ReplayAgent*>();
LaneletMapReader* Simulator::mapreader = new LaneletMapReader(exampleMapPath,0.0,0.0);
//map = load(exampleMapPath, projection::UtmProjector(Origin({37.8997956297, -122.29974290381})));


