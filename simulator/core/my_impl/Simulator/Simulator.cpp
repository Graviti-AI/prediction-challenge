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
#include "../Behaviours/FSM.hpp"
#include "../Behaviours/AoBehaviour.hpp"
#include "../Predictors/Predictor.hpp"

#include <stdio.h>      /* printf */
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>


// char write_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";
// std::ofstream out;

#define Car_Num 1       //TODO: the number of cars generated at first

namespace {
    //std::string exampleMapPath = "/home/mscsim/ao/framework/Maps/test.osm";
    //std::string exampleMapPath = "/home/mscsim/ao/framework/Maps/with_negative_xy/DR_USA_Roundabout_SR.osm";
    std::string exampleMapPath = "../core/my_impl/Maps/with_negative_xy/DR_USA_Intersection_MA.osm";
    // DR_CHN_Merging_ZS.osm    DR_CHN_Roundabout_LN.osm    DR_DEU_Merging_MT.osm   DR_DEU_Roundabout_OF.osm
    // DR_USA_Intersection_EP0.osm  DR_USA_Intersection_EP1.osm DR_USA_Intersection_GL.osm  DR_USA_Intersection_MA.osm
    // DR_USA_Roundabout_EP.osm DR_USA_Roundabout_FT.osm    DR_USA_Roundabout_SR.osm
}

using namespace std;
using namespace lanelet;
using namespace lanelet::matching;

///
/// \param simulatorState reference to the simulator state, an enumerator.
/// \param humanInputs reference to a map from agent to pertaining vector human input.
/// \param agentDictionary reference to a map from agent to pertaining controller, model, and planner.
/// \param mutex reference to a global mutex lock, which avoids thread conflicts.
Simulator::Simulator():myThreadPool(10){
    simulatorState = Running; // will be shared by simulator thread and server thread

    //generateVirtualCar();
    //ReplayGeneratorPtr->loadCSV("/home/sunyaofeng/Desktop/CSV/DR_USA_Intersection_MA/vehicle_tracks_001.csv");
    //cout<<"EndLaneletIds: "<<mapreader->EndLaneletIds.size()<<" StartLaneletIds: "<<mapreader->StartLaneletIds.size()<<endl;
    CILQR_car_flag = false;//true; 
}

void Simulator::generateBehaveCar() {
    int id = 0; //random() % 1000;   //TODO:
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
        mutex.lock();
        std::vector<Agent *> agents = Simulator::agentsForThread;
        mutex.unlock();
        tmpAgent = nullptr;
        for (auto agent : agents) {
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

    virtualCar->setBehaviour(aobehave);
    virtualCar->setMapinfo(mapinfo);
    virtualCar->setPredictor(py_predictor);
    virtualCar->setfollowingPlanner(new AstarPlanner(mapinfo));
    //virtualCar->setlinechangePlanner(new AstarPlanner(mapinfo));
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
                }   //TODO: nothing happen? since the initial value of replayAgentDictionary is empty
            this->agentDictionary.erase(agent);
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

            //generateVirtualCar();
            //generateFSMVirtualCar();
            generateBehaveCar();
            cout<<"\n******* add car, update times = " << updateTimes <<"******"<<endl<<endl;
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

//TODO: Now I set `Car_Num` = 1, so the default car_id is 0
core::Trajectory Simulator::randomly_sample(int car_id){

    printf("## There are %d cars now\n", int(this->agentDictionary.size()));
    core::Trajectory traj;

    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; 
        auto prestate = agent->get_preState();
        auto curstate = agent->getState();

        if (agent->getId() == car_id){
            printf("# Find car_id %d, historical length = %d\n", car_id, int(prestate.size()));
   
            // NOTE: Now I just push one `state` into `traj`, you can change the time horizon
            auto state = new core::State();
            state->track_id=car_id;
            state->frame_id=int(prestate.size());
            state->timestamp_ms=int(prestate.size()) * 100;
            state->agent_type="car";
            state->x=curstate[0];      
            state->y=curstate[1];
            state->vx=curstate[3];
            state->vy=curstate[4];
            state->psi_rad=curstate[2];
            state->length=agent->length_;
            state->width=agent->width_;

            traj.emplace_back(state);
            return traj;
        }
    }

    printf("# Did not find car_id %d\n", car_id);
    auto state = new core::State();
    state->track_id=233;
    state->frame_id=233;
    state->timestamp_ms=233;
    state->agent_type="car";
    state->x=233;
    state->y=233;
    state->vx=233;
    state->vy=233;
    state->psi_rad=233;
    state->length=233;
    state->width=233;

    traj.emplace_back(state);
    return traj;
}


void Simulator::upload_traj(int car_id, core::Trajectory traj){
    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first; 

        if (agent->getId() == car_id){
            printf("# Find car_id %d, Upload the PredictTraj\n", car_id);

            PredictTra result;
            OneTra inittraj;
            result.Trajs.push_back(inittraj);

            for (auto state: traj){
                TraPoints initpoint;

                initpoint.t = state->timestamp_ms;
                initpoint.x = state->x;
                initpoint.y = state->y;
                initpoint.theta = state->psi_rad;

                result.Trajs[0].Traj.push_back(initpoint);
            }
            result.Trajs[0].Probability = 1.0;

            agent->getPredictor()->set_client_traj(result);
        }
    }

    /////////////////////////// Tick()
    //COPY FROM Simulator::run()

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
    // if( creat_flag == 0)
    // {
    //     timeval T_now;
    //     tm *area;
    //     gettimeofday(&T_now,NULL);
    //     area=localtime(&(T_now.tv_sec));
    //     sprintf(write_file_name,"/home/mscsim/ao/framework/test"); //_%s.txt",asctime(area));
    //     ofstream File_creat(write_file_name);
    //     File_creat.close();
    //     creat_flag++;
    //     std::cout<<creat_flag<<std::endl;
    //     std::cout<<write_file_name<<std::endl;
    // }
    // std::string writebuf = "";
    // for (auto pair : this->agentDictionary) {
    //     agents.push_back(pair.first); // get pointers to all agents
    //     int i;
    //     Vector vehstate;
    //     if (pair.first->getType() != AgentType::RealCar) {
    //         i = pair.first->getId();
    //         vehstate = pair.first->getState();
    //         num++;
    //         // writebuf += std::to_string(i) + ',' + std::to_string(vehstate[0]) + ',' + std::to_string(vehstate[1]) + ',' +
    //                     // std::to_string(vehstate[2]) + ',' + std::to_string(vehstate[3]) + ',' + std::to_string(vehstate[4]) +
    //                     // ',' + std::to_string(vehstate[5]) + ','  + std::to_string(2.6) + ',' + std::to_string(1.6) + '\n';
    //     }
    // }
    // writebuf = "-----------------\n" + writebuf;
    // out.open(write_file_name,std::ios::app);
    // if(out.is_open()) {
    //     out << writebuf;
    //     out.close();
    // }
    // else
    // {
    //     cout << "no such file" << endl;
    // }

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
//ReplayGenerator* Simulator::ReplayGeneratorPtr = new ReplayGenerator(); 
vector<ReplayAgent*> Simulator::replayAgentDictionary  = vector<ReplayAgent*>();
LaneletMapReader* Simulator::mapreader = new LaneletMapReader(exampleMapPath,0.0,0.0);
//map = load(exampleMapPath, projection::UtmProjector(Origin({37.8997956297, -122.29974290381})));