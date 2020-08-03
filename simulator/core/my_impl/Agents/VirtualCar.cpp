//
// Created by mscsim on 12/13/18.
//

#include "VirtualCar.hpp"
#include "../Simulator/Simulator.hpp"
#include "../Behaviours/ReplayGenerator.hpp"
#include "../alglib/interpolation.h"
#include "../PlannerLib/RTCLib/EigenHelper.h"

using namespace std;
bool EB_flag = false;
/// Constructor.
/// \param id id of the new virtual car.
/// \param initialState initial state of the virtual car.
VirtualCar::VirtualCar(int id, Vector initialState, LaneletBehaviour* b, Planner *p, Controller *c, Model *m)
        : Agent(id, initialState) {
        behaviour = b;
        // planner = p;
        controller = c;
        model = m;
}

VirtualCar::VirtualCar(int id, Vector initialState)
        : Agent(id, initialState) {
        first_run = true;
        last_left_lanelet_id = -1; //false;
        last_right_lanelet_id = -1; //false;
}

VirtualCar::~VirtualCar()
{
     if (CILQR_flag) { delete CILQR_planner; }
}

void VirtualCar::CILQRRun() {
    // Get the input vector from the planner, by giving human inputs and agent state.
    if (isRunning) {
        return;
    } else {
        isRunning = true;
    }
    std::vector<Agent *> agents =  Simulator::agentsForThread;
    Vector humanInput;
    Vector curState = this->getState();

    if(curState.empty()) {
        this->hasReachedDestinaiton = true;
        isRunning = false;
        return;
    }
    curState[0] /= 100.0;
    curState[1] /= -100.0;

    //std::cout << agents[0]->getType() << " " << agents[0]->getId() << std::endl;
    if (CILQR_flag) {
        // Check if need to remove    
        double dest_x = behaviour->wholePath.back().x();
        double dest_y = behaviour->wholePath.back().y();
        double dis = sqrt(pow(dest_x-curState[0], 2.0) + pow(dest_y-curState[1], 2.0));
        if(dis < 7.0) {
            this->hasReachedDestinaiton = true;
            isRunning = false;
            return; 
        }

        time_t start_t = clock(), end_t;
        std::chrono::time_point<std::chrono::system_clock> inrun_time = std::chrono::system_clock::now(), endrun_time;
        if (first_run) { // Make the initial state be on the start point of route path
            CILQR_planner = new CILQRPlanner();
            CILQR_planner->setId(id);
            CILQR_planner->enable_prediction = true;//false;//true;
            update_n = 0;
            update_t_cost = 0;
            inrun_time = std::chrono::system_clock::now();

            first_run = false;
            CILQR_planner->behaviour = behaviour;
            std::cout << endl << "Car ID : " << id << endl;
            curState = this->getState();
            curState[0] /= 100.0;
            curState[1] /= -100.0;
            curState[2] = - curState[2]; // note
            BasicPoint2d currPos(curState[0], curState[1]);

            // get the route path and initialization
            behaviour->getRoutingReferencePath(lane_change_s, behaviour->length_route_path);
            currentLanelet_ = behaviour->getCurrentLanelet();
            std::cout<< "lane_change_s.size(): " << lane_change_s.size() <<endl;
            index_lane_change_s = 0; // lane_change_s[0] = 0; the start length of route path
            behaviour->route_last_index = 0; behaviour->left_last_index = -1; behaviour->ego_last_index = -1; behaviour->right_last_index = -1;
            behaviour->route_start_s = 0;
            // set the initial state of the path
            double xx = 0, yy = 0;
            double dx = 0, dy = 0;
            double d2x = 0, d2y = 0;
            alglib::spline1ddiff(behaviour->route_x, behaviour->route_start_s, xx, dx, d2x);
            alglib::spline1ddiff(behaviour->route_y, behaviour->route_start_s, yy, dy, d2y);
            double yaw_angle = atan2(dy, dx);
            Vector nextState(6);
            nextState[0] = xx*100;
            nextState[1] = yy*(-100);
            nextState[2] = -yaw_angle;
            nextState[3] = 0;//5; // 5m/s
            nextState[4] = 0;
            nextState[5] = 0;
            state = nextState;
            isRunning = false;
            // Print shortestPath
            shortestPath_ = behaviour->getShortestPathCILQR();
            for (int i = 0; i < shortestPath_.size(); i++)
                std::cout << shortestPath_[i].id() << " ";
            std::cout << endl;
            /*
            // Print
            Eigen::MatrixXd out_point = Eigen::MatrixXd::Zero(2, behaviour->route_path.size());
            for (int i = 0; i < behaviour->route_path.size(); i++) {
                out_point(0, i) = behaviour->route_path[i].x();
                out_point(1, i) = behaviour->route_path[i].y();
            }
            RTCLib::EigenHelper::save2csv(out_point, "Output_point_route.csv");
            */
        } else {
            std::cout << endl << "Car ID : " << id << endl;
            curState = this->getState();
            curState[0] /= 100.0;
            curState[1] /= -100.0;
            curState[2] = - curState[2]; // note
            BasicPoint2d currPos(curState[0], curState[1]);

            double aaa1 = geometry::toArcCoordinates(behaviour->route_path, currPos).length;
            std::tuple<double, double, double, double, int, int> 
                ptemp = CILQR_planner->toArcCoordinatesBS(behaviour->route_path, behaviour->route_path_each_len, currPos(0,0), currPos(1,0), behaviour->route_last_index);
            double aaa2 = get<0>(ptemp);
            behaviour->route_last_index = get<5>(ptemp);
            // std::cout << "aaa1: " << aaa1 << " aaa2: " << aaa2 << " aaa1-aaa2: " << aaa1-aaa2 << " "  << endl;
            behaviour->route_start_s = aaa1;
            // if (fabs(aaa1-aaa2)>2) throw::runtime_error("Calculation error is unsatisfied!"); // aaa1 is more precise.

            // update ego, left, right lane
            // 1. check if Current Lanelet needs to be updated
            bool need_update_lane = behaviour->updateCurrentLanelet(currPos);
            if (need_update_lane) currentLanelet_ = behaviour->getCurrentLanelet();
            std::cout<< "Id: " << id << " "; std::cout << "Current Lanelet Id: " << behaviour->getCurrentLaneletId() << endl;
            // 2. check other conditions to update lanes
            std::pair<int, int> now_leftright_lane_id = behaviour->getLeftAndRightLanletId(currentLanelet_);
            std::cout<< "Id: " << id << " "; std::cout << "Left/Right Lanelet Id: " << now_leftright_lane_id.first << " " << now_leftright_lane_id.second << endl;
            if ( (index_lane_change_s+1 < lane_change_s.size() // if the route path makes lane change, update lanes!
                    && lane_change_s[index_lane_change_s] <= behaviour->route_start_s) // lane_change_s[i] store the s when the lane change is complete
                    || (last_left_lanelet_id<0)!=(now_leftright_lane_id.first<0) || (last_right_lanelet_id<0)!=(now_leftright_lane_id.second<0) ) // if left or right lane existing state changes, update it
            { 
                double stop_length;
                if (index_lane_change_s+1 < lane_change_s.size() && lane_change_s[index_lane_change_s] <= behaviour->route_start_s) {
                    stop_length = lane_change_s[index_lane_change_s+1] - behaviour->route_start_s + 10; // add 10m as redundant path
                    index_lane_change_s++;
                } else {
                    stop_length = behaviour->length_ego_lane;
                    // throw::runtime_error("index_lane_change_s is out of range!");
                }
                behaviour->getEgoReferencePath(stop_length, behaviour->length_ego_lane);
                behaviour->getLeftReferencePath(stop_length, behaviour->length_left_lane);
                behaviour->getRightReferencePath(stop_length, behaviour->length_right_lane);
                std::cout<< "Id: " << id << " "; std::cout << "Update lanes! Lanes exist? " << (! behaviour->left_lane.empty()) << (! behaviour->ego_lane.empty()) << (! behaviour->right_lane.empty()) << endl;
                /*
                // Print
                Eigen::MatrixXd out_point = Eigen::MatrixXd::Zero(2, behaviour->ego_lane.size());
                for (int i = 0; i < behaviour->ego_lane.size(); i++) {
                    out_point(0, i) = behaviour->ego_lane[i].x();
                    out_point(1, i) = behaviour->ego_lane[i].y();
                }
                RTCLib::EigenHelper::save2csv(out_point, "Output_point_ego.csv");
                if (! behaviour->left_lane.empty()){
                Eigen::MatrixXd out_point = Eigen::MatrixXd::Zero(2, behaviour->left_lane.size());
                for (int i = 0; i < behaviour->left_lane.size(); i++) {
                    out_point(0, i) = behaviour->left_lane[i].x();
                    out_point(1, i) = behaviour->left_lane[i].y();
                }
                RTCLib::EigenHelper::save2csv(out_point, "Output_point_left.csv");
                }
                if (! behaviour->right_lane.empty()){
                Eigen::MatrixXd out_point = Eigen::MatrixXd::Zero(2, behaviour->right_lane.size());
                for (int i = 0; i < behaviour->right_lane.size(); i++) {
                    out_point(0, i) = behaviour->right_lane[i].x();
                    out_point(1, i) = behaviour->right_lane[i].y();
                }
                RTCLib::EigenHelper::save2csv(out_point, "Output_point_right.csv");
                }
                */
            }
            last_left_lanelet_id = now_leftright_lane_id.first; last_right_lanelet_id = now_leftright_lane_id.second;
            // 3. update start_s of each lane
            behaviour->ego_start_s = geometry::toArcCoordinates(behaviour->ego_lane, currPos).length;
            if (! behaviour->left_lane.empty())  behaviour->left_start_s = geometry::toArcCoordinates(behaviour->left_lane, currPos).length;
            if (! behaviour->right_lane.empty())  behaviour->right_start_s = geometry::toArcCoordinates(behaviour->right_lane, currPos).length;
            //behaviour->ego_start_s = get<0>(CILQR_planner->toArcCoordinatesBS(behaviour->ego_lane, behaviour->ego_lane_each_len, currPos(0,0), currPos(1,0)));
            //if (! behaviour->left_lane.empty())
            //    behaviour->left_start_s = get<0>(CILQR_planner->toArcCoordinatesBS(behaviour->left_lane, behaviour->left_lane_each_len, currPos(0,0), currPos(1,0)));
            //if (! behaviour->right_lane.empty())
            //    behaviour->right_start_s = get<0>(CILQR_planner->toArcCoordinatesBS(behaviour->right_lane, behaviour->right_lane_each_len, currPos(0,0), currPos(1,0)));
            std::cout<< "Id: " << id << " "; std::cout << "behaviour->length_route_path: " << behaviour->length_route_path << " behaviour->route_start_s: " << behaviour->route_start_s << endl;
            std::cout<< "Id: " << id << " "; std::cout << "Lanes start s: " << behaviour->left_start_s << " " << behaviour->ego_start_s << " " << behaviour->right_start_s << std::endl;

            // call CILQR planner to obtain the next state
            //vector<Agent* > relatedAgents = behaviour->getRelatedAgents();
            vector<Agent* > relatedAgents;
            for (int i = 0; i < agents.size(); i++)
                if (agents[i]->getId() != id) relatedAgents.push_back(agents[i]);
            if (CILQR_planner->enable_prediction) {
                CILQR_planner->obs_deter.clear();
                int NN = CILQR_planner->getN();

                // get future traj from replaycar
                // ReplayAgent * replay_agent = dynamic_cast<ReplayAgent *> (agents[0]);
                std::vector<ReplayAgent *> replay_agents =  Simulator::replayAgentDictionary;
                std::cout << "replay_agents.size(): " << replay_agents.size() << std::endl;
                for (auto agent_car : replay_agents) { // It is ReplayCar
                    std::vector< std::vector<double> > future_state; //first: k*Ts; second: state = [x, y, yaw, vx, vy, yaw_rate, temp?, length, width]
                    future_state.resize(NN);
                    for (int i = 0; i < NN; i++) future_state[i].resize(9);
                    if (agent_car->getFutureState(future_state, NN))
                        //void setObsPrediction(int agent_id, std::vector< std::vector<double> > & seq_state, double half_L, double half_W, double max_acc, double min_acc);
                        //CILQR_planner->setObsPrediction(agent_car->getId(), future_state, future_state[0][7]/2, future_state[0][8]/2, 2, -3);
                        CILQR_planner->setObsPrediction(agent_car->getId(), future_state, 2.0, 1.0, 2, -3); // 2 and 1 are default
                    else
                        throw::runtime_error("Empty predicted future trajectory!");
                }
            }
            std::cout << "agents.size(): " << agents.size() << std::endl;
            auto plannerResult = CILQR_planner->update(curState, behaviour->route_path, humanInput, relatedAgents);
            plannerResult[0] *= 100.0;
            plannerResult[1] *= -100.0;
            plannerResult[2] = - plannerResult[2];
            Vector nextState = plannerResult;
            std::cout<< "Id: " << id << " "; std::cout << "Current State: X:" << curState[0] << " Y:" << curState[1] << " V:" << curState[3] << " yaw:" << curState[2] << std::endl;
            std::cout<< "Id: " << id << " "; std::cout << "Next State: X:" << nextState[0]/100.0 << " Y:" << nextState[1]/(-100.0) << " V:" << nextState[3] << " yaw:" << -nextState[2] << std::endl;
            state = nextState;
            isRunning = false;
        }
        end_t = clock();
        //std::cout << "Update state. Time Cost: " << double(end_t-start_t)/double(CLOCKS_PER_SEC) << std::endl;
        endrun_time = std::chrono::system_clock::now();
        cout<<"Update state. Time Cost: "<< double (std::chrono::duration_cast<std::chrono::milliseconds>(endrun_time - inrun_time).count()) << " ms"<<endl;
        //update_t_cost += double (std::chrono::duration_cast<std::chrono::milliseconds>(endrun_time - inrun_time).count());
        //update_n++;
        //cout<<"Update state. Mean Time Cost: "<< update_t_cost/update_n << " ms"<<endl;
        return;
    }
}

void VirtualCar::Run() {
    if (CILQR_flag) { CILQRRun(); return; }

    std::chrono::time_point<std::chrono::system_clock> inrun_time = std::chrono::system_clock::now();
    // Get the input vector from the planner, by giving human inputs and agent state.
    if (isRunning) {
        return;
    } else {
        isRunning = true;
    }
    std::vector<Agent *> agents =  Simulator::agentsForThread;
    Vector humanInput;
    Vector curState = this->getState();

    if(curState.empty()) {
        this->hasReachedDestinaiton = true;
        isRunning = false;
        return;
    }
    curState[0] /= 100.0;
    curState[1] /= -100.0;

    // Check if need to remove    
    double dest_x = behaviour->wholePath.back().x();
    double dest_y = behaviour->wholePath.back().y();
    double dis = sqrt(pow(dest_x-curState[0], 2.0) + pow(dest_y-curState[1], 2.0));
    if(dis < 3.0) {
        this->hasReachedDestinaiton = true;
        isRunning = false;
        return;
    }

    Vector nextState(7, 0.0);
    int currMode = behaviour->getMode();
    if (curState[3] > 10) {
        std::cout << "Velocity : " << curState[3] << std::endl;
        std::cout << "Mode : " << currMode << std::endl;
    }
    
    if (first_run) ebplanner = new EBPlanner();
    first_run = false;

    testmapptr->update(curState);
    cout<<"behave lanelet id: "<< behaviour->getCurrentLaneletId() <<" s: "<<behaviour->getS()<<" state: ";
    for(auto one : behaviour->getCurrentState())
    {cout<<one<<" ";}
    cout<<endl;
    cout<<"mapinfo lanelet id: "<< testmapptr->getCurrentLaneletId()<<" s: "<<testmapptr->getS()<<" state: ";
    for(auto one : testmapptr->getCurrentState())
    {cout<<one<<" ";}
    cout<<endl;

    //cout<<"agent size is "<<agents.size()<<endl;
    std::chrono::time_point<std::chrono::system_clock> behave_time = std::chrono::system_clock::now();
    Vector behaviourState = behaviour->update(curState, humanInput, agents);

    if (EB_flag) {
        std::vector<ReplayAgent *> replay_agents =  Simulator::replayAgentDictionary;
        std::cout << "replay_agents.size(): " << replay_agents.size() << std::endl;
        for (auto agent_car : replay_agents) { // It is ReplayCar
            std::vector<std::vector<double> > future_state; //first: k*Ts; second: state = [x, y, yaw, vx, vy, yaw_rate, temp?, length, width]
            future_state.resize(51); /// 26 means 5 seconds.
            for (int i = 0; i < 51; i++) future_state[i].resize(9);
            if (agent_car->getFutureState(future_state, 26)){
                //void setObsPrediction(int agent_id, std::vector< std::vector<double> > & seq_state, double half_L, double half_W, double max_acc, double min_acc);
                //CILQR_planner->setObsPrediction(agent_car->getId(), future_state, future_state[0][7]/2, future_state[0][8]/2, 2, -3);
                //CILQR_planner->setObsPrediction(agent_car->getId(), future_state, 2.0, 1.0, 2, -3); // 2 and 1 are default
                // ebplanner.obstacles_set;
            }else
                throw::runtime_error("Empty predicted future trajectory!");
        }
    }
    

    if (behaviourState.empty()) {
        hasReachedDestinaiton = true; // Reached end
        state = behaviourState;
        isRunning = false;
        return;
    } else if (currMode == Mode::merging) {
        std::chrono::time_point<std::chrono::system_clock> init_time = std::chrono::system_clock::now();

        // Vector plannerResult = planner->update(curState, behaviourState, humanInput, agents);
        vector<Agent* > relatedAgents = behaviour->getRelatedAgents();
        vector<Agent* > surrAgents; //= behaviour->getSurroundingAgents(curState, agents);
        // std::cout << "Size : " << relatedAgents.size() << std::endl;
        BasicPoint2d currPos(curState[0], curState[1]);
        this->s_ = geometry::toArcCoordinates(behaviour->wholePath, currPos).length;
        auto tmpPlannerResult = planner->update(curState, behaviour->spl_x,behaviour->spl_y, humanInput, relatedAgents, this->s_);
        //auto tmpPlannerResult = planner->update(curState, behaviour->mergingPath, humanInput, relatedAgents, s);
        if (behaviour->getMode() == Mode::following) {
            behaviour->mergingPath.clear();
        }
        Vector plannerResult(7, 0.0);

        double dis = 0;
        std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
        double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
        cout<<"merging"<<spinnedTime<<" ms"<<endl;
        int frame_id = spinnedTime/10+1;
        //frame_id = 10;
        plannerResult[0] = tmpPlannerResult[1 + 5*frame_id];
        plannerResult[1] = tmpPlannerResult[2 + 5*frame_id];
        plannerResult[3] = tmpPlannerResult[3 + 5*frame_id];
        plannerResult[2] = tmpPlannerResult[4 + 5*frame_id];
        //plannerResult[2] = std::atan2(-plannerResult[1] + curState[1], plannerResult[0] - curState[0]);


        
        plannerResult[0] *= 100.0;
        plannerResult[1] *= -100.0;

        if (plannerResult[3] > 10) {
            std::cout << "Before Astar velocity : " << curState[3] << std::endl;
            std::cout << "Astar velocity : " << plannerResult[3] << std::endl;
        }

        nextState = plannerResult;
    } else if (currMode == Mode::following) {
        // planner->tmpCount = 0;
        std::chrono::time_point<std::chrono::system_clock> init_time = std::chrono::system_clock::now();
        //cout<<"behavior time: "<< double (std::chrono::duration_cast<std::chrono::milliseconds>(init_time - behave_time).count()) << " ms"<<endl;
        behaviourState[2] = std::atan2(-behaviourState[1] + curState[1], behaviourState[0] - curState[0]);
        if (behaviour->getCurrentLanelet().id() == 30400 || behaviour->getCurrentLanelet().id() == 30402) {
            behaviourState[2] = curState[2];
        }
        // behaviourState[0] += 1.8 * cos(behaviourState[2]);
        // behaviourState[1] += 1.8 * sin(behaviourState[2]);


        behaviourState[0] *= 100.0;
        behaviourState[1] *= -100.0;
        nextState = behaviourState;


        ////test difference between IDM and Planner
        //cout<<"state now : "<<endl;
        //for (auto onestate : curState) cout<<onestate<<" "<<endl;
        //cout<<"state from behaviourState : "<<endl;
        //for (auto onestate : behaviourState) cout<<onestate<<" "<<endl;
        vector<Agent* > OtherAgent;//= behaviour->getRelatedAgents();// = behaviour->getOtherAgents(curState, agents);
        BasicPoint2d currPos(curState[0], curState[1]);
        // cout<<"OK"<<endl;
        this->s_ = geometry::toArcCoordinates(behaviour->wholePath, currPos).length;
        //cout<<"s_now: "<<s<<endl;
        std::vector<double> tmpPlannerResult;
        if (EB_flag) tmpPlannerResult = ebplanner->update(curState, behaviour->spl_x,behaviour->spl_y, humanInput, OtherAgent, this->s_);
        else tmpPlannerResult = planner->update(curState, behaviour->spl_x,behaviour->spl_y, humanInput, OtherAgent, this->s_);
        //auto test_result = ebplanner->update(curState, behaviour->spl_x,behaviour->spl_y, humanInput, OtherAgent, this->s_);

        Vector plannerResult(7, 0.0);
        
        
        std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();        
        double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
        //if (spinnedTime > 100)
        cout<<spinnedTime<<" ms"<<endl;
        int frame_id = spinnedTime/10+1;

        plannerResult[0] = tmpPlannerResult[1 + 5*frame_id];
        plannerResult[1] = tmpPlannerResult[2 + 5*frame_id];
        plannerResult[3] = tmpPlannerResult[3 + 5*frame_id];
        plannerResult[2] = tmpPlannerResult[4 + 5*frame_id];
        //plannerResult[2] = std::atan2(-plannerResult[1] + curState[1], plannerResult[0] - curState[0]);
        //cout<<"state from plannerResult : ";
        //cout<<plannerResult[3]<<endl;
        //for (auto onestate : plannerResult) cout<<onestate<<" "<<endl;
        plannerResult[0] *= 100.0;
        plannerResult[1] *= -100.0;

        if (plannerResult[3] > 10) {
            std::cout << "Before Astar velocity : " << curState[3] << std::endl;
            std::cout << "Astar velocity : " << plannerResult[3] << std::endl;
        }

        nextState = plannerResult;
    } else {
        cout<<"use behaviour directly!"<<endl;
        BasicPoint2d currPos(curState[0], curState[1]);
        this->s_ = geometry::toArcCoordinates(behaviour->wholePath, currPos).length;
        behaviourState[2] = std::atan2(-behaviourState[1] + curState[1], behaviourState[0] - curState[0]);
        if (behaviour->getCurrentLanelet().id() == 30400 || behaviour->getCurrentLanelet().id() == 30402) {
            behaviourState[2] = curState[2];
        }
        // behaviourState[0] += 1.8 * cos(behaviourState[2]);
        // behaviourState[1] += 1.8 * sin(behaviourState[2]);


        behaviourState[0] *= 100.0;
        behaviourState[1] *= -100.0;
        nextState = behaviourState;
    }
    

    if (state[0] == 5000.0) {
        state = nextState;
        isRunning = false;
        return;
    }

// Init a bicycle pure pursuit model
    bicyclePurePursuiter.stateInitial(state[0], state[1], state[2], state[3]);
    bicyclePurePursuiter.pursuit_a(nextState[0], nextState[1]);
    bicyclePurePursuiter.pursuit_df(nextState[0], nextState[1]);
    bicyclePurePursuiter.stateUpdate();
    // cout << "------------------------" << endl;
    // cout << "1 : " << state[0] << " vs "<< bicyclePurePursuiter.x << endl;
    // cout << "2 : " << state[1] << " vs "<< bicyclePurePursuiter.y << endl;
    // cout << "3 : " << state[2] << " vs "<< bicyclePurePursuiter.yaw << endl;
    // cout << "------------------------" << endl;

    // nextState[0] = bicyclePurePursuiter.x;
    // nextState[1] = bicyclePurePursuiter.y;
    // nextState[2] = bicyclePurePursuiter.yaw;
    state = nextState;
        // std::cout << "Simulator Run" << std::endl;
// tmp
    if (state[3] < 0.1) {
        state[2] = curState[2];
    }
    
//     Vector nextState = model->update(this->getState(), intermediate);
    // this->setNextState(nextState); // Set the next state to apply, but not apply right now.
//     this->setPreState(this->getState());
    // this->applyNextState();
    isRunning = false;

    std::chrono::time_point<std::chrono::system_clock> endrun_time = std::chrono::system_clock::now();
    cout<<"run time: "<< double (std::chrono::duration_cast<std::chrono::milliseconds>(endrun_time - inrun_time).count()) << " ms"<<endl;
}

/// Type getter (overridden)
/// \return type (virtual car)
AgentType VirtualCar::getType() const {
    return AgentType::VirtualCar;
}

LaneletBehaviour* VirtualCar::getBehaviour()  {
    return this->behaviour;
}
