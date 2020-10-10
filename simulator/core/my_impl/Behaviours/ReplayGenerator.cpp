//
// Modified by SYF on 9/20/20.
//

#include "ReplayGenerator.hpp"
#include "../Simulator/Simulator.hpp"
#include <random>
#include <iostream>
#include <fstream>
#include <sstream>
#define Pi 3.1415926

template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

ReplayAgent::ReplayAgent(int id, Vector initialState)
: Agent(id, initialState)  {
    update_times = 0;
}

/// Type getter (overridden)
/// \return type (virtual car)
AgentType ReplayAgent::getType() const {
    return AgentType::ReplayCar;
}
/// Task run
void ReplayAgent::Run() {
    if (isRunning) {
        return;
    } else {
        isRunning = true;
    }

    Vector tmpState = this->Update();
    Vector nextState = this->getState();

    if(! tmpState.empty()) {
        //tmpState[0] *= 100.0;
        //tmpState[1] *= 100.0;

        for (int i = 0;i<6;i++) {
            nextState[i] = tmpState[i];
        }
    }  
    else{
        hasReachedDestinaiton = true;
        printf("Replat Car (%d) has arrived destination!\n", getId());
    }

    mapinfo->update(nextState);

    this->setNextState(nextState); // Set the next state to apply, but not apply right now.
    this->setPreState(this->getState());
    this->applyNextState();

    if (in_predictor->getType() == PredictorType::GroundTruthPredictor){
        set_planner_buffer();   // Designed for ground truth predictor
    }

    vector<Agent *> agents =  Simulator::agentsForThread;
    in_PredictTra_ = in_predictor->update(nextState, agents);
    PredictTra_ = in_PredictTra_;   //TODO:

    if (in_predictor->getType() == PredictorType::NoPredictor){
        assert(in_PredictTra_.Trajs.size() == 0);
    }
    else{
        assert(in_PredictTra_.Trajs.size() >= 1);
    }

    if (ex_predictor != nullptr){
        ex_PredictTra_ = ex_predictor->update(nextState, agents);
        PredictTra_ = ex_PredictTra_;   //TODO:

        assert(ex_PredictTra_.Trajs.size() >= 1);
    }

    isRunning = false;
}
/// Update agent's state
/// \reuturn next state
Vector ReplayAgent::Update() {
    int kTs = 1; // wwx - kTs = 1 means planner_Ts = 0.1ms; kTs = 2 means planner_Ts = 0.2ms;
    //std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    //double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();

    //int frame_id = (spinnedTime-trajectory.second[0].first)/100;
    //double interpolateValue = std::fmod(spinnedTime, 100)/100.0;

    int frame_id = update_times / 10;
    double interpolateValue = 1 - 0.1 * (update_times % 10);

    printf("Replay car (%d) Updated, progress (%d / %d)\n", getId(), frame_id, int(trajectory.second.size()));

    if(frame_id < int(trajectory.second.size()) - kTs) {
        Vector preFrame = trajectory.second[frame_id].second;
        Vector priorFrame = trajectory.second[frame_id + kTs].second;
        double x = preFrame[0]; //interpolateValue*preFrame[0] + (1 - interpolateValue)*priorFrame[0];
        double y = preFrame[1]; //interpolateValue*preFrame[1] + (1 - interpolateValue)*priorFrame[1];
        double yaw = preFrame[2];   //interpolateValue*preFrame[2] + (1 - interpolateValue)*priorFrame[2];
        double v_x = interpolateValue*preFrame[3] + (1 - interpolateValue)*priorFrame[3];
        double v_y = interpolateValue*preFrame[4] + (1 - interpolateValue)*priorFrame[4];
        double length = preFrame[6];
        double width = preFrame[7];

        update_times ++;    //NOTE

        //std::cout <<" Id: " << id << " Ts: " << (priorFrame[0]-preFrame[0])/v_x << " Ts: " << (priorFrame[1]-preFrame[1])/v_y << std::endl;
        Vector result{x, y, yaw, v_x, v_y, 0, 0, length, width};
        //last_result = result;
        return result;
    }

    return Vector();
}

void ReplayAgent::set_planner_buffer(){
    int kTs = 1;
    planner_buffer.clear();

    for (int i = 0; i < 30; i ++){         // add 30 future points (the same horizon as INTERPRET challenge)
        Vector futureState = Vector(6, 0.0);

        int frame_id = (update_times + i) / 10;
        double interpolateValue = 1 - 0.1 * ( (update_times + i) % 10);

        if(frame_id < int(trajectory.second.size()) - kTs) {
            Vector preFrame = trajectory.second[frame_id].second;
            Vector priorFrame = trajectory.second[frame_id + kTs].second;

            futureState[0] = preFrame[0];
            futureState[1] = preFrame[1];
            futureState[2] = preFrame[2];
            futureState[3] = interpolateValue*preFrame[3] + (1 - interpolateValue)*priorFrame[3];
            futureState[4] = interpolateValue*preFrame[4] + (1 - interpolateValue)*priorFrame[4];
            futureState[5] = 0.0;

            planner_buffer.push_back(futureState);
        }
        else {
            if (planner_buffer.size() > 0)
                futureState = planner_buffer.back();
            else {
                auto now_state = getState();
                for (int i = 0;i<6;i++) futureState[i] = now_state[i];
            }
            
            assert(futureState.size() == 6);
            futureState[0] += futureState[3] * std::cos(futureState[2]) * SIM_TICK;
            futureState[1] += futureState[3] * std::sin(futureState[2]) * SIM_TICK;

            planner_buffer.push_back(futureState);
        }
    }
    assert(planner_buffer.size() == 30);

    /*
    printf("*** DEBUG | planner_buffer\n");
    for (int i = 0; i < 30; i ++){
        for (int j = 0; j < 6; j ++)
            printf("%.3lf ", planner_buffer[i][j]);
        printf("\n");
    }
    */
}

/*
bool ReplayAgent::getFutureState(std::vector< std::vector<double> > & future_state, int N)
{
    // N preview hrizon, including the start state at the current time stamp and N-1 future states
    int kTs = 1; // wwx - kTs = 1 means planner_Ts = 0.1ms; kTs = 2 means planner_Ts = 0.2ms;
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
    if (trajectory.second.size() == 0) return false;
    int frame_id0 = (spinnedTime-trajectory.second[0].first)/100;
    int index = 0;
    //double interpolateValue = std::fmod(spinnedTime, 100)/100.0;
    for (int frame_id = frame_id0; frame_id < frame_id0 + kTs*N; frame_id += kTs)
        if(frame_id < trajectory.second.size()) {
            Vector preFrame = trajectory.second[frame_id].second;
            double x = preFrame[0];
            double y = -preFrame[1]; // Note!
            double yaw = -preFrame[2];
            double v_x = preFrame[3];
            double v_y = -preFrame[4];
            double length = preFrame[6];
            double width = preFrame[7];
            Vector result{x, y, yaw, v_x, v_y, 0, 0, length, width};
            future_state[index] = result;
            index++;
        } else {
            if (index - 1 >= 0)
                future_state[index] = future_state[index-1];
            else
                future_state[index] = last_result;
            index++;
        }
    return true;
}
*/


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
/// randomly sample a replay car, then you can use `ReplayGenerator::generateReplayAgent` to generate
/// \return vector<int> = (track_id, start_timestamp, end_timestamp)
std::vector<int> ReplayGenerator::random_sample(){
    int tt = rand() % allTrajectories.size() + 1;

    for (auto it : allTrajectories){
        tt --;
        if (tt == 0){
            Trajectory traj = it.second;
            assert(traj.first == it.first);

            std::vector<int> info;
            info.push_back(traj.first);
            info.push_back(traj.second.front().first);
            info.push_back(traj.second.back().first);

            return info;
        }
    }
    assert(false);
}

/// return a replay car with a specific start_timestamp
/// \return vector<int> = (track_id, start_timestamp, end_timestamp)
std::vector<int> ReplayGenerator::specific_sample(int start_timestamp){
    for (auto it : allTrajectories){
        Trajectory traj = it.second;
        
        if (traj.second.front().first == start_timestamp){
            std::vector<int> info;
            info.push_back(traj.first);
            info.push_back(traj.second.front().first);
            info.push_back(traj.second.back().first);

            return info;
        }
    }

    return random_sample();
}
*/


std::vector<std::vector<int> > ReplayGenerator::filter_replay_car(int ReplayStartTimestamp_ms, int ReplayEndTimestamp_ms){
    std::vector<std::vector<int> > replay_info_pool;

    for (auto it : allTrajectories){
        Trajectory traj = it.second;

        // This car doesn't appear in [ReplayStartTimestamp_ms, ReplayEndTimestamp_ms]
        if (traj.second.front().first >= ReplayEndTimestamp_ms)
            continue;
        
        if (traj.second.back().first <= ReplayStartTimestamp_ms)
            continue;
        
        std::vector<int> info;
        info.push_back(traj.first);
        info.push_back(max(traj.second.front().first, ReplayStartTimestamp_ms));
        info.push_back(min(traj.second.back().first, ReplayEndTimestamp_ms+100));

        replay_info_pool.push_back(info);
    }
    return replay_info_pool;
}


/// Generate a replay car
/// \return an agent with a recorded trajectory
ReplayAgent* ReplayGenerator::generateReplayAgent(int track_id, int start_timestamp, int end_timestamp){
    assert(this->allTrajectories.find(track_id) != this->allTrajectories.end());
    Trajectory traj = this->allTrajectories[track_id];

    assert(traj.second.front().first <= start_timestamp && end_timestamp <= traj.second.back().first);

    //std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    //double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
    
    ReplayAgent* newAgent = nullptr;
    // std::cout << "Spinned time : " << spinnedTime << std::endl;
    // std::cout << "first time stamp : " << traj.second.front().first << std::endl;

    //if (start_timestamp <= spinnedTime && spinnedTime < end_timestamp) {
        //printf("## Generate Replay Car, track id: %d, start: %d, end: %d\n", track_id, start_timestamp, end_timestamp);

        Vector initState(6, 0.0);
        newAgent = new ReplayAgent(track_id, initState);

        // slice trajectory
        Trajectory slice_traj = std::make_pair(traj.first, TrajectoryPoints());
        for (auto it : traj.second){
            if (it.first < start_timestamp){
                Vector preLoadedState = Vector(6, 0.0);
                for (int j = 0; j < 6; j ++)
                    preLoadedState[j] = it.second[j];
                
                for (int i = 0; i < 10; i ++){
                    // Don't need to interpolate, since the pre-loaded trajectory is only used for py_predictor
                    // and the py_predictor will downsample per 10 timestamps
                    newAgent->setPreState(preLoadedState);
                }
            }
            else {
                slice_traj.second.push_back(it);
            }
        }

        assert(slice_traj.first == track_id);
        assert(slice_traj.second.front().first == start_timestamp);
        assert(slice_traj.second.back().first >= end_timestamp);
        assert(newAgent->get_preState().size() == (start_timestamp - traj.second.front().first) / 10);

        newAgent->setTrajectory(slice_traj);
    //}
    
    return newAgent;
}

/// Split a string
std::vector<std::string> ReplayGenerator::split(const std::string &str,const std::string &pattern) {
        // const char* convert to char*
        char * strc = new char[strlen(str.c_str())+1];
        strcpy(strc, str.c_str());
        std::vector<std::string> resultVec;
        char* tmpStr = strtok(strc, pattern.c_str());
        while (tmpStr != NULL)
        {
            resultVec.push_back(std::string(tmpStr));
            tmpStr = strtok(NULL, pattern.c_str());
        }

        delete[] strc;

        return resultVec;
}

/// Load data from recorded CSV file
void ReplayGenerator::loadCSV(std::string filePath) {
    printf("Replay Generator: Load CSV From %s\n", filePath.c_str());

    //init_time = std::chrono::system_clock::now(); // set init time

    std::ifstream infile;
    infile.open(filePath);

    std::string x, y, line;
    int id, lastId = 1;
    double xd, yd, yaw, vx, vy, yaw_rate, length, width;
    TrajectoryPoints trajectory_points;

    getline(infile, line); // ignore the first line
    while(getline(infile, line))
    {
        std::vector<std::string> values = split(line, ",");
        id = stringToNum<double>(values[0]);//std::stoi(values[0].c_str());
        int timestamp = stringToNum<int>(values[2]);// atof(values[2].c_str());
        xd = stringToNum<double>(values[4]) - 1000.0;//(atoi(values[4].c_str()));
        yd = stringToNum<double>(values[5]) - 1000.0;//(atof(values[5].c_str()));
        vx = stringToNum<double>(values[6]);//atof(values[6].c_str());
        vy = stringToNum<double>(values[7]);//atof(values[7].c_str());
        yaw = stringToNum<double>(values[8]);//atof(values[8].c_str());

        length = stringToNum<double>(values[9]);//atof(values[9].c_str());
        width = stringToNum<double>(values[10]);//atof(values[10].c_str());
        //double co = std::cos(155 * Pi / 180.0);  // 155 is the rotation bewteen the mini map and our global map
        //double si = std::sin(155 * Pi / 180.0);

        //double x  = xd*co + yd*si;
        //double y  = -xd*si + yd*co;
        //std::cout<<xd*100<<std::endl;

        //Vector p{xd, yd, yaw, vx, vy, 0, length, width};
        Vector p{xd, yd, yaw, sqrt(vx*vx+vy*vy), 0, 0, length, width}; //TODO: I set the v = sqrt(vx**2 + vy ** 2)
        TrajectoryPoint tp = std::make_pair(timestamp, p);

        if (lastId != id) {
            this->allTrajectories[lastId] = std::make_pair(lastId, trajectory_points);
            trajectory_points.clear();
        }
        trajectory_points.push_back(tp);
        lastId = id;
    }

    if (trajectory_points.size() > 0){
        this->allTrajectories[lastId] = std::make_pair(lastId, trajectory_points);
        trajectory_points.clear();
    }

    /*  DEBUG
    for (auto it = allTrajectories.begin(); it != allTrajectories.end(); it ++){
        auto traj = it->second.second;

        for (int i = 0; i + 1 < traj.size(); i ++){
            auto p_now = traj[i].second;
            auto p_after = traj[i + 1].second;

            printf("p_now | x: %.3lf, y: %.3lf, vx: %.3lf, vy: %.3lf csv_psi: %.3lf, my_psi: %.3lf\n", p_now[0], p_now[1], p_now[3], p_now[4], p_now[2], atan2(p_now[4], p_now[3]));
        }
        exit(0);
    }
    */

    printf("Replay Generator: Trajectory Number: %d\n", int(this->allTrajectories.size()));
}
