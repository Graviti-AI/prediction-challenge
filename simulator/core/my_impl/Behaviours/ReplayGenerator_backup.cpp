#include "ReplayGenerator_backup.hpp"
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


ReplayAgent::ReplayAgent(int id, Vector initialState)
: Agent(id, initialState)  {
    init_time = std::chrono::system_clock::now();
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
    if(!tmpState.empty()) {
        tmpState[0] *= 100.0;
        tmpState[1] *= 100.0;
    }
    this->state = tmpState;
    // std::cout << "Current Position : " << state[0] << ", " << state[1] << std::endl;
    isRunning = false;
}
/// Update agent's state
/// \reuturn next state
Vector ReplayAgent::Update() {
    int kTs = 1; // wwx - kTs = 1 means planner_Ts = 0.1ms; kTs = 2 means planner_Ts = 0.2ms;
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
    int frame_id = (spinnedTime-trajectory.second[0].first)/100;
    double interpolateValue = std::fmod(spinnedTime, 100)/100.0;
    if(frame_id < trajectory.second.size() - kTs) {
        Vector preFrame = trajectory.second[frame_id].second;
        Vector priorFrame = trajectory.second[frame_id + kTs].second;
        double x = preFrame[0]; //interpolateValue*preFrame[0] + (1 - interpolateValue)*priorFrame[0];
        double y = preFrame[1]; //interpolateValue*preFrame[1] + (1 - interpolateValue)*priorFrame[1];
        double yaw = preFrame[2];   //interpolateValue*preFrame[2] + (1 - interpolateValue)*priorFrame[2];
        double v_x = interpolateValue*preFrame[3] + (1 - interpolateValue)*priorFrame[3];
        double v_y = interpolateValue*preFrame[4] + (1 - interpolateValue)*priorFrame[4];
        double length = preFrame[6];
        double width = preFrame[7];
        //std::cout <<" Id: " << id << " Ts: " << (priorFrame[0]-preFrame[0])/v_x << " Ts: " << (priorFrame[1]-preFrame[1])/v_y << std::endl;
        Vector result{x, y, yaw, v_x, v_y, 0, 0, length, width};
        last_result = result;
        return result;
    }
    return Vector();
}

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

/// ############################################################################


/// Generate a replay car
/// \return an agent with a recorded trajectory
ReplayAgent* ReplayGenerator::generateReplayAgentIfNeeded() {
    ReplayAgent* newAgent = nullptr;
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
    int frame_id = spinnedTime/100;
    int next_index = index + 1;
    Trajectory traj = this->allTrajectories[next_index];
    // std::cout << "Spinned time : " << spinnedTime << std::endl;
    // std::cout << "first time stamp : " << traj.second.front().first << std::endl;
    if (traj.second.front().first <= spinnedTime) {
        int id = random();
        Vector initState(7, 0);
        newAgent = new ReplayAgent(id, initState);
        newAgent->setTrajectory(traj);
        newAgent->setInittime(init_time);
        this->index = next_index;
        //std::cout << "Replay car generated ~ " << std::endl;
    }
    
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
    printf("Load CSV From %s\n", filePath.c_str());

    init_time = std::chrono::system_clock::now(); // set init time

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
        int timestamp =stringToNum<double>(values[2]);// atof(values[2].c_str());
        xd = stringToNum<double>(values[4]);//(atoi(values[4].c_str()));
        yd = -stringToNum<double>(values[5]);//(atof(values[5].c_str()));
        vx = stringToNum<double>(values[6]);//atof(values[6].c_str());
        vy = -stringToNum<double>(values[7]);//atof(values[7].c_str());
        yaw = -stringToNum<double>(values[8]);//atof(values[8].c_str());
        length = stringToNum<double>(values[9]);//atof(values[9].c_str());
        width = stringToNum<double>(values[10]);//atof(values[10].c_str());
        //double co = std::cos(155 * Pi / 180.0);  // 155 is the rotation bewteen the mini map and our global map
        //double si = std::sin(155 * Pi / 180.0);

        //double x  = xd*co + yd*si;
        //double y  = -xd*si + yd*co;
        //std::cout<<xd*100<<std::endl;
        Vector p{xd, yd, yaw, vx, vy, 0, length, width};
        TrajectoryPoint tp = std::make_pair(timestamp, p);

        if (lastId != id) {
            this->allTrajectories.push_back(std::make_pair(lastId, trajectory_points));
            trajectory_points.clear();
        }
        trajectory_points.push_back(tp);
        lastId = id;
    }

    printf("Trajectory Number: %d\n", int(this->allTrajectories.size()));
}
