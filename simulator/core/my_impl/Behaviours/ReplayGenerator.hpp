//
// Modified by SYF on 9/20/20.
//


#pragma once

#include "../Agents/Agent.hpp"
#include <map>
#include <vector>
#include <cstring>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <chrono>

typedef std::pair<int, Vector> TrajectoryPoint;
typedef std::vector<TrajectoryPoint> TrajectoryPoints;
typedef std::pair<int, TrajectoryPoints> Trajectory;



class ReplayAgent : public Agent {

public:
    void Run();
    AgentType getType() const override;
    ReplayAgent(int id, Vector initialState);
    
    void setTrajectory(Trajectory& traj) {
        trajectory = traj;
    }
    const Trajectory& getTrajectory(){
        return trajectory;
    }

    //void setInittime(std::chrono::time_point<std::chrono::system_clock>& time) {
    //    init_time = time;
    //}
    //bool getFutureState(std::vector< std::vector<double> > & future_state, int N);
    //Vector last_result;
    //int currentStateIndex = 0;

    void set_planner_buffer();
    Vector Update();
private:
    Trajectory trajectory;
    int update_times;
    //std::chrono::time_point<std::chrono::system_clock> init_time;
};


class ReplayGenerator {
public:
    void loadCSV(std::string filePath);
    ReplayAgent* generateReplayAgent(int track_id, int start_timestamp, int end_timestamp);

    std::vector<std::vector<int> > filter_replay_car(int ReplayStartTimestamp_ms, int ReplayEndTimestamp_ms);
    //std::vector<int> random_sample();
    //std::vector<int> specific_sample(int start_timestamp);

private:
    std::vector<std::string> split(const std::string &str,const std::string &pattern);

    std::map<int, Trajectory> allTrajectories;
    //std::chrono::time_point<std::chrono::system_clock> init_time;
};
