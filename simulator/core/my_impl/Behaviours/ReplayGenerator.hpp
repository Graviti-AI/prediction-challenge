#pragma once

#include "../Agents/Agent.hpp"
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
    void setInittime(std::chrono::time_point<std::chrono::system_clock>& time) {
        init_time = time;
    }
    bool getFutureState(std::vector< std::vector<double> > & future_state, int N);
    Vector last_result;
protected:
    int currentStateIndex = 0;
    Vector Update();
private:
    Trajectory trajectory;
    std::chrono::time_point<std::chrono::system_clock> init_time;
};


class ReplayGenerator {
public:
    void loadCSV(std::string filePath);
    ReplayAgent* generateReplayAgentIfNeeded();

private:
    std::vector<std::string> split(const std::string &str,const std::string &pattern);
    std::vector<Trajectory> allTrajectories;
    std::chrono::time_point<std::chrono::system_clock> init_time;
    int index = 0;
};
