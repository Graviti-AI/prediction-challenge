//
// Created by SYF on 10/7/20.
//

#include "GroundTruthPredictor.hpp"
#include "../Simulator/Simulator.hpp"
#include <assert.h>


GroundTruthPredictor::GroundTruthPredictor(Agent* agent_ibt, double time_step, double horizon): Predictor(agent_ibt,time_step,horizon){
}

void GroundTruthPredictor::set_traj(PredictTra traj){
    assert(false);  // python client wouldn't send back to this predictor.
}

PredictTra GroundTruthPredictor::update(Vector currentState,std::vector<Agent*> agents){
    state = PredictorState::wait4update;

    PredictTra result;
    OneTra inittraj;
    auto mapinfo_ = agent_ibt_->mapinfo;

    result.Trajs.push_back(inittraj);
    assert(agent_ibt_->planner_buffer.size() == 30);

    for (auto state: agent_ibt_->planner_buffer){
        TraPoints initpoint;

        //TODO: change data type from vector<double> to TraPoints
        initpoint.t = SIM_TICK * result.Trajs[0].Traj.size();  //state->timestamp_ms;
        initpoint.x = state[0];
        initpoint.y = state[1];
        initpoint.theta = state[2];
        initpoint.v = std::sqrt(state[3] * state[3] + state[4] * state[4]);
        
        initpoint.delta_theta = 0.0;
        initpoint.a = 0.0;
        initpoint.jerk = 0.0;

        auto xy2laneid_res = HelperFunction::xy2laneid(initpoint.x, initpoint.y, initpoint.theta, mapinfo_->mapPtr_);
        auto candidate_lanelet_id = xy2laneid_res.first;

        initpoint.current_lanelet = mapinfo_->mapPtr_->laneletLayer.get(candidate_lanelet_id);
        initpoint.s_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).length;
        initpoint.d_of_current_lanelet = geometry::toArcCoordinates(initpoint.current_lanelet.centerline2d(), BasicPoint2d(initpoint.x, initpoint.y)).distance;

        result.Trajs[0].Traj.push_back(initpoint);
    }
    assert(result.Trajs[0].Traj.size() == 30);

    result.Trajs[0].Probability = 1.0;

    //printf("$$$$ GroundTruth\n");
    for (int j = 0; j < result.Trajs[0].Traj.size(); j ++)
        if (j == 0 || result.Trajs[0].Traj[j].current_lanelet.id() != result.Trajs[0].Traj[j-1].current_lanelet.id()){
            for (auto &ll: Simulator::mapreader->ConflictLane_[result.Trajs[0].Traj[j].current_lanelet.id()]){
                    result.Trajs[0].confilictlanes.push_back(ll);
                    //printf("$$$ DEBUG | %d\n", int(ll.id()));
            }
        }
    
    return result;
}