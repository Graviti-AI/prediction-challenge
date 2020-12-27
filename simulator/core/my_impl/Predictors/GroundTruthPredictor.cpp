//
// Created by SYF on 10/7/20.
//

#include "GroundTruthPredictor.hpp"
#include "../Simulator/Simulator.hpp"
#include <assert.h>


GroundTruthPredictor::GroundTruthPredictor(Agent* agent_ibt, double time_step, double horizon): Predictor(agent_ibt,time_step,horizon){
}

PredictorType GroundTruthPredictor::getType() const{
    return PredictorType::GroundTruthPredictor;
}

void GroundTruthPredictor::set_traj(PredictTra traj){
    assert(false);  // python client wouldn't send back to this predictor.
}

PredictTra GroundTruthPredictor::update(Vector currentState,std::vector<Agent*> agents){
    assert(state == PredictorState::fine);
    state = PredictorState::wait4update;    //just wait4update

    PredictTra result;
    OneTra inittraj;
    auto mapinfo_ = agent_ibt_->mapinfo;

    result.Trajs.push_back(inittraj);
    assert(agent_ibt_->planner_buffer.size() == 31);
    // `planner_buffer` is calculated after the planning is done. `planner_buffer` stores the ground truth.

    for (auto state: agent_ibt_->planner_buffer){
        TraPoints initpoint;

        //change data type from vector<double> to TraPoints
        initpoint.t = 0.1 * result.Trajs[0].Traj.size();  //the interval of predictor is 0.1s
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
    assert(result.Trajs[0].Traj.size() == 31);

    result.Trajs[0].Probability = 1.0;

    // conflict lanes
    for (int j = 0; j < result.Trajs[0].Traj.size(); j ++)
        if (j == 0 || result.Trajs[0].Traj[j].current_lanelet.id() != result.Trajs[0].Traj[j-1].current_lanelet.id()){
            for (auto &ll: Simulator::mapreader->ConflictLane_[result.Trajs[0].Traj[j].current_lanelet.id()]){
                    result.Trajs[0].confilictlanes.push_back(ll);
            }
        }
    
    return result;
}