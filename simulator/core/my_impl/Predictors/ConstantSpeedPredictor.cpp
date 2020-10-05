//
// Created by LCR on 7/18/20.
//
#include "ConstantSpeedPredictor.hpp"
#include "../Simulator/Simulator.hpp"
#include <assert.h>


ConstantSpeedPredictor::ConstantSpeedPredictor(MapInfo* map,double time_step,double horizon): Predictor(map,time_step,horizon){
}

void ConstantSpeedPredictor::set_traj(PredictTra traj){
    assert(false);  // python client wouldn't send back to this predictor.
}

PredictTra ConstantSpeedPredictor::update(Vector currentState,std::vector<Agent*> agents){
    state = PredictorState::wait4update;

    BasicPoint2d currPos(currentState[0], currentState[1]);
    bool RoutingLineChange = mapinfo_->RoutingLineChange_;
    PredictTra result;
    OneTra inittraj;
    result.Trajs.push_back(inittraj);
    TraPoints initpoint;
    initpoint.t = 0;
    initpoint.x = currentState[0];
    initpoint.y = currentState[1];
    initpoint.theta = currentState[2];
    initpoint.v = std::sqrt(currentState[3]*currentState[3]+currentState[4]*currentState[4]);
    initpoint.a = 0;
    initpoint.jerk = 0;
    initpoint.current_lanelet = mapinfo_->getCurrentLanelet();
    initpoint.s_of_current_lanelet = mapinfo_->getS();
    initpoint.d_of_current_lanelet = geometry::toArcCoordinates(mapinfo_->getCurrentLanelet().centerline2d(), currPos).distance;
    //cout<<"begin predict"<<endl;
    result.Trajs[0].Traj.push_back(initpoint);
    for (auto &ll:Simulator::mapreader->ConflictLane_[initpoint.current_lanelet.id()]){
            result.Trajs[0].confilictlanes.push_back(ll);
    }
    double time_now = 0;

    while (time_now < horizon_)
    {
        time_now+=time_step_;
        for(int i=0;i<result.Trajs.size();i++){

            TraPoints one_point = result.Trajs[i].Traj.back();
            BasicPoint2d old_Pos(one_point.x,one_point.y);
            one_point.t = time_now;
            BasicPoint2d pAfter;
            if (RoutingLineChange && (!(mapinfo_->marge_first)) ){
                double s_now = geometry::toArcCoordinates(mapinfo_->mergingPath, currPos).length;
                double s_new = s_now +one_point.v*one_point.t;
                
                if (s_new > mapinfo_->original_merge_length) {
                    ConstLanelet next_lanelet = mapinfo_->findNextLanelet(one_point.current_lanelet);
                    if (one_point.current_lanelet.id()==next_lanelet.id()){
                        BasicPoint2d p_b = geometry::interpolatedPointAtDistance(one_point.current_lanelet.centerline2d(),geometry::length2d(one_point.current_lanelet));
                        BasicPoint2d p_f = geometry::interpolatedPointAtDistance(one_point.current_lanelet.centerline2d(),geometry::length2d(one_point.current_lanelet)-0.1);
                        BasicPoint2d one_add = p_b-p_f;
                        one_point.x = p_b.x() + 10*(s_new-mapinfo_->original_merge_length)*one_add.x();
                        one_point.y = p_b.y() + 10*(s_new-mapinfo_->original_merge_length)*one_add.y();
                        one_point.theta = std::atan2(one_add.y(),one_add.x());
                        one_point.s_of_current_lanelet = s_new-mapinfo_->original_merge_length+geometry::length2d(one_point.current_lanelet);
                        one_point.d_of_current_lanelet = 0;
                    }
                    else {
                        pAfter = geometry::interpolatedPointAtDistance(next_lanelet.centerline2d(), s_new-mapinfo_->original_merge_length);
                        one_point.x = pAfter.x();
                        one_point.y = pAfter.y();
                        BasicPoint2d new_drection = pAfter - old_Pos;
                        one_point.theta = std::atan2(new_drection.y(),new_drection.x());
                        one_point.current_lanelet = next_lanelet;
                        one_point.s_of_current_lanelet = s_new-mapinfo_->original_merge_length;
                        one_point.d_of_current_lanelet = 0;
                        RoutingLineChange  = false;
                        for(auto &ll: Simulator::mapreader->ConflictLane_[one_point.current_lanelet.id()]){
                            result.Trajs[i].confilictlanes.push_back(ll);
                        }
                    }
                }
                else {
                    pAfter = geometry::interpolatedPointAtDistance(mapinfo_->mergingPath, s_new);
                    one_point.x = pAfter.x();
                    one_point.y = pAfter.y();
                    BasicPoint2d new_drection = pAfter - old_Pos;
                    one_point.theta = std::atan2(new_drection.y(),new_drection.x());
                    ConstLanelet next_lanelet = mapinfo_->findNextLanelet(one_point.current_lanelet);
                    if (mapinfo_->AsRoutingLineChangefirst(one_point.current_lanelet)){
                        double dis_f = geometry::toArcCoordinates(one_point.current_lanelet.centerline2d(), pAfter).distance;
                        double dis_s = geometry::toArcCoordinates(next_lanelet.centerline2d(), pAfter).distance;
                        if (abs(dis_f)>abs(dis_s)) one_point.current_lanelet = next_lanelet;
                    }
                    one_point.s_of_current_lanelet = geometry::toArcCoordinates(one_point.current_lanelet.centerline2d(), pAfter).length;
                    one_point.d_of_current_lanelet = geometry::toArcCoordinates(one_point.current_lanelet.centerline2d(), pAfter).distance;
                } 
                result.Trajs[i].Traj.push_back(one_point);
            }
            else{
                double s_now = one_point.s_of_current_lanelet;
                double s_new = s_now +one_point.v*time_step_;
                if (s_new > geometry::length2d(one_point.current_lanelet)) {
                    ConstLanelets followinglanelets = mapinfo_->routingGraphPtr_->following(one_point.current_lanelet);
                    if (followinglanelets.size()==0){
                        BasicPoint2d p_b = geometry::interpolatedPointAtDistance(one_point.current_lanelet.centerline2d(),geometry::length2d(one_point.current_lanelet));
                        BasicPoint2d p_f = geometry::interpolatedPointAtDistance(one_point.current_lanelet.centerline2d(),geometry::length2d(one_point.current_lanelet)-0.1);
                        BasicPoint2d one_add = p_b-p_f;
                        pAfter = p_b + 10*(s_new-geometry::length2d(one_point.current_lanelet))*one_add;
                        one_point.x = pAfter.x();
                        one_point.y = pAfter.y();
                        one_point.theta = std::atan2(one_add.y(),one_add.x());
                        one_point.s_of_current_lanelet = s_new;
                        one_point.d_of_current_lanelet = 0;
                        result.Trajs[i].Traj.push_back(one_point);
                    }
                    else if (followinglanelets.size()==1){
                        pAfter = geometry::interpolatedPointAtDistance(followinglanelets.front().centerline2d(), s_new-geometry::length2d(one_point.current_lanelet));
                        one_point.x = pAfter.x();
                        one_point.y = pAfter.y();
                        BasicPoint2d new_drection = pAfter - old_Pos;
                        one_point.theta = std::atan2(new_drection.y(),new_drection.x());
                        one_point.s_of_current_lanelet = s_new-geometry::length2d(one_point.current_lanelet);
                        one_point.current_lanelet = followinglanelets.front();
                        one_point.d_of_current_lanelet = 0;
                        result.Trajs[i].Traj.push_back(one_point);
                        for(auto &ll: Simulator::mapreader->ConflictLane_[one_point.current_lanelet.id()]){
                            result.Trajs[i].confilictlanes.push_back(ll);
                        }
                    }else{
                        for (int j = 1; j < followinglanelets.size(); j++)
                        {   
                            TraPoints new_point = one_point;
                            result.Trajs.push_back(result.Trajs[i]);
                            pAfter = geometry::interpolatedPointAtDistance(followinglanelets[j].centerline2d(), s_new-geometry::length2d(one_point.current_lanelet));
                            new_point.x = pAfter.x();
                            new_point.y = pAfter.y();
                            BasicPoint2d new_drection = pAfter - old_Pos;
                            new_point.theta = std::atan2(new_drection.y(),new_drection.x());
                            new_point.s_of_current_lanelet =  s_new-geometry::length2d(one_point.current_lanelet);
                            new_point.current_lanelet = followinglanelets[j];
                            new_point.d_of_current_lanelet = 0;
                            result.Trajs.back().Traj.push_back(new_point);
                            for(auto &ll: Simulator::mapreader->ConflictLane_[new_point.current_lanelet.id()]){
                                result.Trajs.back().confilictlanes.push_back(ll);
                            }
                        }
                        pAfter = geometry::interpolatedPointAtDistance(followinglanelets[0].centerline2d(), s_new-geometry::length2d(one_point.current_lanelet));
                        one_point.x = pAfter.x();
                        one_point.y = pAfter.y();
                        BasicPoint2d new_drection = pAfter - old_Pos;
                        one_point.theta = std::atan2(new_drection.y(),new_drection.x());
                        one_point.s_of_current_lanelet =  s_new-geometry::length2d(one_point.current_lanelet);
                        one_point.current_lanelet = followinglanelets[0];
                        one_point.d_of_current_lanelet = 0;
                        result.Trajs[i].Traj.push_back(one_point);
                        for(auto &ll: Simulator::mapreader->ConflictLane_[one_point.current_lanelet.id()]){
                            result.Trajs[i].confilictlanes.push_back(ll);
                        }
                    }
                }
                else{
                    pAfter = geometry::interpolatedPointAtDistance(one_point.current_lanelet.centerline2d(), s_new);
                    one_point.x = pAfter.x();
                    one_point.y = pAfter.y();
                    BasicPoint2d new_drection = pAfter - old_Pos;
                    one_point.theta = std::atan2(new_drection.y(),new_drection.x());
                    one_point.s_of_current_lanelet = s_new;
                    one_point.d_of_current_lanelet = 0;
                    result.Trajs[i].Traj.push_back(one_point);
                }
            }
        }
    }
    for(int i = 0; i<result.Trajs.size();i++){
        result.Trajs[i].Probability = 1.0/result.Trajs.size();
    }
    //cout<<"over!"<<endl;
    return result;

}