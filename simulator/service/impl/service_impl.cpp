#include "service_impl.h"
#include "core/simulator.hpp"
#include "core/trajectory.hpp"

#include <typeinfo>

namespace core {
class MySimulator;
}

ServiceImpl::ServiceImpl(core::MySimulator *simulator)
    :m_simulator(simulator)
{

}

ServiceImpl::~ServiceImpl()
{

}

void ServiceImpl::TrajToProtoTraj(core::Trajectory &coreTraj, service::Trajectory* protoTraj)
{
    if (!protoTraj) {
        return;
    }
    for(auto pt : coreTraj){
        auto state = protoTraj->add_state();
        state->set_track_id(pt->track_id);
        state->set_frame_id(pt->frame_id);
        state->set_timestamp_ms(pt->timestamp_ms);
        state->set_agent_type(pt->agent_type);
        state->set_x(pt->x);
        state->set_y(pt->y);
        state->set_vx(pt->vx);
        state->set_vy(pt->vy);
        state->set_psi_rad(pt->psi_rad);
        state->set_length(pt->length);
        state->set_width(pt->width);
        state->set_jerk(pt->jerk);
        state->set_current_lanelet_id(pt->current_lanelet_id);
        state->set_s_of_current_lanelet(pt->s_of_current_lanelet);
        state->set_d_of_current_lanelet(pt->d_of_current_lanelet);
    }
}

void ServiceImpl::ProtoTrajToTraj(const service::Trajectory& protoTraj, core::Trajectory* coreTraj)
{
    if (!coreTraj) {
        return;
    }
    for (int i=0; i< protoTraj.state_size(); ++i) {
        const auto& pt= protoTraj.state(i);
        auto state = new core::State();
        state->track_id = pt.track_id();
        state->frame_id = pt.frame_id();
        state->timestamp_ms = pt.timestamp_ms();
        state->agent_type = pt.agent_type();
        state->x = pt.x();
        state->y = pt.y();
        state->vx = pt.vx();
        state->vy = pt.vy();
        state->psi_rad = pt.psi_rad();
        state->length = pt.length();
        state->width = pt.width();
        state->jerk = pt.jerk();
        state->current_lanelet_id = pt.current_lanelet_id();
        state->s_of_current_lanelet = pt.s_of_current_lanelet();
        state->d_of_current_lanelet = pt.d_of_current_lanelet();
        coreTraj->push_back(state);
    }
}

grpc::Status ServiceImpl::FetchEnv(grpc::ServerContext */*context*/,
                                   const service::FetchEnvRequest *request,
                                   service::FetchEnvResponse *response)
{
    const char* myTag = request->tag().c_str();

    if (strcmp(myTag, "planner") == 0) {

        auto env = m_simulator->fetchEnvPlanner();
        // TODO - how to deal with planner here?

        assert(false);
        /* Yaofeng:
            
            Now `env` contains those terms:
                std::vector<State> reference_points;
                std::vector<State> human_input;
                std::vector<State> obstacle_info;

            we need to add those terms to `service::FetchEnvResponse *response`, like this:
                state = response->reference_points().add_state();
                state->set_x(...)
                state->set_y(...)

            BTW, I recommend you to use `message Trajectory` to replace `repeated State` in the proto,
            since we already have `ProtoTrajToTraj` and `TrajToProtoTraj` functions, 
            which can help us convert `Trajectory` to `ProtoTraj`.

            Another Problem: `FetchEnvResponse` contains  `repeated Obstacle obstacle_info`,
            but now, the date type of `env.obstacle_info` is `std::vector<State> obstacle_info`. 
            So you need to define a new `struct obstacle`, and convert `obstacle` to `Proto obstacle` here.

            TODO List:
                1. Modify `simulator.proto`, replace `repeated State` with `Trajectory`.
                2. Add `env.planned_trajectory`, `env.reference_points`, `env.human_input` to `response`, Use `TrajToProtoTraj` to convert those terms.
                3. Define `struct obstacle` in `trajectory.hpp`, change the date type of `env.obstacle_info` to `vector<obstacle>`.
                4. Add `env.obstacle_info` to `response`, write a function `ObstacleToProtoObstacle` to convert `obstacle` to `Proto obstacle`.
                5. You can mimic the code `if (strcmp(myTag, "predictor") == 0)`.
        */

        if (env.paused) {
            printf("simulator paused\n");
            response->set_msg("simulator paused");
            response->set_resp_code(233);
        } else {
            printf("sending ok response to planner");
            response->set_msg("ok");
            response->set_resp_code(0);
        }
    
    } else if (strcmp(myTag, "predictor") == 0) {
        auto env = m_simulator->fetchEnvPredictor();

        // map name
        response->set_map_name(env.map_name);

        // my trajectory
        auto my_traj = new service::Trajectory();
        response->set_allocated_my_traj(my_traj);
        TrajToProtoTraj(env.my_traj, my_traj);

        // others trajectory
        for(auto otherTraj: env.other_trajs) {
            auto protoTraj = response->add_other_trajs();
            TrajToProtoTraj(otherTraj, protoTraj);
        }

        if (env.paused) {
            printf("simulator paused\n");
            response->set_msg("simulator paused");
            response->set_resp_code(233);
        } else {
            printf("sending ok response to predictor");
            response->set_msg("ok");
            response->set_resp_code(0);
        } 
    } else {
        response->set_msg("invalid request");
        response->set_resp_code(1);
    }
    return grpc::Status();

}

grpc::Status ServiceImpl::PushMyTrajectory(grpc::ServerContext */*context*/,
                                           const service::PushMyTrajectoryRequest *request,
                                           service::PushMyTrajectoryResponse *response)
{
    const char * myTag = request->tag().c_str();

    if (strcmp(myTag, "planner") == 0) {
        // planner;
        printf("Running code for planner...");
        core::Trajectory planned_traj;
        auto protoTraj = request->planned_traj();
        ProtoTrajToTraj(protoTraj, &planned_traj);
        if ( m_simulator->onPlannerState(std::move(planned_traj)) ) {
            response->set_msg("ok");
            response->set_resp_code(0);
        } else {
            response->set_msg("core failed");
            response->set_resp_code(-1);
        }

    } else if (strcmp(myTag, "predictor") == 0) {
        printf("Running code for predictor....");
        std::vector<core::Trajectory> pred_trajs;
        for(int i=0; i<request->pred_trajs().size(); ++i) {
            auto traj = core::Trajectory();
            auto protoTraj = request->pred_trajs().at(i);
            ProtoTrajToTraj(protoTraj, &traj);
            pred_trajs.emplace_back(traj);
        }

        std::vector<double> probabilities;
        for(int i=0; i<request->probability().size(); ++i) {
            probabilities.push_back(request->probability().at(i));
        }

        if (pred_trajs.size() == 0 && probabilities.size() == 0) {
            printf("The client closed\n");
            printf("The simulator closed\n");
            exit(0);
        }

        if ( m_simulator->onPredictorState(std::move(pred_trajs), std::move(probabilities)) ) {
            response->set_msg("ok");
            response->set_resp_code(0);
        } else {
            response->set_msg("core failed");
            response->set_resp_code(-1);
        }
    } else {
        printf("invalid request");
        response->set_msg("failed");
        response->set_resp_code(-1);
    }
    return grpc::Status();
}
