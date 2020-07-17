#include "service_impl.h"
#include "core/simulator.h"

ServiceImpl::ServiceImpl(core::Simulator *simulator)
    :m_simulator(simulator)
{

}

ServiceImpl::~ServiceImpl()
{

}

grpc::Status ServiceImpl::FetchEnv(grpc::ServerContext */*context*/,
                                   const service::FetchEnvRequest */*request*/,
                                   service::FetchEnvResponse *response)
{
    auto traj = new service::Trajectory();
    for(auto pt : m_simulator->fetchEnv()){
        auto state = traj->add_state();
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
    }
    response->set_allocated_trajectory(traj);
    response->set_msg("ok");
    response->set_resp_code(0);

    return grpc::Status();
}

grpc::Status ServiceImpl::PushMyTrajectory(grpc::ServerContext */*context*/,
                                           const service::PushMyTrajectoryRequest *request,
                                           service::PushMyTrajectoryResponse *response)
{

    auto traj = core::Trajectory();
    for(int i=0; i<request->trajectory().state_size(); ++i) {
        auto state = request->trajectory().state(i);
        auto pt = new core::State();
        pt->track_id = state.track_id();
        pt->frame_id = state.frame_id();
        pt->timestamp_ms = state.timestamp_ms();
        pt->agent_type = state.agent_type();
        pt->x = state.x();
        pt->y = state.y();
        pt->vx = state.vx();
        pt->vy = state.vy();
        pt->psi_rad = state.psi_rad();
        pt->length = state.length();
        pt->width = state.width();
        traj.push_back(pt);
    }
    if (m_simulator->onUserState(traj)){
        response->set_msg("ok");
        response->set_resp_code(0);
    } else {
        response->set_msg("core failed");
        response->set_resp_code(-1);
    }
    return grpc::Status();
}
