#pragma once
#include "service/proto/simulator.grpc.pb.h"
#include "core/trajectory.hpp"

namespace core {
class MySimulator;
}

class ServiceImpl:public service::SimulatorServer::Service {
public:
    ServiceImpl(core::MySimulator* simulator);
    virtual ~ServiceImpl();

private:
    static void TrajToProtoTraj(core::Trajectory& coreTraj, service::Trajectory* protoTraj);
    // convert `core::Trajectory` to `service::Trajectory`

    static void ProtoTrajToTraj(const service::Trajectory& protoTraj, core::Trajectory* coreTraj);
    // convert `service::Trajectory` to `core::Trajectory`

private:
    grpc::Status FetchEnv(grpc::ServerContext *, const service::FetchEnvRequest *, service::FetchEnvResponse *);
    // fetch the input of client
    
    grpc::Status PushMyTrajectory(grpc::ServerContext *, const service::PushMyTrajectoryRequest *, service::PushMyTrajectoryResponse *);
    // upload the prediction results

public:
    core::MySimulator* m_simulator;
};
