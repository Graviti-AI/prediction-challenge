#pragma once
#include "service/proto/simulator.grpc.pb.h"

namespace core {
class Simulator;
}

class ServiceImpl:public service::SimulatorServer::Service {
public:
    ServiceImpl(core::Simulator* simulator);
    virtual ~ServiceImpl();


    // Service interface
private:
    grpc::Status FetchEnv(grpc::ServerContext *, const service::FetchEnvRequest *, service::FetchEnvResponse *);
    grpc::Status PushMyTrajectory(grpc::ServerContext *, const service::PushMyTrajectoryRequest *, service::PushMyTrajectoryResponse *);

private:
    core::Simulator* m_simulator;
};
