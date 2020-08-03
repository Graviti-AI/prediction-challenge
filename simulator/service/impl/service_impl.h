#pragma once
#include "service/proto/simulator.grpc.pb.h"

namespace core {
class MySimulator;
}

class ServiceImpl:public service::SimulatorServer::Service {
public:
    ServiceImpl(core::MySimulator* simulator);
    virtual ~ServiceImpl();

    // Service interface
private:
    grpc::Status FetchEnv(grpc::ServerContext *, const service::FetchEnvRequest *, service::FetchEnvResponse *);
    grpc::Status PushMyTrajectory(grpc::ServerContext *, const service::PushMyTrajectoryRequest *, service::PushMyTrajectoryResponse *);

public:
    core::MySimulator* m_simulator;
};