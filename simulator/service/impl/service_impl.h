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
    static void ProtoTrajToTraj(const service::Trajectory& protoTraj, core::Trajectory* coreTraj);
    // Service interface

    static void ObstacleToProtoObstacle(core::Obstacle &coreObstacle, service::Obstacle* protoObstacle);


private:
    grpc::Status FetchEnv(grpc::ServerContext *, const service::FetchEnvRequest *, service::FetchEnvResponse *);
    grpc::Status PushMyTrajectory(grpc::ServerContext *, const service::PushMyTrajectoryRequest *, service::PushMyTrajectoryResponse *);

public:
    core::MySimulator* m_simulator;
};
