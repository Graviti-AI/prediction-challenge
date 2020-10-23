#include "service.h"
#include "service/impl/service_impl.h"

#include <stdio.h>
#include <memory>
#include <grpc++/grpc++.h>
#include <grpcpp/health_check_service_interface.h>

Service::Service(core::MySimulator *simulator) : m_simulator(simulator)
{
}

Service::~Service()
{
}

bool Service::initialize()
{
    return true;
}


int Service::run(core::SimulationScenario& scenario, std::string address, int port, const std::string &config_file, const std::string &log_folder, const bool verbose)
{
    grpc::EnableDefaultHealthCheckService(true);

    char buf[100];
    sprintf(buf, "%s:%d", address.c_str(), port);

    printf("# server address %s, config: %s, log folder: %s\n", buf, config_file.c_str(), log_folder.c_str());

    m_impl = new ServiceImpl(m_simulator);
    m_simulator->start(scenario, config_file, log_folder, verbose); //Generate initial cars

    grpc::ServerBuilder builder;
    builder.AddListeningPort(buf, grpc::InsecureServerCredentials());
    builder.RegisterService(m_impl);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();
    return 0;
} 
