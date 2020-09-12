#include "service.h"
#include "service/impl/service_impl.h"

#include <stdio.h>
#include <memory>
#include <grpc++/grpc++.h>
#include <grpcpp/health_check_service_interface.h>


Service::Service(core::MySimulator* simulator):
    m_simulator(simulator)
{

}

Service::~Service()
{

}

bool Service::initialize()
{
    return true;
}

int Service::run(std::string address, int port, std::string config_file)
{
    grpc::EnableDefaultHealthCheckService(true);

    char buf[100];
    sprintf(buf, "%s:%d", address.c_str(), port);

    printf("$$$$$$$ Server Runs on %s, config: %s $$$$$$$$$\n", buf, config_file.c_str());

    m_impl = new ServiceImpl(m_simulator);
    m_simulator->start(config_file);   //Generate initial cars

    printf("Binding ......\n");

    grpc::ServerBuilder builder;
    builder.AddListeningPort(buf, grpc::InsecureServerCredentials());
    builder.RegisterService(m_impl);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();
    return 0;
}

