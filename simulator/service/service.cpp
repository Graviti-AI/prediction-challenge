#include "service.h"
#include "service/impl/service_impl.h"
#include <memory>
#include <grpc++/grpc++.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

Service::Service(core::Simulator* simulator):
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

int Service::run(std::string address, int port)
{
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();

    m_impl = new ServiceImpl(m_simulator);

    char buf[100];
    sprintf(buf, "%s:%d", address.c_str(), port);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(buf, grpc::InsecureServerCredentials());
    builder.RegisterService(m_impl);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();
    return 0;
}
