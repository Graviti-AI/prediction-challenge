#include "service.h"
#include "service/impl/service_impl.h"
#include <memory>
#include <grpc++/grpc++.h>

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

int Service::run()
{
    std::string address("0.0.0.0:50051");

    grpc::ServerBuilder builder;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    m_impl = new ServiceImpl(m_simulator);
    builder.RegisterService(m_impl);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();
    return 0;
}
