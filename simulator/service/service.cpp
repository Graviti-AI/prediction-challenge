#include "service.h"
#include "service/impl/service_impl.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
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

static bool is_directory(const char *dirname)
{
    struct stat file_stat;
    if (stat(dirname, &file_stat))
    {
        return false;
    }

    return S_ISDIR(file_stat.st_mode) != 0;
}

static bool mkdir(const char *dirname)
{
    if (mkdir(dirname, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH))
    {
        int k = errno;
        return false;
    }
    return true;
}

int Service::run(core::SimulationScenario& scenario, std::string address, int port, const std::string &config_file, const std::string &log_folder)
{
    grpc::EnableDefaultHealthCheckService(true);

    char buf[100];
    sprintf(buf, "%s:%d", address.c_str(), port);

    printf("$$$$$$$ Server Runs on %s, config: %s $$$$$$$$$, log folder: %s\n", buf, config_file.c_str(), log_folder.c_str());
    if (!is_directory(log_folder.c_str()) && !mkdir(log_folder.c_str())) {
        printf("can't touch log folder %s", log_folder.c_str());
    }
    m_impl = new ServiceImpl(m_simulator);
    m_simulator->start(scenario, config_file, log_folder); //Generate initial cars

    printf("Binding ......\n");

    grpc::ServerBuilder builder;
    builder.AddListeningPort(buf, grpc::InsecureServerCredentials());
    builder.RegisterService(m_impl);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();
    return 0;
} 
