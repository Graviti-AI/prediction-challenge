#include <iostream>
#include <string.h>
#include "core/simulator.hpp"
#include "service/service.h"

#include <typeinfo>

static int print_help(){
    std::cout<<"Useage: simulator -p <port> -r <rviz_port> -h <host_adress> -c <configuration_file>"<<std::endl;
    return -1;
}

int main(int argc, char *argv[])
{
    int port = 50051;
    int rviz_port = -1;
    const char* host = "0.0.0.0";
    const char* config_file = "";

    for(int i=0; i<argc; ++i) {
        auto arg = argv[i];

        if(strcmp(arg, "-h") == 0) {
            return print_help();
        } else if (strcmp(arg, "-p") == 0) {
            if (i < argc-1) {
                port = atoi(argv[i+1]);
            } else {
                return print_help();
            }
        } else if (strcmp(arg, "-h") == 0) {
            if (i<argc-1) {
                host = argv[i+1];
            } else {
                return print_help();
            }
        } else if (strcmp(arg, "-r") == 0) {
            if (i < argc-1) {
                rviz_port = atoi(argv[i+1]);
            } else {
                return print_help();
            }
        } else if (strcmp(arg, "-c") == 0) {
            if (i < argc-1) {
                config_file = argv[i+1];
            } else {
                return print_help();
            }
        }
    }

    auto simu = core::create_simulator(rviz_port);
    Service svc(simu);

    return svc.run(host, port, config_file);
}

