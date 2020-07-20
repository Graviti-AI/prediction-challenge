#include <iostream>
#include <string.h>
#include "core/simulator.h"
#include "service/service.h"

static int print_help(){
    std::cout<<"Useage: simulator -p <port>"<<std::endl;
    return -1;
}

int main(int argc, char *argv[])
{
    if(argc == 1) {
        return print_help();
    }
    int port = 50051;
    const char* host = "0.0.0.0";
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
        }
    }

    auto simu = core::create_simulator();
    Service svc(simu);
    return svc.run(host, port);
}
