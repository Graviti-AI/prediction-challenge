#include <iostream>
#include <string.h>
#include "core/simulator.hpp"
#include "service/service.h"

#include <typeinfo>

static int print_help(){
    std::cout<<"Useage: simulator -p <port> -r <rviz_port> -h <host_adress> -c <configuration_file> -l <path_to_save_logs> --scenario-id <scenario_id> --scenario-name <scenario_name>"<<std::endl;
    return -1;
}

int main(int argc, char *argv[])
{
    int port = 50051;
    int rviz_port = -1;
    const char* host = "0.0.0.0";
    const char* config_file = "../conf/config.txt";
    const char* log_folder = "../Log";
    const char *scenario_id = getenv("SCENARIO_ID");
    const char *scenario_name = getenv("SCENARIO_NAME");

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
        } else if (strcmp(arg, "-l") == 0) {
            if (i < argc-1) {
                log_folder = argv[i+1];
            } else {
                return print_help();
            }
        } else if (strcmp(arg, "--scenario-id") == 0) {
            if (i < argc-1) {
                scenario_id = argv[i+1];
            } else {
                return print_help();
            }
        } else if (strcmp(arg, "--scenario-name") == 0) {
            if (i < argc-1) {
                scenario_name = argv[i+1];
            } else {
                return print_help();
            }
        }
    }
    if (!scenario_id){
        scenario_id = "";
    }
    if(!scenario_name) {
        scenario_name = "";
    }

    auto simu = core::create_simulator(rviz_port);
    Service svc(simu);

    auto scenario = core::SimulationScenario(scenario_id, scenario_name);

    return svc.run(scenario, host, port, config_file, log_folder);
}

