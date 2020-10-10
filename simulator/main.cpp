#include <stdio.h>
#include <fcntl.h>
#if defined(__APPLE__) || defined(__FreeBSD__)
#include <copyfile.h>
#else
#include <sys/sendfile.h>
#endif
#include <unistd.h>
#include <sys/time.h>
#include <time.h> /* clock_t, clock, CLOCKS_PER_SEC */
#include <iostream>
#include <string.h>
#include "core/simulator.hpp"
#include "service/service.h"

#include <typeinfo>

static int print_help()
{
    std::cout << "Useage: simulator -p <port> -r <rviz_port> -h <host_adress> -c <configuration_file> -l <path_to_save_logs> --scenario-id <scenario_id> --scenario-name <scenario_name>" << std::endl;
    return -1;
}

static int copy_file(const char *source, const char *destination)
{    
    int input, output;    
    if ((input = open(source, O_RDONLY)) == -1)
    {
        return -1;
    }    
    if ((output = creat(destination, 0660)) == -1)
    {
        close(input);
        return -1;
    }

    //Here we use kernel-space copying for performance reasons
#if defined(__APPLE__) || defined(__FreeBSD__)
    //fcopyfile works on FreeBSD and OS X 10.5+ 
    int result = fcopyfile(input, output, 0, COPYFILE_ALL);
#else
    //sendfile will work with non-socket output (i.e. regular file) on Linux 2.6.33+
    off_t bytesCopied = 0;
    struct stat fileinfo = {0};
    fstat(input, &fileinfo);
    int result = sendfile(output, input, &bytesCopied, fileinfo.st_size);
#endif

    close(input);
    close(output);

    return result;
}

int main(int argc, char *argv[])
{
    int port = 50051;
    int rviz_port = -1;
    const char *host = "0.0.0.0";
    const char *config_file = "../conf/config.txt";
    const char *log_folder = "../Log";
    const char *scenario_id = getenv("SCENARIO_ID");
    const char *scenario_name = getenv("SCENARIO_NAME");

    for (int i = 0; i < argc; ++i)
    {
        auto arg = argv[i];

        if (strcmp(arg, "-h") == 0)
        {
            return print_help();
        }
        else if (strcmp(arg, "-p") == 0)
        {
            if (i < argc - 1)
            {
                port = atoi(argv[i + 1]);
            }
            else
            {
                return print_help();
            }
        }
        else if (strcmp(arg, "-h") == 0)
        {
            if (i < argc - 1)
            {
                host = argv[i + 1];
            }
            else
            {
                return print_help();
            }
        }
        else if (strcmp(arg, "-r") == 0)
        {
            if (i < argc - 1)
            {
                rviz_port = atoi(argv[i + 1]);
            }
            else
            {
                return print_help();
            }
        }
        else if (strcmp(arg, "-c") == 0)
        {
            if (i < argc - 1)
            {
                config_file = argv[i + 1];
            }
            else
            {
                return print_help();
            }
        }
        else if (strcmp(arg, "-l") == 0)
        {
            if (i < argc - 1)
            {
                log_folder = argv[i + 1];
            }
            else
            {
                return print_help();
            }
        }
        else if (strcmp(arg, "--scenario-id") == 0)
        {
            if (i < argc - 1)
            {
                scenario_id = argv[i + 1];
            }
            else
            {
                return print_help();
            }
        }
        else if (strcmp(arg, "--scenario-name") == 0)
        {
            if (i < argc - 1)
            {
                scenario_name = argv[i + 1];
            }
            else
            {
                return print_help();
            }
        }
    }
    if (!scenario_id)
    {
        scenario_id = "";
    }
    if (!scenario_name)
    {
        scenario_name = "";
    }

    // make a copy of configuarion file for metrics
    {
        timeval T_now;
        tm *area;
        gettimeofday(&T_now, NULL);
        area = localtime(&(T_now.tv_sec));

        char *format_area = asctime(area);
        format_area[strcspn(format_area, "\n")] = '\0';

        for (int i = 0; i < strlen(format_area); i++)
            if (format_area[i] == ' ')
                format_area[i] = '_';

        char new_config_file[255];
        sprintf(new_config_file, "%s/scenario%s_config_%s.txt", log_folder, scenario_id, format_area);

        int res = copy_file(config_file, new_config_file);
        if (res)
        {
            std::cout << "failed to save config file from " << config_file << " to " << new_config_file << std::endl;
            exit(-1);
        }
    }

    auto simu = core::create_simulator(rviz_port);
    Service svc(simu);

    auto scenario = core::SimulationScenario(scenario_id, scenario_name);

    return svc.run(scenario, host, port, config_file, log_folder);
}
