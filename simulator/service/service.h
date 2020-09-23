#pragma once
#include <string>
#include "core/simulator.hpp"

namespace core {
class MySimulator;
class MySimulatorImpl;
}

class ServiceImpl;

class Service {
public:
    Service(core::MySimulator* simulator);
    ~Service();
public:
    bool initialize();
    int run(std::string address, int port, const std::string &config_file, const std::string &log_folder);
    void shutdown();
private:
    ServiceImpl* m_impl;
    core::MySimulator *m_simulator;
};
