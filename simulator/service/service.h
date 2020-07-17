#pragma once

namespace core {
class Simulator;
}

class ServiceImpl;
class Service {
public:
    Service(core::Simulator* simulator);
    ~Service();
public:
    bool initialize();
    int run();
    void shutdown();
private:
    ServiceImpl* m_impl;
    core::Simulator* m_simulator;
};
