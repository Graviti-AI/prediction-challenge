#include "core/simulator.h"
#include "service/service.h"

int main(int /*argc*/, char */*argv*/[])
{
    auto simu = core::create_simulator();
    Service svc(simu);
    return svc.run();
}
