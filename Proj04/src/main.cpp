#include "model/Aircraft.h"
#include "dynamics/SixDofEOM.h"
#include "sim/Simulation.h"

int main() {
    using namespace aero;

    model::AircraftParams params;
    params.mass = 1000.0;
    // TODO: inertia 등 설정

    model::Aircraft aircraft(params);
    dynamics::SixDofEOM eom(aircraft);
    sim::Simulation sim(eom);

    dynamics::State6DOF x0;
    // 초기 상태 설정
    sim.setState(x0);

    double dt = 0.01;
    for (int i = 0; i < 10000; ++i) {
        sim.step(dt);
    }

    return 0;
}
