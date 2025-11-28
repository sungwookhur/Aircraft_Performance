#include <iostream>
#include <iomanip>
#include <fstream>   // ★ 파일 출력용


#include "model/Aircraft.h"
#include "dynamics/SixDofEOM.h"
#include "sim/Simulation.h"

int main() {
    using namespace aero;

    model::AircraftParams params;
    params.mass = 1000.0;
    params.Ixx = 800.0;
    params.Iyy = 1000.0;
    params.Izz = 1200.0;
    params.S_ref = 16.2;
    params.c_bar = 1.5;
    params.b_span = 10.9;
    params.cg_body = math::Vec3(0.0, 0.0, 0.0);

    model::Aircraft aircraft(params);
    dynamics::SixDofEOM eom(aircraft);
    sim::Simulation sim(eom);

    dynamics::State6DOF x0;
    x0.pos_NED = math::Vec3(0.0, 0.0, -1000.0); // 1000 m altitude (NED: negative up)
    x0.vel_body = math::Vec3(70.0, 0.0, 0.0);   // 70 m/s forward
    x0.euler = math::Vec3(0.0, 0.0, 0.0);       // level flight
    x0.omega_body = math::Vec3(0.0, 0.0, 0.0);

    sim.setState(x0);

    dynamics::ControlInput u;
    u.throttle = 0.6;
    u.elevator = 0.0;
    u.aileron  = 0.0;
    u.rudder   = 0.0;
    sim.setControlInput(u);

    // 5) 출력 파일 열기
    std::ofstream log("sim_output.txt");
    if (!log) {
        std::cerr << "Failed to open sim_output.txt\n";
        return 1;
    }

    auto print_state = [&](std::ostream& os, double t,
                           const dynamics::State6DOF& xs) {
        os << std::fixed << std::setprecision(3)
           << "t=" << t
           << " pos_NED=(" << xs.pos_NED.x << ", "
                           << xs.pos_NED.y << ", "
                           << xs.pos_NED.z << ")"
           << " vel_body=(" << xs.vel_body.x << ", "
                            << xs.vel_body.y << ", "
                            << xs.vel_body.z << ")"
           << " euler=(" << xs.euler.x << ", "
                         << xs.euler.y << ", "
                         << xs.euler.z << ")"
           << '\n';
    };

    double dt = 0.01;
    double t_end = 5.0;
    int steps = static_cast<int>(t_end / dt);

    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        const auto& xs = sim.state();

        // 콘솔 출력
        print_state(std::cout, t, xs);
        // 파일 출력
        print_state(log, t, xs);

        // std::cout << std::fixed << std::setprecision(3)
        //           << "t=" << t
        //           << " pos_NED=(" << xs.pos_NED.x << ", " << xs.pos_NED.y << ", " << xs.pos_NED.z << ")"
        //           << " vel_body=(" << xs.vel_body.x << ", " << xs.vel_body.y << ", " << xs.vel_body.z << ")"
        //           << " euler=(" << xs.euler.x << ", " << xs.euler.y << ", " << xs.euler.z << ")"
        //           << std::endl;

        sim.step(dt);
    }

    return 0;
}
