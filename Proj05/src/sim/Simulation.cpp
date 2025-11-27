#include "Simulation.h"
#include "math/Vec3.h"

namespace aero::sim {

void Simulation::step(double dt) {
    auto xdot = eom_.evaluate(state_, input_, atm_);

    state_.pos_NED    = state_.pos_NED    + xdot.pos_dot    * dt;
    state_.vel_body   = state_.vel_body   + xdot.vel_dot    * dt;
    state_.euler      = state_.euler      + xdot.euler_dot  * dt;
    state_.omega_body = state_.omega_body + xdot.omega_dot  * dt;
}

} // namespace aero::sim
