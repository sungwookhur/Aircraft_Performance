#pragma once
#include "dynamics/SixDofEOM.h"
#include "env/Atmosphere.h"

namespace aero::sim {

class Simulation {
public:
    // explicit Simulation(aero::dynamics::SixDofEOM eom)
    //     : eom_(std::move(eom)) {}
    explicit Simulation(const aero::dynamics::SixDofEOM& eom)
        : eom_(eom) {}

    void step(double dt);

    void setState(const aero::dynamics::State6DOF& x) { state_ = x; }
    const aero::dynamics::State6DOF& state() const { return state_; }

    void setControlInput(const aero::dynamics::ControlInput& u) { input_ = u; }
    const aero::dynamics::ControlInput& controlInput() const { return input_; }

private:
    aero::dynamics::SixDofEOM eom_;
    aero::dynamics::State6DOF state_;
    aero::dynamics::ControlInput input_;
    aero::env::Atmosphere atm_;
};

} // namespace aero::sim
