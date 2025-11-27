#pragma once

namespace aero::dynamics {

struct ControlInput {
    double throttle{0.0};   // 0~1
    double elevator{0.0};   // rad
    double aileron{0.0};    // rad
    double rudder{0.0};     // rad
};

} // namespace aero::dynamics
