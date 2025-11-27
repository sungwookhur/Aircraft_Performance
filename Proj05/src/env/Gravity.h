#pragma once
#include "math/Vec3.h"

namespace aero::env {

class Gravity {
public:
    static constexpr double g = 9.80665;

    // Gravity acceleration in NED frame (down positive)
    static aero::math::Vec3 accelerationNED() {
        return aero::math::Vec3(0.0, 0.0, g);
    }
};

} // namespace aero::env
