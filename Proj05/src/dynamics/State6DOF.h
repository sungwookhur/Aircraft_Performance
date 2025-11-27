#pragma once
#include "math/Vec3.h"

namespace aero::dynamics {

struct State6DOF {
    aero::math::Vec3 pos_NED;    // position in NED frame
    aero::math::Vec3 vel_body;   // velocity in body frame
    aero::math::Vec3 euler;      // (phi, theta, psi)
    aero::math::Vec3 omega_body; // (p, q, r)
};

struct StateDerivative6DOF {
    aero::math::Vec3 pos_dot;
    aero::math::Vec3 vel_dot;
    aero::math::Vec3 euler_dot;
    aero::math::Vec3 omega_dot;
};

} // namespace aero::dynamics
