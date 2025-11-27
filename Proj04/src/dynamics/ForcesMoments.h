// dynamics/ForcesMoments.hpp
#pragma once
#include "math/Vec3.h"

namespace aero::dynamics {

struct ForcesMoments {
    math::Vec3 force_body;   // [Fx, Fy, Fz] in body
    math::Vec3 moment_body;  // [Mx, My, Mz] in body
};

} // namespace aero::dynamics
