#pragma once
#include "math/Vec3.h"

namespace aero::dynamics {

struct ForcesMoments {
    aero::math::Vec3 force_body;
    aero::math::Vec3 moment_body;
};

} // namespace aero::dynamics
