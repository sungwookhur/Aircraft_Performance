#pragma once
#include "dynamics/State6DOF.h"
#include "dynamics/ControlInput.h"
#include "dynamics/ForcesMoments.h"
#include "env/Atmosphere.h"
#include "math/Vec3.h"

namespace aero::model {

class Propulsion {
public:
    explicit Propulsion(double maxThrust)
        : maxThrust_(maxThrust) {}

    aero::dynamics::ForcesMoments computeThrust(
        const aero::dynamics::State6DOF&,
        const aero::dynamics::ControlInput& u,
        const aero::env::Atmosphere&
    ) const {
        aero::dynamics::ForcesMoments fm;
        fm.force_body = aero::math::Vec3(maxThrust_ * u.throttle, 0.0, 0.0);
        fm.moment_body = aero::math::Vec3(0.0, 0.0, 0.0);
        return fm;
    }

private:
    double maxThrust_;
};

} // namespace aero::model
