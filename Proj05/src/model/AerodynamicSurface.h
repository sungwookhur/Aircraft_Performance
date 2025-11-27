#pragma once
#include "dynamics/State6DOF.h"
#include "dynamics/ControlInput.h"
#include "dynamics/ForcesMoments.h"
#include "env/Atmosphere.h"

namespace aero::model {

class AerodynamicSurface {
public:
    virtual ~AerodynamicSurface() = default;

    virtual aero::dynamics::ForcesMoments computeForcesMoments(
        const aero::dynamics::State6DOF& x,
        const aero::dynamics::ControlInput& u,
        const aero::env::Atmosphere& atm
    ) const = 0;
};

} // namespace aero::model
