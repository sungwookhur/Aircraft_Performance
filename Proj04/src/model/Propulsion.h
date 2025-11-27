// model/Propulsion.hpp
#pragma once
#include "dynamics/State6DOF.h"
#include "dynamics/ControlInput.h"
#include "dynamics/ForcesMoments.h"
#include "env/Atmosphere.h"

namespace aero::model {

class Propulsion {
public:
    dynamics::ForcesMoments computeThrust(
        const dynamics::State6DOF& x,
        const dynamics::ControlInput& u,
        const env::Atmosphere& atm
    ) const {
        dynamics::ForcesMoments fm;
        // TODO: thrust in body x-axis, maybe some moment due to offset
        return fm;
    }
};

} // namespace aero::model
