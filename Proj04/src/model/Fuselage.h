// model/Fuselage.hpp
#pragma once
#include "AerodynamicSurface.h"

namespace aero::model {

class Fuselage : public AerodynamicSurface {
public:
    dynamics::ForcesMoments computeForcesMoments(
        const dynamics::State6DOF& x,
        const dynamics::ControlInput& u,
        const env::Atmosphere& atm
    ) const override;
};

} // namespace aero::model
