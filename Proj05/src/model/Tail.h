#pragma once
#include "AerodynamicSurface.h"

namespace aero::model {

class Tail : public AerodynamicSurface {
public:
    Tail() = default;

    aero::dynamics::ForcesMoments computeForcesMoments(
        const aero::dynamics::State6DOF& x,
        const aero::dynamics::ControlInput& u,
        const aero::env::Atmosphere& atm
    ) const override;
};

} // namespace aero::model
