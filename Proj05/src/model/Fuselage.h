#pragma once
#include "AerodynamicSurface.h"

namespace aero::model {

class Fuselage : public AerodynamicSurface {
public:
    explicit Fuselage(double area, double cd0)
        : S_fus_(area), CD0_(cd0) {}

    aero::dynamics::ForcesMoments computeForcesMoments(
        const aero::dynamics::State6DOF& x,
        const aero::dynamics::ControlInput& u,
        const aero::env::Atmosphere& atm
    ) const override;

private:
    double S_fus_;
    double CD0_;
};

} // namespace aero::model
