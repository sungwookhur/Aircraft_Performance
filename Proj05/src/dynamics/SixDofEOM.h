#pragma once
#include "State6DOF.h"
#include "ControlInput.h"
#include "ForcesMoments.h"
#include "model/Aircraft.h"
#include "env/Atmosphere.h"
#include "env/Gravity.h"
#include "math/Rotation.h"
#include "math/Vec3.h"

namespace aero::dynamics {

class SixDofEOM {
public:
    explicit SixDofEOM(const aero::model::Aircraft& aircraft)
        : aircraft_(aircraft) {}

    StateDerivative6DOF evaluate(
        const State6DOF& x,
        const ControlInput& u,
        const aero::env::Atmosphere& atm
    ) const;

private:
    const aero::model::Aircraft& aircraft_;
};

} // namespace aero::dynamics
