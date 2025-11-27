// dynamics/SixDofEOM.hpp
#pragma once
#include "State6DOF.h"
#include "ControlInput.h"
#include "ForcesMoments.h"
#include "model/Aircraft.h"
#include "env/Atmosphere.h"

namespace aero::dynamics {

class SixDofEOM {
public:
    explicit SixDofEOM(const model::Aircraft& aircraft)
        : aircraft_(aircraft) {}

    StateDerivative6DOF evaluate(
        const State6DOF& x,
        const ControlInput& u,
        const env::Atmosphere& atm
    ) const;

private:
    const model::Aircraft& aircraft_;
};

} // namespace aero::dynamics
