// model/Aircraft.hpp
#pragma once
#include <vector>
#include <memory>

#include "AircraftParams.h"
#include "AerodynamicSurface.h"
#include "Propulsion.h"
#include "dynamics/State6DOF.h"
#include "dynamics/ControlInput.h"
#include "dynamics/ForcesMoments.h"
#include "env/Atmosphere.h"
#include "env/WindModel.h"
#include "env/Gravity.h"

namespace aero::model {

class Aircraft {
public:
    explicit Aircraft(const AircraftParams& params);

    dynamics::ForcesMoments computeTotalForcesMoments(
        const dynamics::State6DOF& x,
        const dynamics::ControlInput& u,
        const env::Atmosphere& atm
    ) const;

    const AircraftParams& params() const { return params_; }

private:
    AircraftParams params_;
    std::vector<std::unique_ptr<AerodynamicSurface>> surfaces_; // wing, tail, fuselage 등
    Propulsion propulsion_;
};

} // namespace aero::model
