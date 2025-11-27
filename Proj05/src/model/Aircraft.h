#pragma once
#include <vector>
#include <memory>

#include "AircraftParams.h"
#include "AerodynamicSurface.h"
#include "Wing.h"
#include "Tail.h"
#include "Fuselage.h"
#include "Propulsion.h"
#include "dynamics/State6DOF.h"
#include "dynamics/ControlInput.h"
#include "dynamics/ForcesMoments.h"
#include "env/Atmosphere.h"

namespace aero::model {

class Aircraft {
public:
    explicit Aircraft(const AircraftParams& params);

    aero::dynamics::ForcesMoments computeTotalForcesMoments(
        const aero::dynamics::State6DOF& x,
        const aero::dynamics::ControlInput& u,
        const aero::env::Atmosphere& atm
    ) const;

    const AircraftParams& params() const { return params_; }

private:
    AircraftParams params_;
    std::vector<std::unique_ptr<AerodynamicSurface>> surfaces_;
    Propulsion propulsion_;
};

} // namespace aero::model
