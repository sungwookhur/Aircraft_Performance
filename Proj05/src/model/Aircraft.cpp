#include "Aircraft.h"
#include "math/Vec3.h"

namespace aero::model {

Aircraft::Aircraft(const AircraftParams& params)
    : params_(params),
      propulsion_(5000.0) // max thrust [N], arbitrary
{
    // Simple composition: one wing, one fuselage, one tail
    surfaces_.push_back(std::make_unique<Wing>(params_.S_ref, params_.c_bar));
    surfaces_.push_back(std::make_unique<Fuselage>(params_.S_ref * 0.7, 0.04));
    surfaces_.push_back(std::make_unique<Tail>());
}

aero::dynamics::ForcesMoments Aircraft::computeTotalForcesMoments(
    const aero::dynamics::State6DOF& x,
    const aero::dynamics::ControlInput& u,
    const aero::env::Atmosphere& atm
) const
{
    aero::dynamics::ForcesMoments total;
    total.force_body = aero::math::Vec3(0.0, 0.0, 0.0);
    total.moment_body = aero::math::Vec3(0.0, 0.0, 0.0);

    for (const auto& s : surfaces_) {
        auto fm = s->computeForcesMoments(x, u, atm);
        total.force_body += fm.force_body;
        total.moment_body += fm.moment_body;
    }

    auto thrust = propulsion_.computeThrust(x, u, atm);
    total.force_body += thrust.force_body;
    total.moment_body += thrust.moment_body;

    // Note: gravity is handled in dynamics, not here

    return total;
}

} // namespace aero::model
