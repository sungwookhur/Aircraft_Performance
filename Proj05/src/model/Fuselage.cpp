#include "Fuselage.h"
#include "math/Vec3.h"

namespace aero::model {

aero::dynamics::ForcesMoments Fuselage::computeForcesMoments(
    const aero::dynamics::State6DOF& x,
    const aero::dynamics::ControlInput&,
    const aero::env::Atmosphere& atm
) const
{
    using aero::math::Vec3;
    using aero::math::norm;

    aero::dynamics::ForcesMoments fm;
    fm.force_body = Vec3(0.0, 0.0, 0.0);
    fm.moment_body = Vec3(0.0, 0.0, 0.0);

    const Vec3& v = x.vel_body;
    double V = norm(v);
    if (V < 1e-3) {
        return fm;
    }

    double q_dyn = 0.5 * atm.rho * V * V;
    double D = q_dyn * S_fus_ * CD0_;

    fm.force_body.x += -D;
    return fm;
}

} // namespace aero::model
