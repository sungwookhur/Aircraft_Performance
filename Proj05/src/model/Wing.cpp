#include "Wing.h"
#include "math/Vec3.h"

namespace aero::model {

aero::dynamics::ForcesMoments Wing::computeForcesMoments(
    const aero::dynamics::State6DOF& x,
    const aero::dynamics::ControlInput& u,
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

    double alpha = std::atan2(v.z, v.x);           // angle of attack
    // double beta = std::asin(v.y / V);           // sideslip (unused here)

    double q_dyn = 0.5 * atm.rho * V * V;

    double CL = coeffs_.CL0
              + coeffs_.CL_alpha * alpha
              + coeffs_.CL_de * u.elevator;

    double CD = coeffs_.CD0 + coeffs_.k * CL * CL;

    double Cm = coeffs_.Cm0
              + coeffs_.Cm_alpha * alpha
              + coeffs_.Cm_de * u.elevator;

    double L = q_dyn * S_ * CL;
    double D = q_dyn * S_ * CD;

    // Simple model: drag along -x, lift along -z in body frame
    fm.force_body.x += -D;
    fm.force_body.z += -L;

    // Pitching moment about body-y
    fm.moment_body.y += q_dyn * S_ * c_ * Cm;

    return fm;
}

} // namespace aero::model
