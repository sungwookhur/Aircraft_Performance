#include "Tail.h"
#include "math/Vec3.h"

namespace aero::model {

aero::dynamics::ForcesMoments Tail::computeForcesMoments(
    const aero::dynamics::State6DOF&,
    const aero::dynamics::ControlInput&,
    const aero::env::Atmosphere&
) const
{
    aero::dynamics::ForcesMoments fm;
    fm.force_body = aero::math::Vec3(0.0, 0.0, 0.0);
    fm.moment_body = aero::math::Vec3(0.0, 0.0, 0.0);
    return fm;
}

} // namespace aero::model
