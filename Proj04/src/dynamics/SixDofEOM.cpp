// dynamics/SixDofEOM.cpp
#include "SixDofEOM.h"

namespace aero::dynamics {

StateDerivative6DOF SixDofEOM::evaluate(
    const State6DOF& x,
    const ControlInput& u,
    const env::Atmosphere& atm
) const
{
    StateDerivative6DOF xdot;

    // 1) total forces/moments from aircraft
    auto totalFM = aircraft_.computeTotalForcesMoments(x, u, atm);

    // 2) pos_dot = R_body_to_inertial * vel_body
    // TODO: euler -> rotation matrix, multiply

    // 3) vel_dot = (1/m) * F_body - omega x v 등의 식 적용
    // 4) euler_dot = T(euler) * omega  (또는 quaternion 기반)
    // 5) omega_dot = I^{-1} ( M - omega x (I * omega) )

    return xdot;
}

} // namespace aero::dynamics

