#include "SixDofEOM.h"

namespace aero::dynamics {

StateDerivative6DOF SixDofEOM::evaluate(
    const State6DOF& x,
    const ControlInput& u,
    const aero::env::Atmosphere& atm
) const
{
    using aero::math::Vec3;
    using aero::math::Mat3;
    using aero::math::euler321ToMatrix;
    using aero::math::cross;

    StateDerivative6DOF xdot;

    // 1) Total aerodynamic + thrust forces/moments in body frame
    auto totalFM = aircraft_.computeTotalForcesMoments(x, u, atm);

    // 2) Rotation matrix from body to NED
    Mat3 R_bn = euler321ToMatrix(x.euler.x, x.euler.y, x.euler.z);
    Mat3 R_nb = aero::math::transpose(R_bn);

    // 3) Position derivative: NED velocity = R_bn * v_body
    xdot.pos_dot = R_bn * x.vel_body;

    // 4) Translational acceleration in body frame
    const auto& p = aircraft_.params();
    double m = p.mass;

    // Gravity acceleration in NED and converted to body frame (down positive)
    Vec3 g_NED = aero::env::Gravity::accelerationNED();
    Vec3 g_body = R_nb * g_NED;
    Vec3 F_gravity_body = g_body * m;

    // m v_dot = F_total + F_gravity - omega x (m v)
    Vec3 omega = x.omega_body;
    Vec3 mv = x.vel_body * m;
    Vec3 coriolis = cross(omega, mv);

    Vec3 totalForce = totalFM.force_body + F_gravity_body - coriolis;
    xdot.vel_dot = totalForce / m;

    // 5) Euler angle rates from body rates (3-2-1)
    double phi   = x.euler.x;
    double theta = x.euler.y;
    double psi   = x.euler.z;
    double p_b   = x.omega_body.x;
    double q_b   = x.omega_body.y;
    double r_b   = x.omega_body.z;

    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double cth  = std::cos(theta);
    double sth  = std::sin(theta);

    double tan_th = (std::abs(cth) < 1e-6) ? (sth > 0 ? 1e6 : -1e6) : sth / cth;

    double phi_dot   = p_b + q_b * sphi * tan_th + r_b * cphi * tan_th;
    double theta_dot = q_b * cphi - r_b * sphi;
    double psi_dot   = q_b * sphi / cth + r_b * cphi / cth;

    xdot.euler_dot = Vec3(phi_dot, theta_dot, psi_dot);

    // 6) Rotational dynamics with diagonal inertia approximation
    double Ixx = p.Ixx;
    double Iyy = p.Iyy;
    double Izz = p.Izz;

    double Mx = totalFM.moment_body.x;
    double My = totalFM.moment_body.y;
    double Mz = totalFM.moment_body.z;

    double p_dot = (Mx + (Iyy - Izz) * q_b * r_b) / Ixx;
    double q_dot = (My + (Izz - Ixx) * p_b * r_b) / Iyy;
    double r_dot = (Mz + (Ixx - Iyy) * p_b * q_b) / Izz;

    xdot.omega_dot = Vec3(p_dot, q_dot, r_dot);

    return xdot;
}

} // namespace aero::dynamics
