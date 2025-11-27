#pragma once
#include <cmath>
#include "Mat3.h"

namespace aero::math {

// 3-2-1 (yaw-pitch-roll: psi, theta, phi)
// Euler = (phi, theta, psi)
inline Mat3 euler321ToMatrix(double phi, double theta, double psi) {
    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double cth  = std::cos(theta);
    double sth  = std::sin(theta);
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);

    Mat3 Rz;
    Rz.m[0][0] = cpsi; Rz.m[0][1] = -spsi; Rz.m[0][2] = 0.0;
    Rz.m[1][0] = spsi; Rz.m[1][1] =  cpsi; Rz.m[1][2] = 0.0;
    Rz.m[2][0] = 0.0;  Rz.m[2][1] =  0.0;  Rz.m[2][2] = 1.0;

    Mat3 Ry;
    Ry.m[0][0] =  cth; Ry.m[0][1] = 0.0; Ry.m[0][2] = sth;
    Ry.m[1][0] = 0.0;  Ry.m[1][1] = 1.0; Ry.m[1][2] = 0.0;
    Ry.m[2][0] = -sth; Ry.m[2][1] = 0.0; Ry.m[2][2] = cth;

    Mat3 Rx;
    Rx.m[0][0] = 1.0; Rx.m[0][1] = 0.0;  Rx.m[0][2] = 0.0;
    Rx.m[1][0] = 0.0; Rx.m[1][1] = cphi; Rx.m[1][2] = -sphi;
    Rx.m[2][0] = 0.0; Rx.m[2][1] = sphi; Rx.m[2][2] =  cphi;

    // Body -> NED : R_bn = Rz(psi)*Ry(theta)*Rx(phi)
    Mat3 R = Rz * (Ry * Rx);
    return R;
}

inline Mat3 euler321ToMatrix(const Vec3& euler) {
    return euler321ToMatrix(euler.x, euler.y, euler.z);
}

} // namespace aero::math
