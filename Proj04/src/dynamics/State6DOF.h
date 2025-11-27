// dynamics/State6DOF.hpp
#pragma once
#include "math/Vec3.h"

namespace aero::dynamics {

struct State6DOF {
    math::Vec3 pos_NED;    // 위치 (NED 혹은 ENU 중 선택)
    math::Vec3 vel_body;   // 비행체 속도 (body frame)
    math::Vec3 euler;      // roll, pitch, yaw (또는 quaternion으로 교체 가능)
    math::Vec3 omega_body; // 각속도 p,q,r
};

struct StateDerivative6DOF {
    math::Vec3 pos_dot;
    math::Vec3 vel_dot;
    math::Vec3 euler_dot;     // 또는 quaternion_dot
    math::Vec3 omega_dot;
};

} // namespace aero::dynamics
