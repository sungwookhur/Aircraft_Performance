// env/Gravity.hpp
#pragma once
#include "math/Vec3.h"

namespace aero::env {

class Gravity {
public:
    static constexpr double g = 9.80665;

    static math::Vec3 force_in_inertial(double mass) {
        return {0.0, 0.0, mass * g * (-1.0)}; // 예: NED에서 아래 방향이 + 이면 부호 조정
    }
};

} // namespace aero::env
