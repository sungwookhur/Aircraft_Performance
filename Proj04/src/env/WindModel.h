// env/WindModel.hpp
#pragma once
#include "math/Vec3.h"

namespace aero::env {

struct WindState {
    math::Vec3 wind_NED;  // 바람 속도 (NED frame)
};

class WindModel {
public:
    WindState evaluate(double t) const {
        WindState w;
        // TODO: simple constant wind or turbulence model
        return w;
    }
};

} // namespace aero::env
