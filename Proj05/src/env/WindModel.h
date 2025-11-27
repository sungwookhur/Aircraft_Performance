#pragma once
#include "math/Vec3.h"

namespace aero::env {

struct WindState {
    aero::math::Vec3 wind_NED;
};

class WindModel {
public:
    WindState evaluate(double /*t*/) const {
        WindState w;
        w.wind_NED = aero::math::Vec3(0.0, 0.0, 0.0);
        return w;
    }
};

} // namespace aero::env
