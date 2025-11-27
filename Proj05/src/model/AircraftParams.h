#pragma once
#include "math/Vec3.h"

namespace aero::model {

struct AircraftParams {
    double mass{1000.0};
    double Ixx{1000.0};
    double Iyy{1200.0};
    double Izz{1500.0};
    double Ixz{0.0};

    double S_ref{16.2};      // m^2
    double c_bar{1.5};       // m
    double b_span{10.9};     // m

    aero::math::Vec3 cg_body;
};

} // namespace aero::model
