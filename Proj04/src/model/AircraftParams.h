// model/AircraftParams.hpp
#pragma once
#include "math/Vec3.h"

namespace aero::model {

struct AircraftParams {
    double mass{0.0};
    // 관성 모멘트
    double Ixx{0.0};
    double Iyy{0.0};
    double Izz{0.0};
    double Ixz{0.0};

    // reference values
    double S_ref{0.0};     // wing reference area
    double c_bar{0.0};     // mean aerodynamic chord
    double b_span{0.0};    // wingspan

    // 무게 중심 위치, 각 서피스의 위치 등도 여기에
    math::Vec3 cg_body;   // CG position in body frame
};

} // namespace aero::model
