// model/Wing.hpp
#pragma once
#include "AerodynamicSurface.h"

namespace aero::model {

class Wing : public AerodynamicSurface {
public:
    // 날개 고유 파라미터 (CL_alpha, CD0 등) 멤버로 가정
    // ...

    dynamics::ForcesMoments computeForcesMoments(
        const dynamics::State6DOF& x,
        const dynamics::ControlInput& u,
        const env::Atmosphere& atm
    ) const override;
};

} // namespace aero::model
