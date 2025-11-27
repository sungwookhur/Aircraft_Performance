// model/AerodynamicSurface.hpp
#pragma once
#include "dynamics/State6DOF.h"
#include "dynamics/ControlInput.h"
#include "dynamics/ForcesMoments.h"
#include "env/Atmosphere.h"

namespace aero::model {

class AerodynamicSurface {
public:
    virtual ~AerodynamicSurface() = default;

    // state, input, 대기 상태, 상대풍 등으로부터
    // 이 서피스가 만드는 힘/모멘트를 계산 (body frame 기준)
    virtual dynamics::ForcesMoments computeForcesMoments(
        const dynamics::State6DOF& x,
        const dynamics::ControlInput& u,
        const env::Atmosphere& atm
    ) const = 0;
};

} // namespace aero::model
