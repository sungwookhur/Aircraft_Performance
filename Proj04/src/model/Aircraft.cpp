// model/Aircraft.cpp
#include "Aircraft.h"

namespace aero::model {

Aircraft::Aircraft(const AircraftParams& params)
    : params_(params)
{
    // 예시: 생성자에서 서피스들 구성
    // surfaces_.push_back(std::make_unique<Wing>(...));
    // surfaces_.push_back(std::make_unique<Tail>(...));
    // surfaces_.push_back(std::make_unique<Fuselage>(...));
}

dynamics::ForcesMoments Aircraft::computeTotalForcesMoments(
    const dynamics::State6DOF& x,
    const dynamics::ControlInput& u,
    const env::Atmosphere& atm
) const
{
    dynamics::ForcesMoments total;
    total.force_body  = {0.0, 0.0, 0.0};
    total.moment_body = {0.0, 0.0, 0.0};

    // 1) 각 서피스의 힘/모멘트 합산: f_wing, f_tail, f_fuselage ...
    for (const auto& s : surfaces_) {
        auto fm = s->computeForcesMoments(x, u, atm);
        // 여기서 위치 오프셋에 따른 모멘트 추가도 고려 가능
        total.force_body.x  += fm.force_body.x;
        total.force_body.y  += fm.force_body.y;
        total.force_body.z  += fm.force_body.z;
        total.moment_body.x += fm.moment_body.x;
        total.moment_body.y += fm.moment_body.y;
        total.moment_body.z += fm.moment_body.z;
    }

    // 2) 추진력
    auto thrust = propulsion_.computeThrust(x, u, atm);
    total.force_body.x  += thrust.force_body.x;
    total.force_body.y  += thrust.force_body.y;
    total.force_body.z  += thrust.force_body.z;
    total.moment_body.x += thrust.moment_body.x;
    total.moment_body.y += thrust.moment_body.y;
    total.moment_body.z += thrust.moment_body.z;

    // 3) 중력 (inertial -> body frame 로테이션은 여기서 처리)
    // 일단 TODO: 회전 변환 후 body frame 으로 더하기

    return total;
}

} // namespace aero::model
