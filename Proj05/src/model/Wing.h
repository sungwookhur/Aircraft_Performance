#pragma once
#include "AerodynamicSurface.h"
#include <cmath>

namespace aero::model {

struct WingCoefficients {
    double CL0{0.2};
    double CL_alpha{5.5};     // per rad
    double CL_de{0.3};        // elevator effectiveness
    double CD0{0.02};
    double k{0.08};           // induced drag factor
    double Cm0{0.0};
    double Cm_alpha{-1.0};
    double Cm_de{-1.0};
};

class Wing : public AerodynamicSurface {
public:
    explicit Wing(double area, double chord)
        : S_(area), c_(chord) {}

    aero::dynamics::ForcesMoments computeForcesMoments(
        const aero::dynamics::State6DOF& x,
        const aero::dynamics::ControlInput& u,
        const aero::env::Atmosphere& atm
    ) const override;

private:
    double S_;   // reference area
    double c_;   // mean chord
    WingCoefficients coeffs_;
};

} // namespace aero::model
