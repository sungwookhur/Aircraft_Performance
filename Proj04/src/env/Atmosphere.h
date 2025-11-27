// env/Atmosphere.hpp
#pragma once

namespace aero::env {

struct Atmosphere {
    double rho{1.225};    // air density [kg/m^3]
    double temperature{288.15};
    double pressure{101325.0};
};

} // namespace aero::env
