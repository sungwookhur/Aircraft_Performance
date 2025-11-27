#include <vector>

class BEMPropeller {
public:
    double R, B, rho;
    std::vector<double> c;     // chord distribution
    std::vector<double> theta; // pitch distribution
    std::vector<double> r;     // radial stations

    double computeThrust(double Omega, double Vinf);
    double computeTorque(double Omega, double Vinf);
};
