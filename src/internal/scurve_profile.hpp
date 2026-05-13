#ifndef JUSTBLEND_INTERNAL_SCURVE_PROFILE_HPP
#define JUSTBLEND_INTERNAL_SCURVE_PROFILE_HPP

#include <vector>

#include "internal/segment.hpp"

namespace justblend::internal {

double rampDistance(double v0, double v1, double amax, double jmax);

std::vector<ScurvePhase> rampPhases(double v0, double v1, double amax, double jmax);

std::vector<ScurvePhase> scurvePhases(double v0, double v1, double vmax, double amax, double jmax, double D);

struct ScurveSample {
    double s;
    double sd;
    double sdd;
};

ScurveSample sampleScurve(double t_local, double v0, const std::vector<ScurvePhase>& phases);

double reachableScurve(double v0, double amax, double jmax, double D, double v_upper);

}  // namespace justblend::internal

#endif  // JUSTBLEND_INTERNAL_SCURVE_PROFILE_HPP
