#include "internal/trap_profile.hpp"

#include <algorithm>
#include <cmath>

namespace justblend::internal {

std::vector<TrapPhase> trapPhases(double v0, double v1, double vmax, double amax, double D) {
    double vp = vmax;
    double d_acc = std::max((vp * vp - v0 * v0) / (2.0 * amax), 0.0);
    double d_dec = std::max((vp * vp - v1 * v1) / (2.0 * amax), 0.0);

    double t_acc, t_dec, t_cruise;
    if (d_acc + d_dec <= D + 1e-12) {
        t_acc = (vp - v0) / amax;
        t_dec = (vp - v1) / amax;
        t_cruise = (vp > 1e-12) ? (D - d_acc - d_dec) / vp : 0.0;
    } else {
        vp = std::sqrt(std::max(amax * D + 0.5 * (v0 * v0 + v1 * v1), 0.0));
        t_acc = (vp - v0) / amax;
        t_dec = (vp - v1) / amax;
        t_cruise = 0.0;
    }

    std::vector<TrapPhase> phases;
    phases.reserve(3);
    if (t_acc > 1e-12) phases.push_back({t_acc, amax});
    if (t_cruise > 1e-12) phases.push_back({t_cruise, 0.0});
    if (t_dec > 1e-12) phases.push_back({t_dec, -amax});
    return phases;
}

TrapSample sampleTrap(double t_local, double v0, const std::vector<TrapPhase>& phases) {
    double s = 0.0;
    double sd = v0;
    double t_rem = t_local;
    for (const auto& p : phases) {
        if (t_rem < p.duration) {
            s += sd * t_rem + 0.5 * p.sdd * t_rem * t_rem;
            sd += p.sdd * t_rem;
            return {s, sd, p.sdd};
        }
        s += sd * p.duration + 0.5 * p.sdd * p.duration * p.duration;
        sd += p.sdd * p.duration;
        t_rem -= p.duration;
    }
    return {s, sd, 0.0};
}

double reachableTrap(double v0, double amax, double D) {
    return std::sqrt(v0 * v0 + 2.0 * amax * D);
}

}  // namespace justblend::internal
