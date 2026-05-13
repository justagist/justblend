#include "internal/scurve_profile.hpp"

#include <algorithm>
#include <cmath>

namespace justblend::internal {

double rampDistance(double v0, double v1, double amax, double jmax) {
    double dv = std::abs(v1 - v0);
    if (dv < 1e-15) return 0.0;
    if (dv >= amax * amax / jmax) {
        return 0.5 * (v0 + v1) * (dv / amax + amax / jmax);
    }
    return (v0 + v1) * std::sqrt(dv / jmax);
}

std::vector<ScurvePhase> rampPhases(double v0, double v1, double amax, double jmax) {
    double dv = v1 - v0;
    if (std::abs(dv) < 1e-15) return {};
    double sign = dv > 0.0 ? 1.0 : -1.0;
    double dv_abs = std::abs(dv);

    double T1, T2, T3;
    if (dv_abs >= amax * amax / jmax) {
        T1 = amax / jmax;
        T2 = dv_abs / amax - amax / jmax;
        T3 = amax / jmax;
    } else {
        double a_peak = std::sqrt(jmax * dv_abs);
        T1 = a_peak / jmax;
        T2 = 0.0;
        T3 = T1;
    }

    std::vector<ScurvePhase> phases;
    phases.reserve(3);
    phases.push_back({T1, sign * jmax});
    if (T2 > 1e-12) phases.push_back({T2, 0.0});
    phases.push_back({T3, -sign * jmax});
    return phases;
}

std::vector<ScurvePhase> scurvePhases(double v0, double v1, double vmax, double amax, double jmax, double D) {
    double d_acc = rampDistance(v0, vmax, amax, jmax);
    double d_dec = rampDistance(vmax, v1, amax, jmax);

    double v_peak;
    double t_cruise;
    if (d_acc + d_dec <= D + 1e-12) {
        v_peak = vmax;
        t_cruise = (vmax > 1e-12) ? (D - d_acc - d_dec) / vmax : 0.0;
    } else {
        double lo = std::max(v0, v1);
        double hi = vmax;
        for (int i = 0; i < 80; ++i) {
            double mid = 0.5 * (lo + hi);
            if (rampDistance(v0, mid, amax, jmax) + rampDistance(mid, v1, amax, jmax) <= D) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        v_peak = lo;
        t_cruise = 0.0;
    }

    std::vector<ScurvePhase> phases = rampPhases(v0, v_peak, amax, jmax);
    if (t_cruise > 1e-12) phases.push_back({t_cruise, 0.0});
    auto dec = rampPhases(v_peak, v1, amax, jmax);
    phases.insert(phases.end(), dec.begin(), dec.end());
    return phases;
}

ScurveSample sampleScurve(double t_local, double v0, const std::vector<ScurvePhase>& phases) {
    double s = 0.0;
    double sd = v0;
    double sdd = 0.0;
    double t_rem = t_local;
    for (const auto& p : phases) {
        if (t_rem <= p.duration) {
            s += sd * t_rem + 0.5 * sdd * t_rem * t_rem + (1.0 / 6.0) * p.jerk * t_rem * t_rem * t_rem;
            sd += sdd * t_rem + 0.5 * p.jerk * t_rem * t_rem;
            sdd += p.jerk * t_rem;
            return {s, sd, sdd};
        }
        s += sd * p.duration + 0.5 * sdd * p.duration * p.duration + (1.0 / 6.0) * p.jerk * p.duration * p.duration * p.duration;
        sd += sdd * p.duration + 0.5 * p.jerk * p.duration * p.duration;
        sdd += p.jerk * p.duration;
        t_rem -= p.duration;
    }
    return {s, sd, sdd};
}

double reachableScurve(double v0, double amax, double jmax, double D, double v_upper) {
    if (rampDistance(v0, v_upper, amax, jmax) <= D) return v_upper;
    double lo = v0;
    double hi = v_upper;
    for (int i = 0; i < 80; ++i) {
        double mid = 0.5 * (lo + hi);
        if (rampDistance(v0, mid, amax, jmax) <= D) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

}  // namespace justblend::internal
