#include "internal/blend.hpp"

#include <algorithm>
#include <cmath>

namespace justblend::internal
{

double blendVCap(
    const Eigen::VectorXd& d_in, const Eigen::VectorXd& d_out, double r, const Eigen::VectorXd& vmax,
    const Eigen::VectorXd& amax, const std::optional<Eigen::VectorXd>& jmax, BlendShape shape
)
{
    Eigen::VectorXd delta_d = d_out - d_in;
    Eigen::VectorXd abs_delta = delta_d.cwiseAbs().cwiseMax(1e-12);
    Eigen::VectorXd d_max = d_in.cwiseAbs().cwiseMax(d_out.cwiseAbs()).cwiseMax(1e-12);

    double V_vel = (vmax.array() / d_max.array()).minCoeff();

    if (shape == BlendShape::Parabolic)
    {
        double min_a_ratio = (amax.array() / abs_delta.array()).minCoeff();
        double V_acc = std::sqrt(2.0 * r * min_a_ratio);
        return std::min(V_acc, V_vel);
    }

    // Hermite
    double min_a_ratio = (amax.array() / abs_delta.array()).minCoeff();
    double V_acc = std::sqrt((4.0 / 3.0) * r * min_a_ratio);
    double cap = std::min(V_acc, V_vel);
    if (jmax.has_value())
    {
        double min_j_ratio = (jmax->array() / abs_delta.array()).minCoeff();
        double V_jerk = std::cbrt((2.0 / 3.0) * r * r * min_j_ratio);
        cap = std::min(cap, V_jerk);
    }
    return cap;
}

BlendSample sampleBlend(double t_local, const BlendSegmentData& seg)
{
    const double V = seg.V;
    const double r = seg.r;
    const double T_b = seg.duration;
    const Eigen::VectorXd& d_in = seg.d_in;
    const Eigen::VectorXd& d_out = seg.d_out;
    const Eigen::VectorXd& q_start = seg.q_start;

    BlendSample out;

    if (seg.shape == BlendShape::Parabolic)
    {
        Eigen::VectorXd a = Eigen::VectorXd::Zero(d_in.size());
        if (r > 1e-12)
        {
            a = (V * V) * (d_out - d_in) / (2.0 * r);
        }
        out.q = q_start + V * d_in * t_local + 0.5 * a * (t_local * t_local);
        out.qd = V * d_in + a * t_local;
        out.qdd = a;
        return out;
    }

    // cubic-Hermite: alpha(tau) = 3 tau^2 - 2 tau^3
    double tau = (T_b > 1e-12) ? (t_local / T_b) : 0.0;
    double alpha = 3.0 * tau * tau - 2.0 * tau * tau * tau;
    double dalpha_dt = (T_b > 1e-12) ? (6.0 * tau * (1.0 - tau) / T_b) : 0.0;
    double alpha_int = (T_b > 1e-12) ? (T_b * (tau * tau * tau - 0.5 * tau * tau * tau * tau)) : 0.0;

    Eigen::VectorXd delta_d = d_out - d_in;
    out.q = q_start + V * d_in * t_local + V * alpha_int * delta_d;
    out.qd = V * d_in + V * alpha * delta_d;
    out.qdd = V * dalpha_dt * delta_d;
    return out;
}

} // namespace justblend::internal
