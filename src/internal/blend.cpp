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

    if (shape == BlendShape::PARABOLIC)
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

void sampleBlend(
    double t_local, const BlendSegmentData& seg, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qd,
    Eigen::Ref<Eigen::VectorXd> qdd, Eigen::Ref<Eigen::VectorXd> qddd
)
{
    const double V = seg.V;
    const double r = seg.r;
    const double T_b = seg.duration;
    const Eigen::VectorXd& d_in = seg.d_in;
    const Eigen::VectorXd& d_out = seg.d_out;
    // Every output is a single expression of the stored vectors, so Eigen
    // evaluates it in place with no temporaries.
    const auto delta_d = d_out - d_in;

    if (seg.shape == BlendShape::PARABOLIC)
    {
        const double a = (r > 1e-12) ? V * V / (2.0 * r) : 0.0;
        q = seg.q_start + (V * t_local) * d_in + (0.5 * a * t_local * t_local) * delta_d;
        qd = V * d_in + (a * t_local) * delta_d;
        qdd = a * delta_d;
        qddd.setZero();
        return;
    }

    // cubic-Hermite: alpha(tau) = 3 tau^2 - 2 tau^3
    double tau = (T_b > 1e-12) ? (t_local / T_b) : 0.0;
    double alpha = 3.0 * tau * tau - 2.0 * tau * tau * tau;
    double dalpha_dt = (T_b > 1e-12) ? (6.0 * tau * (1.0 - tau) / T_b) : 0.0;
    double d2alpha_dt2 = (T_b > 1e-12) ? (6.0 * (1.0 - 2.0 * tau) / (T_b * T_b)) : 0.0;
    double alpha_int = (T_b > 1e-12) ? (T_b * (tau * tau * tau - 0.5 * tau * tau * tau * tau)) : 0.0;

    q = seg.q_start + (V * t_local) * d_in + (V * alpha_int) * delta_d;
    qd = V * d_in + (V * alpha) * delta_d;
    qdd = (V * dalpha_dt) * delta_d;
    qddd = (V * d2alpha_dt2) * delta_d;
}

} // namespace justblend::internal
