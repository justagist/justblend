#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "internal/blend.hpp"

using namespace justblend;
using namespace justblend::internal;

namespace
{

Eigen::VectorXd vec3(double a, double b, double c)
{
    Eigen::VectorXd v(3);
    v << a, b, c;
    return v;
}

} // namespace

TEST(BlendCap, ParabolicMatchesClosedForm)
{
    Eigen::VectorXd d_in = vec3(1.0, 0.0, 0.0);
    Eigen::VectorXd d_out = vec3(0.0, 1.0, 0.0);
    double r = 0.1;
    Eigen::VectorXd vmax = vec3(2.0, 2.0, 2.0);
    Eigen::VectorXd amax = vec3(3.0, 3.0, 3.0);

    double V = blendVCap(d_in, d_out, r, vmax, amax, std::nullopt, BlendShape::Parabolic);

    // V_acc = sqrt(2 r * min(amax_i / |delta_d_i|))
    double V_acc = std::sqrt(2.0 * r * 3.0 / 1.0); // |delta_d| = 1 for each non-zero component
    EXPECT_NEAR(V, std::min(V_acc, 2.0), 1e-12);
}

TEST(BlendCap, HermiteHasJerkCap)
{
    Eigen::VectorXd d_in = vec3(1.0, 0.0, 0.0);
    Eigen::VectorXd d_out = vec3(0.0, 1.0, 0.0);
    double r = 0.1;
    Eigen::VectorXd vmax = vec3(2.0, 2.0, 2.0);
    Eigen::VectorXd amax = vec3(3.0, 3.0, 3.0);
    Eigen::VectorXd jmax = vec3(20.0, 20.0, 20.0);

    double V_no_jerk = blendVCap(d_in, d_out, r, vmax, amax, std::nullopt, BlendShape::Hermite);
    double V_with_jerk = blendVCap(d_in, d_out, r, vmax, amax, jmax, BlendShape::Hermite);

    EXPECT_LE(V_with_jerk, V_no_jerk + 1e-12);

    // Hermite no-jerk V_acc = sqrt(4r/3 * min(amax/|delta|))
    double V_acc = std::sqrt((4.0 / 3.0) * r * 3.0);
    EXPECT_NEAR(V_no_jerk, std::min(V_acc, 2.0), 1e-12);

    // Jerk cap = ((2r^2/3) * min(jmax/|delta|))^(1/3)
    double V_jerk = std::cbrt((2.0 / 3.0) * r * r * 20.0);
    EXPECT_NEAR(V_with_jerk, std::min({V_acc, 2.0, V_jerk}), 1e-12);
}

TEST(BlendSample, ParabolicEndpointsMatch)
{
    BlendSegmentData seg;
    seg.shape = BlendShape::Parabolic;
    seg.V = 0.5;
    seg.r = 0.1;
    seg.duration = 2.0 * seg.r / seg.V;
    seg.d_in = vec3(1.0, 0.0, 0.0);
    seg.d_out = vec3(0.0, 1.0, 0.0);
    seg.q_start = vec3(0.0, 0.0, 0.0);

    auto s0 = sampleBlend(0.0, seg);
    EXPECT_NEAR((s0.q - seg.q_start).norm(), 0.0, 1e-12);
    EXPECT_NEAR((s0.qd - seg.V * seg.d_in).norm(), 0.0, 1e-12);

    auto s1 = sampleBlend(seg.duration, seg);
    EXPECT_NEAR((s1.qd - seg.V * seg.d_out).norm(), 0.0, 1e-12);
}

TEST(BlendSample, HermiteZeroAccelAtBoundaries)
{
    BlendSegmentData seg;
    seg.shape = BlendShape::Hermite;
    seg.V = 0.5;
    seg.r = 0.1;
    seg.duration = 2.0 * seg.r / seg.V;
    seg.d_in = vec3(1.0, 0.0, 0.0);
    seg.d_out = vec3(0.0, 1.0, 0.0);
    seg.q_start = vec3(0.0, 0.0, 0.0);

    auto s0 = sampleBlend(0.0, seg);
    auto s1 = sampleBlend(seg.duration, seg);
    EXPECT_NEAR(s0.qdd.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s1.qdd.norm(), 0.0, 1e-12);
    EXPECT_NEAR((s0.qd - seg.V * seg.d_in).norm(), 0.0, 1e-12);
    EXPECT_NEAR((s1.qd - seg.V * seg.d_out).norm(), 0.0, 1e-12);
}
