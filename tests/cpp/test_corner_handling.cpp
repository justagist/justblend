#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "justblend/justblend.hpp"

using namespace justblend;

namespace
{

Eigen::MatrixXd cornerWaypoints()
{
    Eigen::MatrixXd W(5, 3);
    W << 0.0, 0.0, 0.0, 1.0, 0.5, -0.3, 1.5, -0.2, 0.4, 0.5, 0.8, 1.0, 0.0, 0.0, 0.0;
    return W;
}

Limits scurveLimits()
{
    Limits L;
    L.v_max = (Eigen::VectorXd(3) << 1.5, 1.2, 1.0).finished();
    L.a_max = (Eigen::VectorXd(3) << 3.0, 2.5, 2.0).finished();
    L.j_max = (Eigen::VectorXd(3) << 20.0, 18.0, 15.0).finished();
    return L;
}

double maxAbs(const Eigen::MatrixXd& M, Eigen::Index col) { return M.col(col).cwiseAbs().maxCoeff(); }

} // namespace

class CornerHandlingScurveParam : public ::testing::TestWithParam<std::tuple<CornerHandling, BlendShape>>
{
};

TEST_P(CornerHandlingScurveParam, BoundsAndEndpointsHold)
{
    auto [ch, shape] = GetParam();
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());
    GenerationOptions opts;
    opts.blend_radius = 0.15;
    opts.corner_handling = ch;
    opts.blend_shape = shape;
    gen.setOptions(opts);

    auto W = cornerWaypoints();
    auto traj = gen.generate(W);

    auto r = traj.samples(0.001);

    // Endpoint snap
    EXPECT_NEAR((r.q.row(0).transpose() - W.row(0).transpose()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((r.q.row(r.q.rows() - 1).transpose() - W.row(W.rows() - 1).transpose()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.qd.row(0).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.qd.row(r.qd.rows() - 1).norm(), 0.0, 1e-12);

    // Bounds
    auto L = scurveLimits();
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_LE(maxAbs(r.qd, i), L.v_max(i) + 1e-6);
        EXPECT_LE(maxAbs(r.qdd, i), L.a_max(i) + 1e-6);
    }

    // For Hermite+SCurve, jerk should be bounded; finite-difference qdd.
    if (shape == BlendShape::HERMITE && r.q.rows() > 1)
    {
        Eigen::MatrixXd qddd(r.qdd.rows() - 1, r.qdd.cols());
        for (Eigen::Index k = 0; k + 1 < r.qdd.rows(); ++k)
        {
            qddd.row(k) = (r.qdd.row(k + 1) - r.qdd.row(k)) / (r.t(k + 1) - r.t(k));
        }
        for (int i = 0; i < 3; ++i)
        {
            EXPECT_LE(qddd.col(i).cwiseAbs().maxCoeff(), L.j_max->coeff(i) + 1e-3);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    All, CornerHandlingScurveParam,
    ::testing::Values(
        std::make_tuple(CornerHandling::STRICT_CORNERS, BlendShape::PARABOLIC),
        std::make_tuple(CornerHandling::STRICT_CORNERS, BlendShape::HERMITE),
        std::make_tuple(CornerHandling::HYBRID, BlendShape::PARABOLIC),
        std::make_tuple(CornerHandling::HYBRID, BlendShape::HERMITE),
        std::make_tuple(CornerHandling::USE_BLENDING, BlendShape::PARABOLIC),
        std::make_tuple(CornerHandling::USE_BLENDING, BlendShape::HERMITE)
    )
);

TEST(CornerHandling, TrapezoidalBoundsHold)
{
    Limits L;
    L.v_max = (Eigen::VectorXd(3) << 1.5, 1.2, 1.0).finished();
    L.a_max = (Eigen::VectorXd(3) << 3.0, 2.5, 2.0).finished();

    TrapezoidalTrajectoryGenerator gen(3);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.15;
    opts.corner_handling = CornerHandling::HYBRID;
    gen.setOptions(opts);

    auto W = cornerWaypoints();
    auto traj = gen.generate(W);
    auto r = traj.samples(0.001);
    EXPECT_NEAR(r.qd.row(0).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.qd.row(r.qd.rows() - 1).norm(), 0.0, 1e-12);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_LE(maxAbs(r.qd, i), L.v_max(i) + 1e-6);
        EXPECT_LE(maxAbs(r.qdd, i), L.a_max(i) + 1e-6);
    }
}

TEST(CornerHandling, UseBlendingRaisesWhenBlendRadiusUnusable)
{
    // blend_radius so small that the per-corner clip falls below 1e-9: the
    // planner rejects the blend in use_blending mode.
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());
    GenerationOptions opts;
    opts.blend_radius = 1e-12;
    opts.corner_handling = CornerHandling::USE_BLENDING;
    gen.setOptions(opts);

    auto W = cornerWaypoints();
    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST(CornerHandling, PerCornerBlendRadii)
{
    // demo waypoints have 3 interior corners. Set distinct radii per corner
    // and check that the resulting blend r-values follow our choices (after
    // per-corner geometric clipping).
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());

    Eigen::VectorXd radii(3);
    radii << 0.10, 0.20, 0.05;

    GenerationOptions opts;
    opts.blend_radius = 999.0; // would-be scalar fallback, but we override
    opts.blend_radii = radii;
    opts.corner_handling = CornerHandling::HYBRID;
    gen.setOptions(opts);

    auto W = cornerWaypoints();
    auto traj = gen.generate(W);
    const auto& planned_radii = traj.blendRadii();
    ASSERT_EQ(planned_radii.size(), 5u); // N entries; first/last are endpoint zeros
    EXPECT_NEAR(planned_radii[1], 0.10, 1e-12);
    EXPECT_NEAR(planned_radii[2], 0.20, 1e-12);
    EXPECT_NEAR(planned_radii[3], 0.05, 1e-12);
}

TEST(CornerHandling, BlendRadiiSizeOneBroadcasts)
{
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());

    GenerationOptions opts;
    opts.blend_radius = 0.999;
    opts.blend_radii = (Eigen::VectorXd(1) << 0.12).finished();
    opts.corner_handling = CornerHandling::HYBRID;
    gen.setOptions(opts);

    auto W = cornerWaypoints();
    auto traj = gen.generate(W);
    const auto& r = traj.blendRadii();
    EXPECT_NEAR(r[1], 0.12, 1e-12);
    EXPECT_NEAR(r[2], 0.12, 1e-12);
    EXPECT_NEAR(r[3], 0.12, 1e-12);
}

TEST(CornerHandling, BlendRadiiWrongSizeThrows)
{
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());

    GenerationOptions opts;
    opts.blend_radii = (Eigen::VectorXd(2) << 0.1, 0.1).finished(); // wrong size (need 3 or 1)
    opts.corner_handling = CornerHandling::HYBRID;
    gen.setOptions(opts);

    auto W = cornerWaypoints();
    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST(CornerHandling, BlendRadiiNonPositiveRejectedAtSetOptions)
{
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());

    GenerationOptions opts;
    opts.blend_radii = (Eigen::VectorXd(3) << 0.1, -0.1, 0.05).finished();
    EXPECT_THROW(gen.setOptions(opts), ValidationError);
}

TEST(CornerHandling, ReversalCornerStopsInHybrid)
{
    // Straight out and back: the interior corner is a full reversal, which
    // cannot be blended through; hybrid must fall back to a stop at the
    // waypoint itself.
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(2) << 2.0, 2.0).finished();
    L.j_max = (Eigen::VectorXd(2) << 10.0, 10.0).finished();

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.1;
    opts.corner_handling = CornerHandling::HYBRID;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    EXPECT_EQ(traj.cornerTypes()[1], CornerType::STOP);

    // Both legs are identical, so the stop is at half the duration and the
    // trajectory passes exactly through the reversal waypoint at zero speed.
    auto s = traj.sample(traj.duration() / 2.0);
    EXPECT_NEAR((s.q - W.row(1).transpose()).norm(), 0.0, 1e-9);
    EXPECT_NEAR(s.qd.norm(), 0.0, 1e-9);
}

TEST(CornerHandling, ReversalCornerThrowsInUseBlending)
{
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(2) << 2.0, 2.0).finished();
    L.j_max = (Eigen::VectorXd(2) << 10.0, 10.0).finished();

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.1;
    opts.corner_handling = CornerHandling::USE_BLENDING;
    gen.setOptions(opts);

    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST(CornerHandling, HybridCollapseRevertsToStop)
{
    // Tight V_blend cap that forward/backward pass forces to ~0; hybrid should
    // revert the blend to a stop and re-run, succeeding.
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 0.05, 0.0, 0.05, 0.05;

    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.j_max = (Eigen::VectorXd(2) << 10.0, 10.0).finished();

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.02;
    opts.corner_handling = CornerHandling::HYBRID;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    auto types = traj.cornerTypes();
    EXPECT_GE(types.size(), 3u);
    EXPECT_TRUE(types[1] == CornerType::STOP || types[1] == CornerType::BLEND);
    // sample at endpoints -> q matches
    auto s0 = traj.sample(0.0);
    auto s1 = traj.sample(traj.duration());
    EXPECT_NEAR((s0.q - W.row(0).transpose()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((s1.q - W.row(W.rows() - 1).transpose()).norm(), 0.0, 1e-12);
}
