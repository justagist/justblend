#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <string>

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
        std::make_tuple(CornerHandling::BLEND_WHERE_FEASIBLE, BlendShape::PARABOLIC),
        std::make_tuple(CornerHandling::BLEND_WHERE_FEASIBLE, BlendShape::HERMITE),
        std::make_tuple(CornerHandling::BLEND_WHERE_FASTER, BlendShape::PARABOLIC),
        std::make_tuple(CornerHandling::BLEND_WHERE_FASTER, BlendShape::HERMITE),
        std::make_tuple(CornerHandling::BLEND_ALL, BlendShape::PARABOLIC),
        std::make_tuple(CornerHandling::BLEND_ALL, BlendShape::HERMITE)
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
    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
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

TEST(CornerHandling, BlendAllRaisesWhenBlendRadiusUnusable)
{
    // blend_radius so small that the per-corner clip falls below 1e-9: the
    // planner rejects the blend in BLEND_ALL mode.
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());
    GenerationOptions opts;
    opts.blend_radius = 1e-12;
    opts.corner_handling = CornerHandling::BLEND_ALL;
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
    opts.corner_handling = CornerHandling::BLEND_ALL;
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
    opts.corner_handling = CornerHandling::BLEND_ALL;
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
    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
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

class FallbackToStop : public ::testing::TestWithParam<CornerHandling>
{
};

TEST_P(FallbackToStop, ReversalCornerStops)
{
    // Straight out and back: the interior corner is a full reversal, which
    // cannot be blended through, so it must fall back to a stop at the
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
    opts.corner_handling = GetParam();
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    EXPECT_EQ(traj.cornerTypes()[1], CornerType::STOP);

    // Both legs are identical, so the stop is at half the duration and the
    // trajectory passes exactly through the reversal waypoint at zero speed.
    auto s = traj.sample(traj.duration() / 2.0);
    EXPECT_NEAR((s.q - W.row(1).transpose()).norm(), 0.0, 1e-9);
    EXPECT_NEAR(s.qd.norm(), 0.0, 1e-9);
}

TEST(CornerHandling, ReversalCornerThrowsInBlendAll)
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
    opts.corner_handling = CornerHandling::BLEND_ALL;
    gen.setOptions(opts);

    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST_P(FallbackToStop, CollapsedBlendStops)
{
    // The first leg is so short, and the jerk limit so low, that the speed
    // reachable before the blend falls below the collapse threshold, so the
    // blend cannot be kept.
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1e-3, 0.0, 1e-3, 1.0;

    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.j_max = (Eigen::VectorXd(2) << 1e-12, 1e-12).finished();

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.02;
    opts.corner_handling = CornerHandling::BLEND_ALL;
    gen.setOptions(opts);
    EXPECT_THROW(gen.generate(W), ValidationError);

    opts.corner_handling = GetParam();
    gen.setOptions(opts);
    auto traj = gen.generate(W);
    ASSERT_EQ(traj.cornerTypes().size(), 3u);
    EXPECT_EQ(traj.cornerTypes()[1], CornerType::STOP);
    EXPECT_EQ(traj.blendRadii()[1], 0.0);

    auto at_corner = traj.sample(traj.waypointTimes()[1]);
    EXPECT_NEAR((at_corner.q - W.row(1).transpose()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(at_corner.qd.norm(), 0.0, 1e-12);
    auto s1 = traj.sample(traj.duration());
    EXPECT_NEAR((s1.q - W.row(2).transpose()).norm(), 0.0, 1e-12);
}

TEST_P(FallbackToStop, UnusableRadiusStops)
{
    SCurveTrajectoryGenerator gen(3);
    gen.setLimits(scurveLimits());
    GenerationOptions opts;
    opts.blend_radius = 1e-12;
    opts.corner_handling = CornerHandling::STRICT_CORNERS;
    gen.setOptions(opts);
    const auto W = cornerWaypoints();
    const double all_stopped = gen.generate(W).duration();

    opts.corner_handling = GetParam();
    gen.setOptions(opts);
    const auto traj = gen.generate(W);
    EXPECT_EQ(traj.duration(), all_stopped);
    for (Eigen::Index k = 1; k + 1 < W.rows(); ++k)
        EXPECT_EQ(traj.cornerTypes()[k], CornerType::STOP) << "waypoint " << k;
}

INSTANTIATE_TEST_SUITE_P(
    Modes, FallbackToStop, ::testing::Values(CornerHandling::BLEND_WHERE_FEASIBLE, CornerHandling::BLEND_WHERE_FASTER),
    [](const ::testing::TestParamInfo<CornerHandling>& info)
    { return info.param == CornerHandling::BLEND_WHERE_FEASIBLE ? "BlendWhereFeasible" : "BlendWhereFaster"; }
);

class MaxCornerDeviation : public ::testing::TestWithParam<BlendShape>
{
};

TEST_P(MaxCornerDeviation, SetsRadiusFromTolerance)
{
    // Right-angle corner on long legs, so the half-leg clip does not bind and
    // the blend reaches the tolerance exactly. blend_radius is ignored.
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 2.0, 0.0, 2.0, 2.0;
    SCurveTrajectoryGenerator gen(2);
    Limits L;
    L.v_max = Eigen::VectorXd::Constant(2, 1.0);
    L.a_max = Eigen::VectorXd::Constant(2, 2.0);
    L.j_max = Eigen::VectorXd::Constant(2, 10.0);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.corner_handling = CornerHandling::BLEND_ALL;
    opts.blend_shape = GetParam();
    opts.blend_radius = 1e-3;
    opts.max_corner_deviation = 0.01;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    const double factor = GetParam() == BlendShape::PARABOLIC ? 0.25 : 3.0 / 16.0;
    EXPECT_NEAR(traj.blendRadii()[1], 0.01 / (factor * std::sqrt(2.0)), 1e-12);
    EXPECT_NEAR(traj.cornerDeviations()[1], 0.01, 1e-12);
    const auto at_corner = traj.sample(traj.waypointTimes()[1]);
    EXPECT_NEAR((at_corner.q - W.row(1).transpose()).norm(), 0.01, 1e-12);
}

TEST_P(MaxCornerDeviation, RadiusStillClippedToHalfLeg)
{
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 0.1, 0.0, 0.1, 0.1;
    TrapezoidalTrajectoryGenerator gen(2);
    Limits L;
    L.v_max = Eigen::VectorXd::Constant(2, 1.0);
    L.a_max = Eigen::VectorXd::Constant(2, 2.0);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
    opts.blend_shape = GetParam();
    opts.max_corner_deviation = 1.0;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    EXPECT_EQ(traj.blendRadii()[1], 0.05);
    EXPECT_LT(traj.cornerDeviations()[1], 1.0);
}

INSTANTIATE_TEST_SUITE_P(
    Shapes, MaxCornerDeviation, ::testing::Values(BlendShape::PARABOLIC, BlendShape::HERMITE),
    [](const ::testing::TestParamInfo<BlendShape>& info)
    { return info.param == BlendShape::PARABOLIC ? "Parabolic" : "Hermite"; }
);

namespace
{

Limits unitLimits2d()
{
    Limits L;
    L.v_max = Eigen::VectorXd::Constant(2, 1.0);
    L.a_max = Eigen::VectorXd::Constant(2, 2.0);
    L.j_max = Eigen::VectorXd::Constant(2, 10.0);
    return L;
}

} // namespace

class CollinearWaypoints : public ::testing::TestWithParam<CornerHandling>
{
};

TEST_P(CollinearWaypoints, AreMergedIntoOneStraightSegment)
{
    // Extra waypoints inside the acceleration phase used to force the
    // S-curve back to zero acceleration at each of them.
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(unitLimits2d());
    GenerationOptions opts;
    opts.corner_handling = GetParam();
    gen.setOptions(opts);

    Eigen::MatrixXd straight(2, 2);
    straight << 0.0, 0.0, 3.0, 0.0;
    Eigen::MatrixXd W(5, 2);
    W << 0.0, 0.0, 0.05, 0.0, 0.1, 0.0, 0.2, 0.0, 3.0, 0.0;

    const auto reference = gen.generate(straight);
    const auto traj = gen.generate(W);
    EXPECT_DOUBLE_EQ(traj.duration(), reference.duration());
    EXPECT_EQ(traj.numSegments(), 1u);
    for (Eigen::Index k = 1; k < 4; ++k)
    {
        EXPECT_EQ(traj.cornerTypes()[k], CornerType::PASS);
        EXPECT_EQ(traj.cornerDeviations()[k], 0.0);
        const auto at_waypoint = traj.sample(traj.waypointTimes()[k]);
        EXPECT_NEAR((at_waypoint.q - W.row(k).transpose()).norm(), 0.0, 1e-12) << "waypoint " << k;
        EXPECT_NEAR(traj.junctionSpeeds()(k), at_waypoint.qd.norm(), 1e-12) << "waypoint " << k;
    }
}

TEST_P(CollinearWaypoints, NearlyCollinearWaypointKeepsVelocityContinuous)
{
    // Within the collinear tolerance: merged, so there is no velocity jump at
    // the waypoint, and it is missed by its distance from the merged segment.
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(unitLimits2d());
    GenerationOptions opts;
    opts.corner_handling = GetParam();
    gen.setOptions(opts);
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 1.0, 2.0, 2.000001;

    const auto traj = gen.generate(W);
    EXPECT_EQ(traj.cornerTypes()[1], CornerType::PASS);
    const Eigen::Vector2d chord = (W.row(2) - W.row(0)).transpose().normalized();
    const Eigen::Vector2d offset = W.row(1).transpose();
    EXPECT_NEAR(traj.cornerDeviations()[1], (offset - offset.dot(chord) * chord).norm(), 1e-15);

    const double t = traj.waypointTimes()[1];
    const auto before = traj.sample(t - 1e-9);
    const auto after = traj.sample(t + 1e-9);
    EXPECT_LT((after.qd - before.qd).norm(), 1e-8);
}

TEST_P(CollinearWaypoints, SlowlyCurvingRunIsNotFlattened)
{
    // Each turn is far below the old direction tolerance, but together they
    // bend the path well beyond the collinear tolerance.
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(unitLimits2d());
    GenerationOptions opts;
    opts.corner_handling = GetParam();
    gen.setOptions(opts);
    const int n = 200;
    Eigen::MatrixXd W(n, 2);
    for (int k = 0; k < n; ++k)
    {
        const double angle = 1e-6 * k;
        W.row(k) << 10.0 * std::sin(angle * 100.0), 10.0 * (1.0 - std::cos(angle * 100.0));
    }

    const auto traj = gen.generate(W);
    const double path_length = 10.0 * 1e-4 * (n - 1);
    EXPECT_LE(traj.maxCornerDeviation(), 1e-6 * path_length);
    int corners = 0;
    for (const auto type : traj.cornerTypes())
        corners += (type == CornerType::STOP || type == CornerType::BLEND) ? 1 : 0;
    EXPECT_GT(corners, 0);
}

INSTANTIATE_TEST_SUITE_P(
    Modes, CollinearWaypoints,
    ::testing::Values(
        CornerHandling::STRICT_CORNERS, CornerHandling::BLEND_ALL, CornerHandling::BLEND_WHERE_FEASIBLE,
        CornerHandling::BLEND_WHERE_FASTER
    ),
    [](const ::testing::TestParamInfo<CornerHandling>& info)
    {
        switch (info.param)
        {
        case CornerHandling::STRICT_CORNERS:
            return std::string("Strict");
        case CornerHandling::BLEND_ALL:
            return std::string("BlendAll");
        case CornerHandling::BLEND_WHERE_FEASIBLE:
            return std::string("BlendWhereFeasible");
        default:
            return std::string("BlendWhereFaster");
        }
    }
);

TEST(CornerHandling, ReversalAlongTheSameLineIsNotMerged)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(unitLimits2d());
    GenerationOptions opts;
    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
    gen.setOptions(opts);
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 0.5, 0.0;

    const auto traj = gen.generate(W);
    EXPECT_EQ(traj.cornerTypes()[1], CornerType::STOP);
    EXPECT_NEAR((traj.sample(traj.waypointTimes()[1]).q - W.row(1).transpose()).norm(), 0.0, 1e-12);
}

TEST(CornerHandling, BlendDoesNotStartBeforeMergedWaypoint)
{
    // The blend at (1, 0) may use up to half the merged 1.0 leg, but must stop
    // short of the collinear waypoint at (0.95, 0), which is passed exactly.
    TrapezoidalTrajectoryGenerator gen(2);
    gen.setLimits(unitLimits2d());
    GenerationOptions opts;
    opts.blend_radius = 0.5;
    opts.corner_handling = CornerHandling::BLEND_ALL;
    gen.setOptions(opts);
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 0.95, 0.0, 1.0, 0.0, 1.0, 1.0;

    const auto traj = gen.generate(W);
    EXPECT_EQ(traj.cornerTypes()[1], CornerType::PASS);
    EXPECT_NEAR(traj.blendRadii()[2], 0.05, 1e-12);
    EXPECT_NEAR((traj.sample(traj.waypointTimes()[1]).q - W.row(1).transpose()).norm(), 0.0, 1e-12);
}

TEST(CornerHandling, OnlyBlendWhereFasterStopsWhereBlendingIsSlower)
{
    // The two_segments_2d benchmark scenario: the jerk-limited Hermite blend
    // caps are so low that blending both corners takes longer than stopping.
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.5, 1.5, -0.2, 0.0, 0.0;
    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.5, 1.2).finished();
    L.a_max = (Eigen::VectorXd(2) << 3.0, 2.5).finished();
    L.j_max = (Eigen::VectorXd(2) << 20.0, 18.0).finished();
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.1;

    opts.corner_handling = CornerHandling::BLEND_ALL;
    gen.setOptions(opts);
    const double all_blended = gen.generate(W).duration();
    opts.corner_handling = CornerHandling::STRICT_CORNERS;
    gen.setOptions(opts);
    const double all_stopped = gen.generate(W).duration();
    ASSERT_GT(all_blended, all_stopped);

    opts.corner_handling = CornerHandling::BLEND_WHERE_FEASIBLE;
    gen.setOptions(opts);
    const auto feasible = gen.generate(W);
    EXPECT_EQ(feasible.duration(), all_blended);
    EXPECT_EQ(feasible.cornerTypes()[1], CornerType::BLEND);
    EXPECT_EQ(feasible.cornerTypes()[2], CornerType::BLEND);

    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
    gen.setOptions(opts);
    const auto faster = gen.generate(W);
    EXPECT_EQ(faster.duration(), all_stopped);
    EXPECT_EQ(faster.cornerTypes()[1], CornerType::STOP);
    EXPECT_EQ(faster.cornerTypes()[2], CornerType::STOP);
}

TEST(CornerHandling, BlendWhereFasterFallsBackToStrictWhenSingleTogglesStall)
{
    // Found by random search: from all blends, toggling one corner at a time
    // stalls 0.16% slower than stopping at every corner, so BLEND_WHERE_FASTER
    // must fall back to the all-stop plan.
    Eigen::MatrixXd W(6, 5);
    W << -0.4592092099485261, 0.74168051384452705, -0.54332847011760643, 0.49965124154219653, -0.1502332728317195,
        -0.040234812519597929, 0.82913847390067086, -1.4211887845307749, 0.8208181315538905, 0.35454380751899284,
        -1.0232885491817427, 1.0324886615216775, -2.3002174314617903, 0.55681515892422562, -0.55933872211613633,
        -0.54301414080114174, 0.41221925311017915, -2.3553347546187897, 1.4798909090128098, 0.25690840576482543,
        0.12105975186956375, 0.43788139530920822, -2.8477858931421483, 1.95257735093882, -0.25466038795437096,
        0.19127304211900364, 0.69974457309115645, -3.6916028236328886, 2.8096648608164871, 0.41137086189098837;
    Limits L;
    L.v_max = (Eigen::VectorXd(5) << 0.96201491541465289, 2.5846627751044986, 0.91100129398311425, 1.6384725541229812,
               2.2887120982001838)
                  .finished();
    L.a_max = (Eigen::VectorXd(5) << 3.9662543599726932, 4.3569571131453211, 7.7367202257538077, 6.6686623757182399,
               0.44980710133480928)
                  .finished();
    L.j_max = (Eigen::VectorXd(5) << 60.614795105767001, 32.299057920067867, 57.478208270593257, 26.250561486221976,
               43.155057094113396)
                  .finished();
    SCurveTrajectoryGenerator gen(5);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.35646290719390455;

    opts.corner_handling = CornerHandling::STRICT_CORNERS;
    gen.setOptions(opts);
    const double all_stopped = gen.generate(W).duration();
    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
    gen.setOptions(opts);
    EXPECT_EQ(gen.generate(W).duration(), all_stopped);
}
