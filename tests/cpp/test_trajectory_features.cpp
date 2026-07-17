#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "justblend/justblend.hpp"

using namespace justblend;

namespace
{

Limits limits2d(double v, double a, double j)
{
    Limits L;
    L.v_max = (Eigen::VectorXd(2) << v, v).finished();
    L.a_max = (Eigen::VectorXd(2) << a, a).finished();
    L.j_max = (Eigen::VectorXd(2) << j, j).finished();
    return L;
}

Eigen::MatrixXd straightLine(double length)
{
    Eigen::MatrixXd W(2, 2);
    W << 0.0, 0.0, length, 0.0;
    return W;
}

// Minimum distance from the corner waypoint to the densely sampled path.
double denseMinDistanceToCorner(const Trajectory& traj, const Eigen::Vector2d& corner)
{
    auto r = traj.samples(1e-4);
    double best = 1e100;
    for (Eigen::Index i = 0; i < r.q.rows(); ++i)
    {
        best = std::min(best, (r.q.row(i).transpose() - corner).norm());
    }
    return best;
}

} // namespace

TEST(BoundarySpeeds, StartAndEndSpeedsHonoured)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));

    auto traj = gen.generate(straightLine(2.0), 0.3, 0.2);

    auto s0 = traj.sample(0.0);
    auto s1 = traj.sample(traj.duration());
    EXPECT_NEAR(s0.qd(0), 0.3, 1e-9);
    EXPECT_NEAR(s0.qd(1), 0.0, 1e-12);
    EXPECT_NEAR(s1.qd(0), 0.2, 1e-9);
    EXPECT_NEAR(s1.qd(1), 0.0, 1e-12);

    EXPECT_NEAR(traj.junctionSpeeds()(0), 0.3, 1e-12);
    EXPECT_NEAR(traj.junctionSpeeds()(1), 0.2, 1e-12);

    // Position endpoints still snap exactly.
    EXPECT_NEAR(s0.q(0), 0.0, 1e-12);
    EXPECT_NEAR(s1.q(0), 2.0, 1e-12);

    // Batch sampling agrees at the boundaries.
    auto r = traj.samples(0.001);
    EXPECT_NEAR(r.qd(0, 0), 0.3, 1e-9);
    EXPECT_NEAR(r.qd(r.qd.rows() - 1, 0), 0.2, 1e-9);
}

TEST(BoundarySpeeds, DefaultsToRestToRest)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    auto traj = gen.generate(straightLine(2.0));
    EXPECT_NEAR(traj.sample(0.0).qd.norm(), 0.0, 1e-12);
    EXPECT_NEAR(traj.sample(traj.duration()).qd.norm(), 0.0, 1e-12);
}

TEST(BoundarySpeeds, InfeasibleEndSpeedThrows)
{
    // 0.01 m is far too short to accelerate to 0.9 m/s.
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 1.0, 10.0));
    EXPECT_THROW(gen.generate(straightLine(0.01), 0.0, 0.9), ValidationError);
}

TEST(BoundarySpeeds, InfeasibleStartSpeedThrows)
{
    // Cannot stop from 0.9 m/s within 0.01 m.
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 1.0, 10.0));
    EXPECT_THROW(gen.generate(straightLine(0.01), 0.9, 0.0), ValidationError);
}

TEST(BoundarySpeeds, ExceedingVelocityLimitThrows)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    EXPECT_THROW(gen.generate(straightLine(2.0), 1.5, 0.0), ValidationError);
}

TEST(BoundarySpeeds, NegativeSpeedThrows)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    EXPECT_THROW(gen.generate(straightLine(2.0), -0.1, 0.0), ValidationError);
}

TEST(CornerDeviation, HermiteMatchesDenseSampling)
{
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0;

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    GenerationOptions opts;
    opts.blend_radius = 0.2;
    opts.corner_handling = CornerHandling::Hybrid;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    ASSERT_EQ(traj.cornerTypes()[1], CornerType::Blend);

    // 90 degree corner: |d_out - d_in| = sqrt(2); Hermite factor 3/16.
    const double expected = (3.0 / 16.0) * 0.2 * std::sqrt(2.0);
    const auto& dev = traj.cornerDeviations();
    ASSERT_EQ(dev.size(), 3u);
    EXPECT_NEAR(dev[1], expected, 1e-12);
    EXPECT_NEAR(traj.maxCornerDeviation(), expected, 1e-12);

    EXPECT_NEAR(denseMinDistanceToCorner(traj, W.row(1).transpose()), expected, 1e-6);
}

TEST(CornerDeviation, ParabolicMatchesDenseSampling)
{
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0;

    TrapezoidalTrajectoryGenerator gen(2);
    Limits L = limits2d(1.0, 2.0, 10.0);
    L.j_max.reset();
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.2;
    opts.corner_handling = CornerHandling::Hybrid;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    ASSERT_EQ(traj.cornerTypes()[1], CornerType::Blend);

    const double expected = 0.25 * 0.2 * std::sqrt(2.0);
    EXPECT_NEAR(traj.cornerDeviations()[1], expected, 1e-12);
    EXPECT_NEAR(denseMinDistanceToCorner(traj, W.row(1).transpose()), expected, 1e-6);
}

TEST(WaypointTimes, BlendCornersPassAtClosestApproach)
{
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 1.0;

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    GenerationOptions opts;
    opts.blend_radius = 0.1;
    opts.corner_handling = CornerHandling::Hybrid;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    const auto& wt = traj.waypointTimes();
    ASSERT_EQ(wt.size(), 4u);
    EXPECT_DOUBLE_EQ(wt.front(), 0.0);
    EXPECT_NEAR(wt.back(), traj.duration(), 1e-12);
    for (std::size_t k = 1; k < wt.size(); ++k)
    {
        EXPECT_GT(wt[k], wt[k - 1]);
    }

    // At a blend corner's waypoint time the distance to the waypoint equals
    // the analytic corner deviation (closest approach).
    for (std::size_t k = 1; k + 1 < wt.size(); ++k)
    {
        if (traj.cornerTypes()[k] == CornerType::Blend)
        {
            auto s = traj.sample(wt[k]);
            EXPECT_NEAR((s.q - W.row(static_cast<Eigen::Index>(k)).transpose()).norm(),
                        traj.cornerDeviations()[k], 1e-9);
        }
    }
}

TEST(WaypointTimes, StopCornersPassExactlyThroughWaypoints)
{
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 1.0;

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    GenerationOptions opts;
    opts.corner_handling = CornerHandling::StrictCorners;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    const auto& wt = traj.waypointTimes();
    for (std::size_t k = 0; k < wt.size(); ++k)
    {
        auto s = traj.sample(wt[k]);
        EXPECT_NEAR((s.q - W.row(static_cast<Eigen::Index>(k)).transpose()).norm(), 0.0, 1e-9);
        EXPECT_NEAR(s.qd.norm(), 0.0, 1e-9);
    }
}

TEST(SegmentStartTimes, MatchSegmentDurations)
{
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 1.0;

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    GenerationOptions opts;
    opts.blend_radius = 0.1;
    opts.corner_handling = CornerHandling::Hybrid;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    const auto& st = traj.segmentStartTimes();
    const auto& segs = traj.segments();
    ASSERT_EQ(st.size(), segs.size() + 1);
    EXPECT_DOUBLE_EQ(st.front(), 0.0);
    for (std::size_t i = 0; i < segs.size(); ++i)
    {
        EXPECT_NEAR(st[i + 1] - st[i], segs[i].duration, 1e-12);
    }
    EXPECT_NEAR(st.back(), traj.duration(), 1e-12);
}

TEST(Scaling, ReducesPeakVelocityAndAcceleration)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));

    auto full = gen.generate(straightLine(3.0));

    GenerationOptions opts;
    opts.velocity_scaling_factor = 0.5;
    opts.acceleration_scaling_factor = 0.5;
    gen.setOptions(opts);
    auto scaled = gen.generate(straightLine(3.0));

    auto r = scaled.samples(0.001);
    EXPECT_LE(r.qd.col(0).cwiseAbs().maxCoeff(), 0.5 * 1.0 + 1e-9);
    EXPECT_LE(r.qdd.col(0).cwiseAbs().maxCoeff(), 0.5 * 2.0 + 1e-9);
    EXPECT_GT(scaled.duration(), full.duration());
}

TEST(Stretch, MatchesTargetDurationAndPreservesPath)
{
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 1.0;

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    GenerationOptions opts;
    opts.blend_radius = 0.1;
    opts.corner_handling = CornerHandling::Hybrid;
    gen.setOptions(opts);

    auto traj = gen.generate(W);
    const double T = traj.duration();
    const double target = 2.5 * T;
    auto slow = traj.stretchedTo(target);

    EXPECT_DOUBLE_EQ(slow.duration(), target);

    // Same path, derivatives scaled: q_slow(s*t) == q(t), qd/s, qdd/s^2.
    const double s = target / T;
    for (double t : {0.0, 0.1 * T, 0.37 * T, 0.5 * T, 0.83 * T, T})
    {
        auto a = traj.sample(t);
        auto b = slow.sample(s * t);
        EXPECT_NEAR((a.q - b.q).norm(), 0.0, 1e-9) << "t=" << t;
        EXPECT_NEAR((a.qd / s - b.qd).norm(), 0.0, 1e-9) << "t=" << t;
        EXPECT_NEAR((a.qdd / (s * s) - b.qdd).norm(), 0.0, 1e-9) << "t=" << t;
    }

    // Metadata scales consistently.
    const auto& wt = traj.waypointTimes();
    const auto& wt_slow = slow.waypointTimes();
    for (std::size_t k = 0; k < wt.size(); ++k)
    {
        EXPECT_NEAR(wt_slow[k], s * wt[k], 1e-9);
    }
    EXPECT_NEAR(slow.junctionSpeeds().maxCoeff(), traj.junctionSpeeds().maxCoeff() / s, 1e-12);
    EXPECT_DOUBLE_EQ(slow.maxCornerDeviation(), traj.maxCornerDeviation());
}

TEST(Stretch, PreservesBoundarySpeedsScaled)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    auto traj = gen.generate(straightLine(2.0), 0.4, 0.0);
    auto slow = traj.stretchedTo(2.0 * traj.duration());
    EXPECT_NEAR(slow.sample(0.0).qd(0), 0.2, 1e-9);
}

TEST(Stretch, ShorterTargetThrows)
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(limits2d(1.0, 2.0, 10.0));
    auto traj = gen.generate(straightLine(2.0));
    EXPECT_THROW(traj.stretchedTo(0.5 * traj.duration()), ValidationError);
    // Equal duration is a no-op copy.
    auto same = traj.stretchedTo(traj.duration());
    EXPECT_DOUBLE_EQ(same.duration(), traj.duration());
}

TEST(Scaling, InvalidFactorsRejectedAtSetOptions)
{
    SCurveTrajectoryGenerator gen(2);
    GenerationOptions opts;
    opts.velocity_scaling_factor = 0.0;
    EXPECT_THROW(gen.setOptions(opts), ValidationError);
    opts.velocity_scaling_factor = 1.5;
    EXPECT_THROW(gen.setOptions(opts), ValidationError);
    opts.velocity_scaling_factor = 1.0;
    opts.acceleration_scaling_factor = -0.2;
    EXPECT_THROW(gen.setOptions(opts), ValidationError);
}
