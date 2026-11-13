#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "justblend/justblend.hpp"

using namespace justblend;

namespace
{

Trajectory makeDemoScurve()
{
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.5, 1.5, -0.2, 0.0, 0.0;
    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.5, 1.2).finished();
    L.a_max = (Eigen::VectorXd(2) << 3.0, 2.5).finished();
    L.j_max = (Eigen::VectorXd(2) << 20.0, 18.0).finished();

    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(L);
    GenerationOptions opts;
    opts.blend_radius = 0.15;
    opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
    gen.setOptions(opts);
    return gen.generate(W);
}

// Out-and-back along x: the acceleration steps at every phase and segment boundary.
Trajectory makeOutAndBackTrapezoid()
{
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(2) << 2.0, 2.0).finished();

    TrapezoidalTrajectoryGenerator gen(2);
    gen.setLimits(L);
    return gen.generate(W);
}

} // namespace

TEST(Sampling, SingleAndBatchAgree)
{
    auto traj = makeDemoScurve();
    Eigen::VectorXd times(11);
    for (int i = 0; i < 11; ++i)
        times(i) = (traj.duration() * i) / 10.0;
    auto batch = traj.samples(times);
    for (Eigen::Index i = 0; i < times.size(); ++i)
    {
        auto s = traj.sample(times(i));
        EXPECT_NEAR((s.q - batch.q.row(i).transpose()).norm(), 0.0, 1e-12) << "row " << i;
        EXPECT_NEAR((s.qd - batch.qd.row(i).transpose()).norm(), 0.0, 1e-12) << "row " << i;
        EXPECT_NEAR((s.qdd - batch.qdd.row(i).transpose()).norm(), 0.0, 1e-12) << "row " << i;
    }
}

TEST(Sampling, DtAndExplicitTimesAgree)
{
    auto traj = makeDemoScurve();
    double dt = 0.01;
    auto a = traj.samples(dt);

    // construct equivalent VectorXd
    Eigen::VectorXd times(a.t.size());
    for (Eigen::Index i = 0; i < times.size(); ++i)
        times(i) = a.t(i);
    auto b = traj.samples(times);

    EXPECT_EQ(a.t.size(), b.t.size());
    EXPECT_NEAR((a.q - b.q).cwiseAbs().maxCoeff(), 0.0, 1e-15);
    EXPECT_NEAR((a.qd - b.qd).cwiseAbs().maxCoeff(), 0.0, 1e-15);
    EXPECT_NEAR((a.qdd - b.qdd).cwiseAbs().maxCoeff(), 0.0, 1e-15);
}

TEST(Sampling, ClampsOutOfRangeTimes)
{
    auto traj = makeDemoScurve();
    auto s_neg = traj.sample(-1.0);
    auto s_past = traj.sample(traj.duration() + 5.0);
    EXPECT_DOUBLE_EQ(s_neg.t, 0.0);
    EXPECT_DOUBLE_EQ(s_past.t, traj.duration());
}

TEST(Sampling, EndpointSnapAppliesAtBoundaries)
{
    auto traj = makeDemoScurve();
    Eigen::MatrixXd W(4, 2);
    W << 0.0, 0.0, 1.0, 0.5, 1.5, -0.2, 0.0, 0.0;

    auto s0 = traj.sample(0.0);
    auto s1 = traj.sample(traj.duration());
    EXPECT_NEAR((s0.q - W.row(0).transpose()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(s0.qd.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s0.qdd.norm(), 0.0, 1e-12);
    EXPECT_NEAR((s1.q - W.row(3).transpose()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(s1.qd.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s1.qdd.norm(), 0.0, 1e-12);
}

TEST(Sampling, NonMonotoneTimesHandled)
{
    auto traj = makeDemoScurve();
    Eigen::VectorXd times(5);
    times << 1.0, 0.2, 0.8, 0.5, 0.0;
    auto batch = traj.samples(times);
    for (Eigen::Index i = 0; i < times.size(); ++i)
    {
        auto s = traj.sample(times(i));
        EXPECT_NEAR((s.q - batch.q.row(i).transpose()).norm(), 0.0, 1e-12) << "row " << i;
    }
}

TEST(Generator, ThrowsWhenLimitsNotSet)
{
    SCurveTrajectoryGenerator gen(3);
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(2, 3);
    W(1, 0) = 1.0;
    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST(Generator, ThrowsOnDuplicateConsecutiveWaypoints)
{
    SCurveTrajectoryGenerator gen(2);
    Limits L;
    L.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    L.j_max = (Eigen::VectorXd(2) << 10.0, 10.0).finished();
    gen.setLimits(L);

    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0;
    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST(Generator, ThrowsOnDimMismatch)
{
    SCurveTrajectoryGenerator gen(2);
    Limits L;
    L.v_max = (Eigen::VectorXd(3) << 1.0, 1.0, 1.0).finished();
    L.a_max = (Eigen::VectorXd(3) << 1.0, 1.0, 1.0).finished();
    L.j_max = (Eigen::VectorXd(3) << 10.0, 10.0, 10.0).finished();
    EXPECT_THROW(gen.setLimits(L), ValidationError);
}

TEST(Sampling, DtNearMultipleOfDurationHasNoDuplicateFinalSample)
{
    auto traj = makeDemoScurve();
    const double T = traj.duration();
    // dt chosen so T is a hair past 1000 steps: the final-time sample would
    // nearly duplicate the last step without the tolerance guard.
    const double dt = T / 1000.0 * (1.0 - 1e-13);
    auto r = traj.samples(dt);
    for (Eigen::Index i = 1; i < r.t.size(); ++i)
    {
        EXPECT_GT(r.t(i) - r.t(i - 1), 1e-9 * dt) << "near-duplicate time at row " << i;
    }
    EXPECT_NEAR(r.t(r.t.size() - 1), T, 1e-9 * dt);
}

TEST(Sampling, BoundaryTimesSnapAnywhereInBatch)
{
    auto traj = makeDemoScurve();
    Eigen::VectorXd times(4);
    times << 0.5, 0.0, traj.duration(), 0.2;
    auto r = traj.samples(times);
    EXPECT_NEAR(r.qd.row(1).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.qdd.row(1).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.qd.row(2).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.qdd.row(2).norm(), 0.0, 1e-12);
}

TEST(Sampling, TrapezoidEndpointsReportLimitAcceleration)
{
    auto traj = makeOutAndBackTrapezoid();
    const Eigen::Vector2d expected(2.0, 0.0);
    EXPECT_NEAR((traj.sample(0.0).qdd - expected).norm(), 0.0, 1e-12);
    EXPECT_NEAR((traj.sample(traj.duration()).qdd - expected).norm(), 0.0, 1e-12);
}

TEST(Sampling, BoundarySamplesIndependentOfTimeOrder)
{
    auto traj = makeOutAndBackTrapezoid();
    const auto& b = traj.segmentStartTimes();
    ASSERT_EQ(b.size(), 3u);

    // The first segment decelerates into the stop at the middle waypoint; a
    // sample exactly on that boundary reports the segment ending there.
    const Eigen::Vector2d at_stop(-2.0, 0.0);
    EXPECT_NEAR((traj.sample(b[1]).qdd - at_stop).norm(), 0.0, 1e-12);

    Eigen::VectorXd ascending(3), descending(3), shuffled(6);
    ascending << b[0], b[1], b[2];
    descending << b[2], b[1], b[0];
    shuffled << b[1], b[2], b[1], b[0], b[1], b[1];
    for (const Eigen::VectorXd& times : {ascending, descending, shuffled})
    {
        auto batch = traj.samples(times);
        for (Eigen::Index i = 0; i < times.size(); ++i)
        {
            auto s = traj.sample(times(i));
            EXPECT_EQ(s.q, batch.q.row(i).transpose()) << "row " << i;
            EXPECT_EQ(s.qd, batch.qd.row(i).transpose()) << "row " << i;
            EXPECT_EQ(s.qdd, batch.qdd.row(i).transpose()) << "row " << i;
        }
    }
}

TEST(Sampling, StepLongerThanDurationStillIncludesFinalTime)
{
    auto traj = makeDemoScurve();
    auto r = traj.samples(1e12);
    ASSERT_EQ(r.t.size(), 2);
    EXPECT_EQ(r.t(0), 0.0);
    EXPECT_EQ(r.t(1), traj.duration());
}

TEST(Sampling, LastUniformSampleIsExactlyTheFinalTime)
{
    auto traj = makeDemoScurve();
    for (double dt : {0.01, traj.duration() / 1000.0, traj.duration() / 1000.0 * (1.0 - 1e-13)})
    {
        auto r = traj.samples(dt);
        EXPECT_EQ(r.t(r.t.size() - 1), traj.duration()) << "dt " << dt;
    }
}

TEST(Sampling, AllSingleSampleCallsAgree)
{
    auto traj = makeDemoScurve();
    std::vector<double> times(traj.segmentStartTimes());
    for (int i = 0; i <= 20; ++i)
        times.push_back(traj.duration() * i / 20.0);
    times.push_back(-1.0);
    times.push_back(traj.duration() + 1.0);

    Sample reused;
    // Caller buffers need not be VectorXd: here they are columns of one matrix.
    Eigen::MatrixXd buffers(2, 4);
    for (double t : times)
    {
        const auto s = traj.sample(t);
        traj.sampleInto(t, reused);
        const double t_clamped = traj.sampleInto(t, buffers.col(0), buffers.col(1), buffers.col(2), buffers.col(3));

        EXPECT_EQ(reused.t, s.t) << "t " << t;
        EXPECT_EQ(t_clamped, s.t) << "t " << t;
        EXPECT_EQ(reused.q, s.q) << "t " << t;
        EXPECT_EQ(reused.qd, s.qd) << "t " << t;
        EXPECT_EQ(reused.qdd, s.qdd) << "t " << t;
        EXPECT_EQ(reused.qddd, s.qddd) << "t " << t;
        EXPECT_EQ(Eigen::VectorXd(buffers.col(0)), s.q) << "t " << t;
        EXPECT_EQ(Eigen::VectorXd(buffers.col(1)), s.qd) << "t " << t;
        EXPECT_EQ(Eigen::VectorXd(buffers.col(2)), s.qdd) << "t " << t;
        EXPECT_EQ(Eigen::VectorXd(buffers.col(3)), s.qddd) << "t " << t;
    }
}

TEST(Sampling, BatchJerkMatchesSingleSamples)
{
    auto traj = makeDemoScurve();
    auto batch = traj.samples(0.01);
    for (Eigen::Index i = 0; i < batch.t.size(); ++i)
        EXPECT_EQ(traj.sample(batch.t(i)).qddd, batch.qddd.row(i).transpose()) << "row " << i;
}
