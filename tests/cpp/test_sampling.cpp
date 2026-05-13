#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

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
    opts.corner_handling = CornerHandling::Hybrid;
    gen.setOptions(opts);
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
    EXPECT_THROW(gen.setLimits(L), std::invalid_argument);
}
