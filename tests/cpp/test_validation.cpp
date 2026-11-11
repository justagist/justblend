#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>

#include "justblend/justblend.hpp"

using namespace justblend;

namespace
{

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

Limits unitLimits()
{
    Limits L;
    L.v_max = Eigen::VectorXd::Constant(2, 1.0);
    L.a_max = Eigen::VectorXd::Constant(2, 2.0);
    L.j_max = Eigen::VectorXd::Constant(2, 10.0);
    return L;
}

Eigen::MatrixXd cornerWaypoints()
{
    Eigen::MatrixXd W(3, 2);
    W << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0;
    return W;
}

SCurveTrajectoryGenerator makeGenerator()
{
    SCurveTrajectoryGenerator gen(2);
    gen.setLimits(unitLimits());
    return gen;
}

} // namespace

TEST(Validation, NonFiniteLimitsRejected)
{
    SCurveTrajectoryGenerator gen(2);
    for (double bad : {kNaN, kInf})
    {
        Limits L = unitLimits();
        L.v_max(0) = bad;
        EXPECT_THROW(gen.setLimits(L), ValidationError);

        L = unitLimits();
        L.a_max(1) = bad;
        EXPECT_THROW(gen.setLimits(L), ValidationError);

        L = unitLimits();
        (*L.j_max)(0) = bad;
        EXPECT_THROW(gen.setLimits(L), ValidationError);
    }
}

TEST(Validation, NonFiniteBlendRadiiRejected)
{
    SCurveTrajectoryGenerator gen(2);
    for (double bad : {kNaN, kInf})
    {
        GenerationOptions opts;
        opts.blend_radius = bad;
        EXPECT_THROW(gen.setOptions(opts), ValidationError);

        opts = GenerationOptions{};
        opts.blend_radii = Eigen::VectorXd::Constant(1, bad);
        EXPECT_THROW(gen.setOptions(opts), ValidationError);
    }
}

TEST(Validation, NonFiniteWaypointsRejected)
{
    const auto gen = makeGenerator();
    for (double bad : {kNaN, kInf, -kInf})
    {
        Eigen::MatrixXd W = cornerWaypoints();
        W(1, 0) = bad;
        EXPECT_THROW(gen.generate(W), ValidationError);
    }
}

TEST(Validation, NonFiniteBoundarySpeedsRejected)
{
    const auto gen = makeGenerator();
    const Eigen::MatrixXd W = cornerWaypoints();
    for (double bad : {kNaN, kInf})
    {
        EXPECT_THROW(gen.generate(W, bad, 0.0), ValidationError);
        EXPECT_THROW(gen.generate(W, 0.0, bad), ValidationError);
    }
}

TEST(Validation, OverflowingPathLengthRejected)
{
    // Finite waypoints whose segment length overflows to infinity.
    const auto gen = makeGenerator();
    Eigen::MatrixXd W(2, 2);
    W << 0.0, 0.0, 1e200, 1e200;
    EXPECT_THROW(gen.generate(W), ValidationError);
}

TEST(Validation, NonFiniteSampleTimes)
{
    const auto traj = makeGenerator().generate(cornerWaypoints());
    EXPECT_THROW(traj.sample(kNaN), ValidationError);

    Eigen::VectorXd times(3);
    times << 0.0, kNaN, 0.5;
    EXPECT_THROW(traj.samples(times), ValidationError);

    // Infinite times clamp like any other out-of-range time.
    EXPECT_DOUBLE_EQ(traj.sample(kInf).t, traj.duration());
    EXPECT_DOUBLE_EQ(traj.sample(-kInf).t, 0.0);
}

TEST(Validation, BadSampleStepRejected)
{
    const auto traj = makeGenerator().generate(cornerWaypoints());
    EXPECT_THROW(traj.samples(kNaN), ValidationError);
    EXPECT_THROW(traj.samples(kInf), ValidationError);
    // A step so small that the sample count cannot be represented.
    EXPECT_THROW(traj.samples(1e-300), ValidationError);
}

TEST(Validation, NonFiniteStretchDurationRejected)
{
    const auto traj = makeGenerator().generate(cornerWaypoints());
    EXPECT_THROW(traj.stretchedTo(kNaN), ValidationError);
    EXPECT_THROW(traj.stretchedTo(kInf), ValidationError);
    // Finite, but the time scale cubed overflows.
    EXPECT_THROW(traj.stretchedTo(1e200), ValidationError);
}

TEST(Validation, StretchTargetWithinRoundingBelowDurationDoesNotSpeedUp)
{
    const auto traj = makeGenerator().generate(cornerWaypoints());
    const double T = traj.duration();
    const auto same = traj.stretchedTo(T - 5e-13);
    EXPECT_EQ(same.duration(), T);
    EXPECT_EQ(same.junctionSpeeds(), traj.junctionSpeeds());
}

TEST(Validation, SampleCountAtIndexLimitRejected)
{
    TrapezoidalTrajectoryGenerator gen(1);
    Limits L;
    L.v_max = Eigen::VectorXd::Constant(1, 1.0);
    L.a_max = Eigen::VectorXd::Constant(1, 2.0);
    gen.setLimits(L);
    Eigen::MatrixXd W(2, 1);
    W << 0.0, 1.0;
    const auto traj = gen.generate(W);
    // duration / dt is exactly 2^63, one past the largest index.
    EXPECT_THROW(traj.samples(std::ldexp(traj.duration(), -63)), ValidationError);
}

TEST(Validation, SegmentTooShortForItsSpeedRejected)
{
    // The middle leg is crossed at full speed in under the shortest phase
    // duration kept, so it would have no duration at all.
    TrapezoidalTrajectoryGenerator gen(1);
    Limits L;
    L.v_max = Eigen::VectorXd::Constant(1, 100.0);
    L.a_max = Eigen::VectorXd::Constant(1, 1e4);
    gen.setLimits(L);
    Eigen::MatrixXd W(4, 1);
    W << 0.0, 1.0, 1.0 + 1e-11, 2.0;
    EXPECT_THROW(gen.generate(W), ValidationError);
}
