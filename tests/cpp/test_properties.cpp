// Invariants checked over seeded random waypoint sets, limits and options.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "justblend/justblend.hpp"

using namespace justblend;

namespace
{

struct Scenario
{
    bool scurve = false;
    Eigen::MatrixXd waypoints;
    Limits limits;
    GenerationOptions options;
};

double uniform(std::mt19937& rng, double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }

Eigen::VectorXd uniformVector(std::mt19937& rng, Eigen::Index size, double lo, double hi)
{
    Eigen::VectorXd v(size);
    for (Eigen::Index i = 0; i < size; ++i)
        v(i) = uniform(rng, lo, hi);
    return v;
}

Scenario randomScenario(std::mt19937& rng, bool scurve)
{
    Scenario s;
    s.scurve = scurve;
    const Eigen::Index dim = std::uniform_int_distribution<Eigen::Index>(1, 6)(rng);
    const Eigen::Index n = std::uniform_int_distribution<Eigen::Index>(2, 7)(rng);
    s.waypoints.resize(n, dim);
    for (Eigen::Index k = 0; k < n; ++k)
        s.waypoints.row(k) = uniformVector(rng, dim, -1.0, 1.0).transpose();

    s.limits.v_max = uniformVector(rng, dim, 0.5, 2.0);
    s.limits.a_max = uniformVector(rng, dim, 1.0, 5.0);
    if (scurve)
        s.limits.j_max = uniformVector(rng, dim, 5.0, 50.0);

    s.options.blend_radius = uniform(rng, 0.01, 0.5);
    s.options.corner_handling =
        std::bernoulli_distribution(0.5)(rng) ? CornerHandling::HYBRID : CornerHandling::STRICT_CORNERS;
    switch (std::uniform_int_distribution<int>(0, 2)(rng))
    {
    case 0:
        s.options.blend_shape = BlendShape::PARABOLIC;
        break;
    case 1:
        s.options.blend_shape = BlendShape::HERMITE;
        break;
    default:
        break;
    }
    s.options.velocity_scaling_factor = uniform(rng, 0.3, 1.0);
    s.options.acceleration_scaling_factor = uniform(rng, 0.3, 1.0);
    return s;
}

Trajectory generate(const Scenario& s)
{
    std::unique_ptr<TrajectoryGenerator> gen;
    if (s.scurve)
        gen = std::make_unique<SCurveTrajectoryGenerator>(static_cast<std::size_t>(s.waypoints.cols()));
    else
        gen = std::make_unique<TrapezoidalTrajectoryGenerator>(static_cast<std::size_t>(s.waypoints.cols()));
    gen->setLimits(s.limits);
    gen->setOptions(s.options);
    return gen->generate(s.waypoints);
}

void checkInvariants(const Scenario& s, const Trajectory& traj)
{
    const double T = traj.duration();
    const auto& starts = traj.segmentStartTimes();
    const auto& segments = traj.segments();

    ASSERT_EQ(starts.size(), segments.size() + 1);
    EXPECT_EQ(starts.front(), 0.0);
    EXPECT_EQ(starts.back(), T);
    for (std::size_t i = 0; i < segments.size(); ++i)
    {
        EXPECT_TRUE(std::isfinite(segments[i].duration));
        EXPECT_GT(segments[i].duration, 0.0);
        EXPECT_NEAR(starts[i + 1] - starts[i], segments[i].duration, 1e-12 * std::max(1.0, T));
    }

    // Position and velocity are continuous across every segment boundary;
    // acceleration too for an S-curve with Hermite blends.
    const bool qdd_continuous = s.scurve && traj.blendShape() == BlendShape::HERMITE;
    for (std::size_t i = 1; i + 1 < starts.size(); ++i)
    {
        const auto before = traj.sample(starts[i]);
        const auto after = traj.sample(std::nextafter(starts[i], std::numeric_limits<double>::infinity()));
        EXPECT_LT((after.q - before.q).norm(), 1e-9) << "boundary " << i;
        EXPECT_LT((after.qd - before.qd).norm(), 1e-9) << "boundary " << i;
        if (qdd_continuous)
            EXPECT_LT((after.qdd - before.qdd).norm(), 1e-6) << "boundary " << i;
    }

    const auto dense = traj.samples(T / 2000.0);
    const Eigen::VectorXd v_limit = s.options.velocity_scaling_factor * s.limits.v_max;
    const Eigen::VectorXd a_limit = s.options.acceleration_scaling_factor * s.limits.a_max;
    for (Eigen::Index j = 0; j < s.waypoints.cols(); ++j)
    {
        EXPECT_LE(dense.qd.col(j).cwiseAbs().maxCoeff(), v_limit(j) + 1e-9) << "axis " << j;
        EXPECT_LE(dense.qdd.col(j).cwiseAbs().maxCoeff(), a_limit(j) + 1e-9) << "axis " << j;
    }

    const auto& waypoint_times = traj.waypointTimes();
    const auto& corner_types = traj.cornerTypes();
    const auto& deviations = traj.cornerDeviations();
    ASSERT_EQ(waypoint_times.size(), static_cast<std::size_t>(s.waypoints.rows()));
    EXPECT_EQ(waypoint_times.front(), 0.0);
    EXPECT_NEAR(waypoint_times.back(), T, 1e-12 * std::max(1.0, T));
    for (std::size_t k = 0; k < waypoint_times.size(); ++k)
    {
        if (k > 0)
            EXPECT_GT(waypoint_times[k], waypoint_times[k - 1]) << "waypoint " << k;

        const auto at_waypoint = traj.sample(waypoint_times[k]);
        const double miss = (at_waypoint.q - s.waypoints.row(static_cast<Eigen::Index>(k)).transpose()).norm();
        if (corner_types[k] == CornerType::BLEND)
        {
            EXPECT_NEAR(miss, deviations[k], 1e-9) << "waypoint " << k;
        }
        else
        {
            EXPECT_LT(miss, 1e-9) << "waypoint " << k;
            EXPECT_EQ(deviations[k], 0.0) << "waypoint " << k;
        }
        if (corner_types[k] == CornerType::STOP)
            EXPECT_LT(at_waypoint.qd.norm(), 1e-9) << "waypoint " << k;
    }
}

} // namespace

class RandomTrajectories : public ::testing::TestWithParam<bool>
{
};

TEST_P(RandomTrajectories, InvariantsHold)
{
    std::mt19937 rng(20261111);
    for (int trial = 0; trial < 300; ++trial)
    {
        SCOPED_TRACE("trial " + std::to_string(trial));
        const Scenario s = randomScenario(rng, GetParam());
        checkInvariants(s, generate(s));
    }
}

INSTANTIATE_TEST_SUITE_P(
    Profiles, RandomTrajectories, ::testing::Bool(),
    [](const ::testing::TestParamInfo<bool>& info) { return info.param ? "SCurve" : "Trapezoidal"; }
);
