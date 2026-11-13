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
    // Sometimes put a waypoint on the segment before it, so runs get merged.
    for (Eigen::Index k = 2; k < n; ++k)
    {
        if (std::bernoulli_distribution(0.2)(rng))
            s.waypoints.row(k) =
                s.waypoints.row(k - 1) + uniform(rng, 0.2, 1.5) * (s.waypoints.row(k - 1) - s.waypoints.row(k - 2));
    }

    s.limits.v_max = uniformVector(rng, dim, 0.5, 2.0);
    s.limits.a_max = uniformVector(rng, dim, 1.0, 5.0);
    if (scurve)
        s.limits.j_max = uniformVector(rng, dim, 5.0, 50.0);

    s.options.blend_radius = uniform(rng, 0.01, 0.5);
    constexpr CornerHandling kModes[] = {
        CornerHandling::STRICT_CORNERS, CornerHandling::BLEND_WHERE_FEASIBLE, CornerHandling::BLEND_WHERE_FASTER
    };
    s.options.corner_handling = kModes[std::uniform_int_distribution<int>(0, 2)(rng)];
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
    if (std::bernoulli_distribution(1.0 / 3.0)(rng))
        s.options.max_corner_deviation = uniform(rng, 1e-3, 0.1);
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
        if (qdd_continuous)
            EXPECT_LE(dense.qddd.col(j).cwiseAbs().maxCoeff(), (*s.limits.j_max)(j) + 1e-9) << "axis " << j;
    }

    // The analytic jerk is the derivative of the acceleration. Compare it with
    // a central difference wherever the jerk is linear across the window, which
    // excludes the jerk steps between S-curve phases.
    if (qdd_continuous)
    {
        const double h = 1e-6;
        for (int i = 1; i < 50; ++i)
        {
            const double t = T * i / 50.0;
            const auto minus = traj.sample(t - h);
            const auto mid = traj.sample(t);
            const auto plus = traj.sample(t + h);
            if ((mid.qddd - 0.5 * (plus.qddd + minus.qddd)).norm() > 1e-6)
                continue;
            const Eigen::VectorXd difference = (plus.qdd - minus.qdd) / (2.0 * h);
            EXPECT_LT((difference - mid.qddd).norm(), 1e-5) << "t " << t;
        }
    }

    if (s.options.max_corner_deviation.has_value())
        EXPECT_LE(traj.maxCornerDeviation(), *s.options.max_corner_deviation * (1.0 + 1e-12));

    // Where every corner can be blended, BLEND_WHERE_FEASIBLE blends them all.
    if (s.options.corner_handling == CornerHandling::BLEND_WHERE_FEASIBLE)
    {
        Scenario other = s;
        other.options.corner_handling = CornerHandling::BLEND_ALL;
        try
        {
            const auto all_blended = generate(other);
            EXPECT_EQ(T, all_blended.duration());
            EXPECT_EQ(traj.cornerTypes(), all_blended.cornerTypes());
        }
        catch (const ValidationError&)
        {
            // Some corner cannot be blended, so there is nothing to compare.
        }
    }

    // BLEND_WHERE_FASTER only blends where that is faster, so it is never
    // slower than stopping at every corner or than blending every corner.
    if (s.options.corner_handling == CornerHandling::BLEND_WHERE_FASTER)
    {
        Scenario other = s;
        other.options.corner_handling = CornerHandling::STRICT_CORNERS;
        EXPECT_LE(T, generate(other).duration());
        other.options.corner_handling = CornerHandling::BLEND_ALL;
        try
        {
            EXPECT_LE(T, generate(other).duration());
        }
        catch (const ValidationError&)
        {
            // Some corner cannot be blended, so there is nothing to compare.
        }
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
        else if (corner_types[k] == CornerType::PASS)
        {
            // A merged collinear waypoint: passed at its distance from the merged leg.
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
