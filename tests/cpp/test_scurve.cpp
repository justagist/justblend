#include <gtest/gtest.h>

#include <cmath>

#include "internal/scurve_profile.hpp"

using namespace justblend::internal;

namespace
{

double totalDuration(const std::vector<ScurvePhase>& phases)
{
    double T = 0.0;
    for (const auto& p : phases)
        T += p.duration;
    return T;
}

struct Bounds
{
    double max_v = 0.0;
    double max_a = 0.0;
    double max_j = 0.0;
};

Bounds sweepBounds(double v0, const std::vector<ScurvePhase>& phases, double dt)
{
    Bounds b;
    double T = totalDuration(phases);
    for (double t = 0.0; t <= T; t += dt)
    {
        auto s = sampleScurve(t, v0, phases);
        b.max_v = std::max(b.max_v, std::abs(s.sd));
        b.max_a = std::max(b.max_a, std::abs(s.sdd));
    }
    for (const auto& p : phases)
        b.max_j = std::max(b.max_j, std::abs(p.jerk));
    return b;
}

} // namespace

TEST(ScurveProfile, SevenPhaseRestToRest)
{
    // 0 -> 0 over D=10, vmax=2, amax=3, jmax=10 -> 7 phases (full sequence).
    auto phases = scurvePhases(0.0, 0.0, 2.0, 3.0, 10.0, 10.0);
    EXPECT_EQ(phases.size(), 7u);

    double T = totalDuration(phases);
    auto end = sampleScurve(T, 0.0, phases);
    EXPECT_NEAR(end.s, 10.0, 1e-9);
    EXPECT_NEAR(end.sd, 0.0, 1e-9);
    EXPECT_NEAR(end.sdd, 0.0, 1e-9);

    auto b = sweepBounds(0.0, phases, 1e-3);
    EXPECT_LE(b.max_v, 2.0 + 1e-9);
    EXPECT_LE(b.max_a, 3.0 + 1e-9);
    EXPECT_LE(b.max_j, 10.0 + 1e-9);
}

TEST(ScurveProfile, NoCruiseWhenDistanceTooShort)
{
    // D=1.5 < ramp-up + ramp-down distance at vmax=2: bisected v_peak (~1.72)
    // stays below vmax and there is no top-cruise phase (3+3 ramp phases).
    auto phases = scurvePhases(0.0, 0.0, 2.0, 3.0, 10.0, 1.5);
    EXPECT_EQ(phases.size(), 6u);

    double T = totalDuration(phases);
    auto end = sampleScurve(T, 0.0, phases);
    EXPECT_NEAR(end.s, 1.5, 1e-9);
    EXPECT_NEAR(end.sd, 0.0, 1e-9);
    EXPECT_NEAR(end.sdd, 0.0, 1e-9);

    auto b = sweepBounds(0.0, phases, 1e-4);
    EXPECT_LT(b.max_v, 2.0 - 1e-3); // vmax never reached
    EXPECT_LE(b.max_a, 3.0 + 1e-9);
    EXPECT_LE(b.max_j, 10.0 + 1e-9);
}

TEST(ScurveProfile, NoAccelPlateauWhenJerkLimited)
{
    // Very small D: peak accel stays below amax, so each ramp is two pure
    // jerk phases (no constant-accel plateau) -> 4 phases total.
    auto phases = scurvePhases(0.0, 0.0, 2.0, 5.0, 50.0, 0.05);
    ASSERT_EQ(phases.size(), 4u);
    for (const auto& p : phases)
    {
        EXPECT_NEAR(std::abs(p.jerk), 50.0, 1e-12);
    }

    double T = totalDuration(phases);
    auto end = sampleScurve(T, 0.0, phases);
    EXPECT_NEAR(end.s, 0.05, 1e-9);
    EXPECT_NEAR(end.sd, 0.0, 1e-9);
    EXPECT_NEAR(end.sdd, 0.0, 1e-9);

    auto b = sweepBounds(0.0, phases, 1e-5);
    EXPECT_LT(b.max_a, 5.0 - 1e-3); // amax never reached
}

TEST(ScurveProfile, ReachableMonotone)
{
    double r1 = reachableScurve(0.0, 3.0, 10.0, 1.0, 5.0);
    double r2 = reachableScurve(0.0, 3.0, 10.0, 2.0, 5.0);
    EXPECT_LT(r1, r2);
    // Sufficiently large D returns v_upper.
    double r3 = reachableScurve(0.0, 3.0, 10.0, 1000.0, 5.0);
    EXPECT_DOUBLE_EQ(r3, 5.0);
}
