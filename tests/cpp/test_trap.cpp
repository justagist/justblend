#include <gtest/gtest.h>

#include <cmath>

#include "internal/trap_profile.hpp"

using namespace justblend::internal;

namespace {

double totalDuration(const std::vector<TrapPhase>& phases) {
    double T = 0.0;
    for (const auto& p : phases) T += p.duration;
    return T;
}

}  // namespace

TEST(TrapProfile, RestToRestTrapezoid) {
    // 0 -> 0 over 10m with vmax=2, amax=2. Expect a 3-phase trapezoid.
    auto phases = trapPhases(0.0, 0.0, 2.0, 2.0, 10.0);
    ASSERT_EQ(phases.size(), 3u);
    EXPECT_DOUBLE_EQ(phases[0].sdd, 2.0);
    EXPECT_DOUBLE_EQ(phases[1].sdd, 0.0);
    EXPECT_DOUBLE_EQ(phases[2].sdd, -2.0);

    double T = totalDuration(phases);
    auto end = sampleTrap(T, 0.0, phases);
    EXPECT_NEAR(end.s, 10.0, 1e-12);
    EXPECT_NEAR(end.sd, 0.0, 1e-12);
}

TEST(TrapProfile, AsymmetricEndpoints) {
    // v0=1, v1=0.5, vmax=3, amax=2, D=5 -> trapezoid (cruise reached)
    auto phases = trapPhases(1.0, 0.5, 3.0, 2.0, 5.0);
    double T = totalDuration(phases);

    auto sEnd = sampleTrap(T, 1.0, phases);
    EXPECT_NEAR(sEnd.s, 5.0, 1e-12);
    EXPECT_NEAR(sEnd.sd, 0.5, 1e-12);
    // bounds
    const double dt = 1e-3;
    double max_v = 0.0, max_a = 0.0;
    for (double t = 0.0; t <= T; t += dt) {
        auto s = sampleTrap(t, 1.0, phases);
        max_v = std::max(max_v, std::abs(s.sd));
        max_a = std::max(max_a, std::abs(s.sdd));
    }
    EXPECT_LE(max_v, 3.0 + 1e-9);
    EXPECT_LE(max_a, 2.0 + 1e-9);
}

TEST(TrapProfile, TriangleNoCruise) {
    // Distance too short to reach vmax -> 2 phases (no cruise).
    auto phases = trapPhases(0.0, 0.0, 5.0, 2.0, 1.0);
    EXPECT_EQ(phases.size(), 2u);
    EXPECT_DOUBLE_EQ(phases[0].sdd, 2.0);
    EXPECT_DOUBLE_EQ(phases[1].sdd, -2.0);

    double T = totalDuration(phases);
    auto end = sampleTrap(T, 0.0, phases);
    EXPECT_NEAR(end.s, 1.0, 1e-12);
    EXPECT_NEAR(end.sd, 0.0, 1e-12);
}

TEST(TrapProfile, ReachableMonotone) {
    // _reachable_trap(v0, amax, D) = sqrt(v0^2 + 2*amax*D)
    EXPECT_NEAR(reachableTrap(0.0, 2.0, 8.0), std::sqrt(32.0), 1e-12);
    EXPECT_NEAR(reachableTrap(1.0, 2.0, 0.0), 1.0, 1e-12);
    // monotone in D
    double r1 = reachableTrap(0.5, 1.0, 1.0);
    double r2 = reachableTrap(0.5, 1.0, 2.0);
    EXPECT_LT(r1, r2);
}
