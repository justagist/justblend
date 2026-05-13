#ifndef JUSTBLEND_INTERNAL_TRAP_PROFILE_HPP
#define JUSTBLEND_INTERNAL_TRAP_PROFILE_HPP

#include <vector>

#include "internal/segment.hpp"

namespace justblend::internal
{

std::vector<TrapPhase> trapPhases(double v0, double v1, double vmax, double amax, double D);

struct TrapSample
{
    double s;
    double sd;
    double sdd;
};

TrapSample sampleTrap(double t_local, double v0, const std::vector<TrapPhase>& phases);

double reachableTrap(double v0, double amax, double D);

} // namespace justblend::internal

#endif // JUSTBLEND_INTERNAL_TRAP_PROFILE_HPP
