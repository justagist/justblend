#include "justblend/s_curve.hpp"

namespace justblend {

void SCurveTrajectoryGenerator::validateLimitsDerived(const Limits& limits) const {
    if (!limits.j_max.has_value()) {
        throw ValidationError("SCurveTrajectoryGenerator requires j_max in limits.");
    }
}

}  // namespace justblend
