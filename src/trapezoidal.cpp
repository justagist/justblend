#include "justblend/trapezoidal.hpp"

namespace justblend {

void TrapezoidalTrajectoryGenerator::validateLimitsDerived(const Limits& /*limits*/) const {
    // No additional constraints beyond the base v/a positivity check.
}

}  // namespace justblend
