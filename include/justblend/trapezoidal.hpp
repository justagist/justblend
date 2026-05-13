#ifndef JUSTBLEND_TRAPEZOIDAL_HPP
#define JUSTBLEND_TRAPEZOIDAL_HPP

#include "justblend/trajectory_generator.hpp"

namespace justblend
{

class TrapezoidalTrajectoryGenerator : public TrajectoryGenerator
{
public:
    using TrajectoryGenerator::TrajectoryGenerator;

protected:
    BlendShape defaultBlendShape() const noexcept override { return BlendShape::Parabolic; }
    bool useScurveProfile() const noexcept override { return false; }
    void validateLimitsDerived(const Limits& limits) const override;
};

} // namespace justblend

#endif // JUSTBLEND_TRAPEZOIDAL_HPP
