#ifndef JUSTBLEND_S_CURVE_HPP
#define JUSTBLEND_S_CURVE_HPP

#include "justblend/trajectory_generator.hpp"

namespace justblend
{

class SCurveTrajectoryGenerator : public TrajectoryGenerator
{
public:
    using TrajectoryGenerator::TrajectoryGenerator;

protected:
    BlendShape defaultBlendShape() const noexcept override { return BlendShape::Hermite; }
    bool useScurveProfile() const noexcept override { return true; }
    void validateLimitsDerived(const Limits& limits) const override;
};

} // namespace justblend

#endif // JUSTBLEND_S_CURVE_HPP
