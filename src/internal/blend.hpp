#ifndef JUSTBLEND_INTERNAL_BLEND_HPP
#define JUSTBLEND_INTERNAL_BLEND_HPP

#include <Eigen/Core>
#include <optional>

#include "internal/segment.hpp"
#include "justblend/types.hpp"

namespace justblend::internal
{

double blendVCap(
    const Eigen::VectorXd& d_in, const Eigen::VectorXd& d_out, double r, const Eigen::VectorXd& vmax,
    const Eigen::VectorXd& amax, const std::optional<Eigen::VectorXd>& jmax, BlendShape shape
);

/// @brief Evaluate a blend segment into caller buffers without allocating.
/// @param t_local Time since the start of the blend.
/// @param seg Blend segment.
/// @param q Position output, sized to the blend dimension.
/// @param qd Velocity output.
/// @param qdd Acceleration output.
/// @param qddd Jerk output.
void sampleBlend(
    double t_local, const BlendSegmentData& seg, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qd,
    Eigen::Ref<Eigen::VectorXd> qdd, Eigen::Ref<Eigen::VectorXd> qddd
);

} // namespace justblend::internal

#endif // JUSTBLEND_INTERNAL_BLEND_HPP
