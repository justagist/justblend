#ifndef JUSTBLEND_INTERNAL_PLANNER_HPP
#define JUSTBLEND_INTERNAL_PLANNER_HPP

#include <Eigen/Core>
#include <memory>

#include "internal/segment.hpp"
#include "justblend/types.hpp"

namespace justblend::internal
{

std::shared_ptr<PlannedTrajectoryData> plan(
    const Eigen::MatrixXd& waypoints, const Limits& limits, const GenerationOptions& options,
    BlendShape effective_blend_shape, bool use_scurve, double v_start, double v_end
);

} // namespace justblend::internal

#endif // JUSTBLEND_INTERNAL_PLANNER_HPP
