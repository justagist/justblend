#ifndef JUSTBLEND_TRAJECTORY_GENERATOR_HPP
#define JUSTBLEND_TRAJECTORY_GENERATOR_HPP

#include <Eigen/Core>
#include <cstddef>
#include <optional>

#include "justblend/trajectory.hpp"
#include "justblend/types.hpp"

namespace justblend
{

class TrajectoryGenerator
{
public:
    explicit TrajectoryGenerator(std::size_t dim);
    virtual ~TrajectoryGenerator() = default;

    TrajectoryGenerator(const TrajectoryGenerator&) = delete;
    TrajectoryGenerator& operator=(const TrajectoryGenerator&) = delete;
    TrajectoryGenerator(TrajectoryGenerator&&) = default;
    TrajectoryGenerator& operator=(TrajectoryGenerator&&) = default;

    void setLimits(const Limits& limits);
    void setOptions(const GenerationOptions& opts);

    // Generate a trajectory through the waypoint rows. v_start / v_end are
    // optional tangential boundary speeds along the first / last segment
    // direction; they must be feasible under the (scaled) limits or a
    // ValidationError is thrown.
    Trajectory generate(const Eigen::MatrixXd& waypoints, double v_start = 0.0, double v_end = 0.0);

    std::size_t dim() const noexcept { return dim_; }
    bool limitsSet() const noexcept { return limits_.has_value(); }
    const Limits& limits() const;
    const GenerationOptions& options() const noexcept { return options_; }

protected:
    virtual BlendShape defaultBlendShape() const noexcept = 0;
    virtual bool useScurveProfile() const noexcept = 0;
    virtual void validateLimitsDerived(const Limits&) const = 0;

    void validateLimitsBase(const Limits&) const;

    std::size_t dim_;
    std::optional<Limits> limits_;
    GenerationOptions options_{};
};

} // namespace justblend

#endif // JUSTBLEND_TRAJECTORY_GENERATOR_HPP
