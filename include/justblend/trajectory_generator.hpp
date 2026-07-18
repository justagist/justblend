#ifndef JUSTBLEND_TRAJECTORY_GENERATOR_HPP
#define JUSTBLEND_TRAJECTORY_GENERATOR_HPP

#include <Eigen/Core>
#include <cstddef>
#include <optional>

#include "justblend/trajectory.hpp"
#include "justblend/types.hpp"

namespace justblend
{

/// @brief Base class for the trajectory generators: holds the dimension,
/// limits and options, and turns waypoint matrices into Trajectory objects.
/// Use SCurveTrajectoryGenerator or TrapezoidalTrajectoryGenerator.
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

    /// @brief Generate a trajectory through the waypoint rows.
    /// @param waypoints N x dim matrix, one waypoint per row; N >= 2.
    /// @param v_start Tangential start speed along the first segment direction.
    /// @param v_end Tangential end speed along the last segment direction. Both
    /// speeds must be feasible under the (scaled) limits or ValidationError is thrown.
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
