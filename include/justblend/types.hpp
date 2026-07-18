#ifndef JUSTBLEND_TYPES_HPP
#define JUSTBLEND_TYPES_HPP

#include <Eigen/Core>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace justblend
{

/// @brief Thrown for any invalid user input: bad limits, bad options,
/// malformed waypoints or an infeasible generation request.
class ValidationError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/// @brief Per-axis kinematic limits; every vector is sized to the trajectory
/// dimension. j_max is optional and only required by the S-curve profile.
struct Limits
{
    Eigen::VectorXd v_max;
    Eigen::VectorXd a_max;
    std::optional<Eigen::VectorXd> j_max;
};

/// @brief Geometric shape of a corner blend.
enum class BlendShape
{
    PARABOLIC, ///< Constant acceleration through the corner (steps at its boundaries).
    HERMITE,   ///< Cubic blend with zero boundary acceleration; jerk-limited when j_max is set.
};

/// @brief How interior waypoints with a direction change are handled.
enum class CornerHandling
{
    STRICT_CORNERS, ///< Full stop at every direction change; exact pass-through.
    USE_BLENDING,   ///< Blend every corner; throw ValidationError when infeasible.
    HYBRID,         ///< Blend where feasible, fall back to a stop otherwise.
};

/// @brief Classification of each waypoint in the planned trajectory.
enum class CornerType
{
    ENDPOINT, ///< First or last waypoint.
    PASS,     ///< Collinear waypoint passed at speed.
    STOP,     ///< Full stop at the waypoint.
    BLEND,    ///< Corner rounded by a blend segment.
};

/// @brief Kind of an analytic trajectory segment.
enum class SegmentType
{
    LINEAR,
    BLEND,
};

/// @brief Corner-handling and scaling options applied at generation time.
struct GenerationOptions
{
    // Scalar fallback used when blend_radii is unset (the common case).
    double blend_radius = 0.1;

    // Optional per-corner radii. If size == 1 it broadcasts (equivalent to
    // setting blend_radius). Otherwise must have size N - 2 (one entry per
    // interior corner, in waypoint order). Entries must be > 0.
    std::optional<Eigen::VectorXd> blend_radii;

    CornerHandling corner_handling = CornerHandling::STRICT_CORNERS;
    std::optional<BlendShape> blend_shape;

    // Uniform limit scaling in (0, 1] applied at generation time (mirrors
    // MoveIt's scaling factors). velocity_scaling_factor scales v_max,
    // acceleration_scaling_factor scales a_max; j_max is never scaled.
    double velocity_scaling_factor = 1.0;
    double acceleration_scaling_factor = 1.0;
};

/// @brief Trajectory state at one time instant: position, velocity, acceleration.
struct Sample
{
    double t = 0.0;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
};

/// @brief Public description of one analytic segment; which fields are valid
/// depends on `type`.
struct SegmentInfo
{
    SegmentType type = SegmentType::LINEAR;
    double duration = 0.0;

    // Linear-segment fields (valid when type == LINEAR)
    double v0 = 0.0;
    double length = 0.0;
    Eigen::VectorXd u_dir;
    Eigen::VectorXd q0;

    // Blend-segment fields (valid when type == BLEND)
    BlendShape blend_shape = BlendShape::PARABOLIC;
    double V = 0.0;
    double r = 0.0;
    Eigen::VectorXd d_in;
    Eigen::VectorXd d_out;
    Eigen::VectorXd q_start;
};

} // namespace justblend

#endif // JUSTBLEND_TYPES_HPP
