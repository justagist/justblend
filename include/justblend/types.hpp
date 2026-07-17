#ifndef JUSTBLEND_TYPES_HPP
#define JUSTBLEND_TYPES_HPP

#include <Eigen/Core>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace justblend
{

class ValidationError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

struct Limits
{
    Eigen::VectorXd v_max;
    Eigen::VectorXd a_max;
    std::optional<Eigen::VectorXd> j_max;
};

enum class BlendShape
{
    Parabolic,
    Hermite,
};

enum class CornerHandling
{
    StrictCorners,
    UseBlending,
    Hybrid,
};

enum class CornerType
{
    Endpoint,
    Pass,
    Stop,
    Blend,
};

enum class SegmentType
{
    Linear,
    Blend,
};

struct GenerationOptions
{
    // Scalar fallback used when blend_radii is unset (the common case).
    double blend_radius = 0.1;

    // Optional per-corner radii. If size == 1 it broadcasts (equivalent to
    // setting blend_radius). Otherwise must have size N - 2 (one entry per
    // interior corner, in waypoint order). Entries must be > 0.
    std::optional<Eigen::VectorXd> blend_radii;

    CornerHandling corner_handling = CornerHandling::StrictCorners;
    std::optional<BlendShape> blend_shape;

    // Uniform limit scaling in (0, 1] applied at generation time (mirrors
    // MoveIt's scaling factors). velocity_scaling_factor scales v_max,
    // acceleration_scaling_factor scales a_max; j_max is never scaled.
    double velocity_scaling_factor = 1.0;
    double acceleration_scaling_factor = 1.0;
};

struct Sample
{
    double t = 0.0;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
};

struct SegmentInfo
{
    SegmentType type = SegmentType::Linear;
    double duration = 0.0;

    // Linear-segment fields (valid when type == Linear)
    double v0 = 0.0;
    double length = 0.0;
    Eigen::VectorXd u_dir;
    Eigen::VectorXd q0;

    // Blend-segment fields (valid when type == Blend)
    BlendShape blend_shape = BlendShape::Parabolic;
    double V = 0.0;
    double r = 0.0;
    Eigen::VectorXd d_in;
    Eigen::VectorXd d_out;
    Eigen::VectorXd q_start;
};

} // namespace justblend

#endif // JUSTBLEND_TYPES_HPP
