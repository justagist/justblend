#ifndef JUSTBLEND_TYPES_HPP
#define JUSTBLEND_TYPES_HPP

#include <Eigen/Core>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace justblend {

class ValidationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct Limits {
    Eigen::VectorXd v_max;
    Eigen::VectorXd a_max;
    std::optional<Eigen::VectorXd> j_max;
};

enum class BlendShape {
    Parabolic,
    Hermite,
};

enum class CornerHandling {
    StrictCorners,
    UseBlending,
    Hybrid,
};

enum class CornerType {
    Endpoint,
    Pass,
    Stop,
    Blend,
};

enum class SegmentType {
    Linear,
    Blend,
};

struct GenerationOptions {
    double blend_radius = 0.1;
    CornerHandling corner_handling = CornerHandling::StrictCorners;
    std::optional<BlendShape> blend_shape;
};

struct Sample {
    double t = 0.0;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
};

struct SegmentInfo {
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

}  // namespace justblend

#endif  // JUSTBLEND_TYPES_HPP
