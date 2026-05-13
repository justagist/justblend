#include "justblend/trajectory_generator.hpp"

#include <stdexcept>

#include "internal/planner.hpp"
#include "internal/segment.hpp"

namespace justblend {

TrajectoryGenerator::TrajectoryGenerator(std::size_t dim) : dim_(dim) {
    if (dim_ == 0) throw std::invalid_argument("TrajectoryGenerator: dim must be >= 1.");
}

void TrajectoryGenerator::setLimits(const Limits& limits) {
    if (static_cast<std::size_t>(limits.v_max.size()) != dim_) {
        throw std::invalid_argument("setLimits: v_max size mismatches dim.");
    }
    if (static_cast<std::size_t>(limits.a_max.size()) != dim_) {
        throw std::invalid_argument("setLimits: a_max size mismatches dim.");
    }
    if (limits.j_max.has_value() && static_cast<std::size_t>(limits.j_max->size()) != dim_) {
        throw std::invalid_argument("setLimits: j_max size mismatches dim.");
    }
    validateLimitsBase(limits);
    validateLimitsDerived(limits);
    limits_ = limits;
}

void TrajectoryGenerator::setOptions(const GenerationOptions& opts) {
    if (!(opts.blend_radius > 0.0)) {
        throw std::invalid_argument("setOptions: blend_radius must be > 0.");
    }
    if (opts.blend_radii.has_value()) {
        if (opts.blend_radii->size() == 0) {
            throw std::invalid_argument("setOptions: blend_radii cannot be empty.");
        }
        if ((opts.blend_radii->array() <= 0.0).any()) {
            throw std::invalid_argument("setOptions: every blend_radii entry must be > 0.");
        }
    }
    options_ = opts;
}

const Limits& TrajectoryGenerator::limits() const {
    if (!limits_.has_value()) throw ValidationError("Limits not set.");
    return *limits_;
}

void TrajectoryGenerator::validateLimitsBase(const Limits& limits) const {
    if ((limits.v_max.array() <= 0.0).any()) {
        throw ValidationError("v_max entries must be strictly positive.");
    }
    if ((limits.a_max.array() <= 0.0).any()) {
        throw ValidationError("a_max entries must be strictly positive.");
    }
    if (limits.j_max.has_value() && (limits.j_max->array() <= 0.0).any()) {
        throw ValidationError("j_max entries must be strictly positive.");
    }
}

Trajectory TrajectoryGenerator::generate(const Eigen::MatrixXd& waypoints) {
    if (!limits_.has_value()) throw ValidationError("Limits must be set before calling generate().");
    if (static_cast<std::size_t>(waypoints.cols()) != dim_) {
        throw std::invalid_argument("generate: waypoints column count does not match dim.");
    }
    BlendShape shape = options_.blend_shape.value_or(defaultBlendShape());
    auto data = internal::plan(waypoints, *limits_, options_, shape, useScurveProfile());
    return Trajectory(std::move(data));
}

}  // namespace justblend
