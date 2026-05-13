#ifndef JUSTBLEND_INTERNAL_BLEND_HPP
#define JUSTBLEND_INTERNAL_BLEND_HPP

#include <Eigen/Core>
#include <optional>

#include "internal/segment.hpp"
#include "justblend/types.hpp"

namespace justblend::internal {

double blendVCap(const Eigen::VectorXd& d_in, const Eigen::VectorXd& d_out, double r,
                 const Eigen::VectorXd& vmax, const Eigen::VectorXd& amax,
                 const std::optional<Eigen::VectorXd>& jmax, BlendShape shape);

struct BlendSample {
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
};

BlendSample sampleBlend(double t_local, const BlendSegmentData& seg);

}  // namespace justblend::internal

#endif  // JUSTBLEND_INTERNAL_BLEND_HPP
