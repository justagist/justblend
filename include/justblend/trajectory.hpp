#ifndef JUSTBLEND_TRAJECTORY_HPP
#define JUSTBLEND_TRAJECTORY_HPP

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <vector>

#include "justblend/types.hpp"

namespace justblend
{

namespace internal
{
struct PlannedTrajectoryData;
}

class Trajectory
{
public:
    struct SamplesResult
    {
        Eigen::VectorXd t;
        Eigen::MatrixXd q;
        Eigen::MatrixXd qd;
        Eigen::MatrixXd qdd;
    };

    Trajectory();
    Trajectory(const Trajectory&) = default;
    Trajectory(Trajectory&&) noexcept = default;
    Trajectory& operator=(const Trajectory&) = default;
    Trajectory& operator=(Trajectory&&) noexcept = default;
    ~Trajectory();

    Sample sample(double t) const;
    SamplesResult samples(double dt) const;
    SamplesResult samples(const Eigen::Ref<const Eigen::VectorXd>& times) const;

    double duration() const noexcept;
    std::size_t dim() const noexcept;

    const Eigen::VectorXd& junctionSpeeds() const;
    const std::vector<double>& blendRadii() const;
    const std::vector<CornerType>& cornerTypes() const;

    // Start time of each segment plus the total duration as the final entry
    // (size numSegments() + 1).
    const std::vector<double>& segmentStartTimes() const;

    // Per-waypoint passage time (size N). For blended corners this is the
    // time of closest approach to the waypoint (the blend midpoint).
    const std::vector<double>& waypointTimes() const;

    // Per-waypoint corner-cut distance (size N). Zero everywhere except at
    // blended corners, where it is the closed-form distance between the
    // waypoint and the blend path at its midpoint.
    const std::vector<double>& cornerDeviations() const;

    // Largest entry of cornerDeviations() (0.0 when there are no blends).
    double maxCornerDeviation() const;
    BlendShape blendShape() const noexcept;
    CornerHandling cornerHandling() const noexcept;

    std::size_t numSegments() const noexcept;
    const std::vector<SegmentInfo>& segments() const;

    bool empty() const noexcept;

private:
    friend class TrajectoryGenerator;
    explicit Trajectory(std::shared_ptr<const internal::PlannedTrajectoryData> data);
    std::shared_ptr<const internal::PlannedTrajectoryData> data_;
};

} // namespace justblend

#endif // JUSTBLEND_TRAJECTORY_HPP
