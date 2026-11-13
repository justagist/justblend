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

/// @brief Immutable planned trajectory: a list of analytic segments that can
/// be sampled lazily at any time in [0, duration()]. Copies are cheap (shared,
/// read-only data) and const sampling is thread-safe.
class Trajectory
{
public:
    /// @brief Batch sampling result; one row per requested time.
    struct SamplesResult
    {
        Eigen::VectorXd t;
        Eigen::MatrixXd q;
        Eigen::MatrixXd qd;
        Eigen::MatrixXd qdd;
        Eigen::MatrixXd qddd;
    };

    Trajectory();
    Trajectory(const Trajectory&) = default;
    Trajectory(Trajectory&&) noexcept = default;
    Trajectory& operator=(const Trajectory&) = default;
    Trajectory& operator=(Trajectory&&) noexcept = default;
    ~Trajectory();

    Sample sample(double t) const;

    /// @brief Allocation-free sample into a reusable Sample. The vectors are
    /// resized only when their size differs from dim(), so after the first
    /// call this never allocates.
    /// @param t Query time; clamped to [0, duration()]. NaN throws ValidationError.
    /// @param out Receives the clamped time and the state.
    void sampleInto(double t, Sample& out) const;

    /// @brief Allocation-free sample into caller-owned buffers, for real-time
    /// loops. Never allocates unless it throws.
    /// @param t Query time; clamped to [0, duration()]. NaN throws ValidationError.
    /// @param q Position output; every buffer must have size dim() or
    /// ValidationError is thrown.
    /// @param qd Velocity output.
    /// @param qdd Acceleration output.
    /// @param qddd Jerk output.
    /// @return The clamped time.
    double sampleInto(
        double t, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qd, Eigen::Ref<Eigen::VectorXd> qdd,
        Eigen::Ref<Eigen::VectorXd> qddd
    ) const;
    SamplesResult samples(double dt) const;
    SamplesResult samples(const Eigen::Ref<const Eigen::VectorXd>& times) const;

    double duration() const noexcept;
    std::size_t dim() const noexcept;

    /// @brief Path speed per waypoint: zero at a stop, the speed at its
    /// passage time at a merged collinear waypoint, and for a blended corner
    /// the corner speed, at which the blend starts and ends.
    /// @return Vector of size N (one entry per input waypoint).
    const Eigen::VectorXd& junctionSpeeds() const;
    const std::vector<double>& blendRadii() const;
    const std::vector<CornerType>& cornerTypes() const;

    /// @brief Start time of each segment plus the total duration as the final entry.
    /// @return Vector of size numSegments() + 1; first entry is 0.
    const std::vector<double>& segmentStartTimes() const;

    /// @brief Passage time per waypoint. For blended corners this is the time of
    /// closest approach to the waypoint (the blend midpoint).
    /// @return Vector of size N (one entry per input waypoint).
    const std::vector<double>& waypointTimes() const;

    /// @brief Distance per waypoint between the waypoint and the trajectory at
    /// its passage time: the closed-form corner cut at a blend, the distance
    /// to the merged straight segment at a collinear (PASS) waypoint, and zero
    /// otherwise.
    /// @return Vector of size N (one entry per input waypoint).
    const std::vector<double>& cornerDeviations() const;

    /// @brief Largest entry of cornerDeviations().
    /// @return 0.0 when every waypoint is passed exactly.
    double maxCornerDeviation() const;

    /// @brief Copy of this trajectory uniformly slowed down to an exact total
    /// duration: same path, velocities scaled by T/target, so limits stay satisfied.
    /// @param target_duration Requested total duration; throws ValidationError when
    /// below the current duration (speeding up could violate the limits).
    Trajectory stretchedTo(double target_duration) const;
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
