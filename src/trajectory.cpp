#include "justblend/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "internal/blend.hpp"
#include "internal/scurve_profile.hpp"
#include "internal/segment.hpp"
#include "internal/trap_profile.hpp"

namespace justblend
{

namespace
{

std::size_t findSegmentIndex(const internal::PlannedTrajectoryData& d, double t)
{
    if (d.segments.empty())
        return 0;
    if (t <= 0.0)
        return 0;
    if (t >= d.total_duration)
        return d.segments.size() - 1;
    // Right-continuous at segment boundaries: t == cumulative_t[j] belongs to segment j-1.
    auto it = std::lower_bound(d.cumulative_t.begin() + 1, d.cumulative_t.end(), t);
    std::size_t j = static_cast<std::size_t>(std::distance(d.cumulative_t.begin(), it));
    if (j == 0)
        return 0;
    return j - 1;
}

void sampleAtSegment(
    const internal::Segment& seg, double t_local, Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::VectorXd& qdd
)
{
    if (seg.type == SegmentType::LINEAR)
    {
        if (seg.linear.is_scurve)
        {
            auto r = internal::sampleScurve(t_local, seg.linear.v0, seg.linear.scurve_phases);
            q = seg.linear.q0 + r.s * seg.linear.u_dir;
            qd = r.sd * seg.linear.u_dir;
            qdd = r.sdd * seg.linear.u_dir;
        }
        else
        {
            auto r = internal::sampleTrap(t_local, seg.linear.v0, seg.linear.trap_phases);
            q = seg.linear.q0 + r.s * seg.linear.u_dir;
            qd = r.sd * seg.linear.u_dir;
            qdd = r.sdd * seg.linear.u_dir;
        }
    }
    else
    {
        auto r = internal::sampleBlend(t_local, seg.blend);
        q = std::move(r.q);
        qd = std::move(r.qd);
        qdd = std::move(r.qdd);
    }
}

void applyEndpointSnap(
    const internal::PlannedTrajectoryData& d, Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::VectorXd& qdd,
    bool is_start, bool is_end
)
{
    // Boundary velocities follow the (possibly nonzero) junction speeds along
    // the first / last segment direction; first and last segments are always
    // linear (endpoints are never blended).
    if (is_start)
    {
        q = d.waypoints.row(0).transpose();
        qd.setZero();
        if (!d.segments.empty() && d.junction_speeds.size() > 0 && d.segments.front().type == SegmentType::LINEAR)
        {
            qd = d.junction_speeds(0) * d.segments.front().linear.u_dir;
        }
        qdd.setZero();
    }
    if (is_end)
    {
        q = d.waypoints.row(d.waypoints.rows() - 1).transpose();
        qd.setZero();
        if (!d.segments.empty() && d.junction_speeds.size() > 0 && d.segments.back().type == SegmentType::LINEAR)
        {
            qd = d.junction_speeds(d.junction_speeds.size() - 1) * d.segments.back().linear.u_dir;
        }
        qdd.setZero();
    }
}

} // namespace

Trajectory::Trajectory() = default;
Trajectory::~Trajectory() = default;

Trajectory::Trajectory(std::shared_ptr<const internal::PlannedTrajectoryData> data) : data_(std::move(data)) {}

bool Trajectory::empty() const noexcept { return data_ == nullptr; }

double Trajectory::duration() const noexcept { return data_ ? data_->total_duration : 0.0; }
std::size_t Trajectory::dim() const noexcept { return data_ ? data_->dim : 0; }

const Eigen::VectorXd& Trajectory::junctionSpeeds() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->junction_speeds;
}
const std::vector<double>& Trajectory::blendRadii() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->blend_radii;
}
const std::vector<CornerType>& Trajectory::cornerTypes() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->corner_types;
}
const std::vector<double>& Trajectory::segmentStartTimes() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->cumulative_t;
}
const std::vector<double>& Trajectory::waypointTimes() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->waypoint_times;
}
const std::vector<double>& Trajectory::cornerDeviations() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->corner_deviations;
}
double Trajectory::maxCornerDeviation() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    const auto& dev = data_->corner_deviations;
    return dev.empty() ? 0.0 : *std::max_element(dev.begin(), dev.end());
}

Trajectory Trajectory::stretchedTo(double target_duration) const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    const double T = data_->total_duration;
    if (!(target_duration > 0.0) || target_duration + 1e-12 < T)
    {
        throw ValidationError("stretchedTo: target duration must be >= the current duration.");
    }
    const double s = target_duration / T;

    // Uniform time dilation t -> s*t: positions unchanged, velocities scale
    // by 1/s, accelerations by 1/s^2, jerks by 1/s^3. Limits stay satisfied.
    auto d = std::make_shared<internal::PlannedTrajectoryData>(*data_);
    for (auto& seg : d->segments)
    {
        seg.duration *= s;
        if (seg.type == SegmentType::LINEAR)
        {
            auto& lin = seg.linear;
            lin.duration *= s;
            lin.v0 /= s;
            for (auto& p : lin.trap_phases)
            {
                p.duration *= s;
                p.sdd /= s * s;
            }
            for (auto& p : lin.scurve_phases)
            {
                p.duration *= s;
                p.jerk /= s * s * s;
            }
        }
        else
        {
            seg.blend.duration *= s;
            seg.blend.V /= s;
        }
    }
    for (auto& info : d->segment_infos)
    {
        info.duration *= s;
        info.v0 /= s;
        info.V /= s;
    }
    for (auto& t : d->cumulative_t)
        t *= s;
    for (auto& t : d->waypoint_times)
        t *= s;
    d->junction_speeds /= s;
    d->total_duration = target_duration;
    return Trajectory(std::move(d));
}
BlendShape Trajectory::blendShape() const noexcept { return data_ ? data_->blend_shape : BlendShape::PARABOLIC; }
CornerHandling Trajectory::cornerHandling() const noexcept
{
    return data_ ? data_->corner_handling : CornerHandling::STRICT_CORNERS;
}
std::size_t Trajectory::numSegments() const noexcept { return data_ ? data_->segments.size() : 0; }
const std::vector<SegmentInfo>& Trajectory::segments() const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    return data_->segment_infos;
}

Sample Trajectory::sample(double t) const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    const auto& d = *data_;
    double t_clamped = std::clamp(t, 0.0, d.total_duration);
    std::size_t idx = findSegmentIndex(d, t_clamped);
    double t_accum = d.cumulative_t[idx];
    const auto& seg = d.segments[idx];
    double t_local = std::min(t_clamped - t_accum, seg.duration);

    Sample s;
    s.t = t_clamped;
    s.q.resize(d.dim);
    s.qd.resize(d.dim);
    s.qdd.resize(d.dim);
    sampleAtSegment(seg, t_local, s.q, s.qd, s.qdd);

    const bool is_start = (t_clamped == 0.0);
    const bool is_end = (t_clamped == d.total_duration);
    applyEndpointSnap(d, s.q, s.qd, s.qdd, is_start, is_end);
    return s;
}

Trajectory::SamplesResult Trajectory::samples(double dt) const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    if (!(dt > 0.0))
        throw ValidationError("samples(dt): dt must be > 0.");

    const double T = data_->total_duration;
    const auto n_steps = static_cast<std::size_t>(std::floor(T / dt));
    const double last_step_t = static_cast<double>(n_steps) * dt;

    Eigen::VectorXd times;
    // Append the final time only when it is meaningfully past the last step,
    // so a duration within rounding of a dt multiple gets no duplicate sample.
    if (T - last_step_t > 1e-9 * dt)
    {
        times.resize(static_cast<Eigen::Index>(n_steps + 2));
        for (std::size_t i = 0; i <= n_steps; ++i)
            times(static_cast<Eigen::Index>(i)) = static_cast<double>(i) * dt;
        times(static_cast<Eigen::Index>(n_steps + 1)) = T;
    }
    else
    {
        times.resize(static_cast<Eigen::Index>(n_steps + 1));
        for (std::size_t i = 0; i <= n_steps; ++i)
            times(static_cast<Eigen::Index>(i)) = static_cast<double>(i) * dt;
    }
    return samples(times);
}

Trajectory::SamplesResult Trajectory::samples(const Eigen::Ref<const Eigen::VectorXd>& times) const
{
    if (!data_)
        throw std::logic_error("Trajectory is empty.");
    const auto& d = *data_;
    const Eigen::Index M = times.size();

    SamplesResult r;
    r.t.resize(M);
    r.q.resize(M, static_cast<Eigen::Index>(d.dim));
    r.qd.resize(M, static_cast<Eigen::Index>(d.dim));
    r.qdd.resize(M, static_cast<Eigen::Index>(d.dim));

    if (M == 0)
        return r;
    if (d.segments.empty())
    {
        for (Eigen::Index i = 0; i < M; ++i)
        {
            r.t(i) = std::clamp(times(i), 0.0, d.total_duration);
            r.q.row(i).setZero();
            r.qd.row(i).setZero();
            r.qdd.row(i).setZero();
        }
        return r;
    }

    std::size_t seg_idx = 0;
    double t_accum = d.cumulative_t[0];

    Eigen::VectorXd q(d.dim);
    Eigen::VectorXd qd(d.dim);
    Eigen::VectorXd qdd(d.dim);

    for (Eigen::Index i = 0; i < M; ++i)
    {
        double t = std::clamp(times(i), 0.0, d.total_duration);
        r.t(i) = t;

        while (seg_idx < d.segments.size() - 1 && t > t_accum + d.segments[seg_idx].duration)
        {
            t_accum += d.segments[seg_idx].duration;
            ++seg_idx;
        }
        while (seg_idx > 0 && t < t_accum)
        {
            --seg_idx;
            t_accum -= d.segments[seg_idx].duration;
        }

        const auto& seg = d.segments[seg_idx];
        double t_local = std::min(t - t_accum, seg.duration);
        sampleAtSegment(seg, t_local, q, qd, qdd);

        const bool is_start = (t == 0.0);
        const bool is_end = (t == d.total_duration);
        applyEndpointSnap(d, q, qd, qdd, is_start, is_end);

        r.q.row(i) = q.transpose();
        r.qd.row(i) = qd.transpose();
        r.qdd.row(i) = qdd.transpose();
    }
    return r;
}

} // namespace justblend
