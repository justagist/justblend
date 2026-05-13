#include "justblend/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "internal/blend.hpp"
#include "internal/scurve_profile.hpp"
#include "internal/segment.hpp"
#include "internal/trap_profile.hpp"

namespace justblend {

namespace {

std::size_t findSegmentIndex(const internal::PlannedTrajectoryData& d, double t) {
    if (d.segments.empty()) return 0;
    if (t <= 0.0) return 0;
    if (t >= d.total_duration) return d.segments.size() - 1;
    // Right-continuous at segment boundaries: t == cumulative_t[j] belongs to segment j-1.
    auto it = std::lower_bound(d.cumulative_t.begin() + 1, d.cumulative_t.end(), t);
    std::size_t j = static_cast<std::size_t>(std::distance(d.cumulative_t.begin(), it));
    if (j == 0) return 0;
    return j - 1;
}

void sampleAtSegment(const internal::Segment& seg, double t_local, Eigen::VectorXd& q, Eigen::VectorXd& qd,
                     Eigen::VectorXd& qdd) {
    if (seg.type == SegmentType::Linear) {
        if (seg.linear.is_scurve) {
            auto r = internal::sampleScurve(t_local, seg.linear.v0, seg.linear.scurve_phases);
            q = seg.linear.q0 + r.s * seg.linear.u_dir;
            qd = r.sd * seg.linear.u_dir;
            qdd = r.sdd * seg.linear.u_dir;
        } else {
            auto r = internal::sampleTrap(t_local, seg.linear.v0, seg.linear.trap_phases);
            q = seg.linear.q0 + r.s * seg.linear.u_dir;
            qd = r.sd * seg.linear.u_dir;
            qdd = r.sdd * seg.linear.u_dir;
        }
    } else {
        auto r = internal::sampleBlend(t_local, seg.blend);
        q = std::move(r.q);
        qd = std::move(r.qd);
        qdd = std::move(r.qdd);
    }
}

void applyEndpointSnap(const internal::PlannedTrajectoryData& d, Eigen::VectorXd& q, Eigen::VectorXd& qd,
                       Eigen::VectorXd& qdd, bool is_start, bool is_end) {
    if (is_start) {
        q = d.waypoints.row(0).transpose();
        qd.setZero();
        qdd.setZero();
    }
    if (is_end) {
        q = d.waypoints.row(d.waypoints.rows() - 1).transpose();
        qd.setZero();
        qdd.setZero();
    }
}

}  // namespace

Trajectory::Trajectory() = default;
Trajectory::~Trajectory() = default;

Trajectory::Trajectory(std::shared_ptr<const internal::PlannedTrajectoryData> data) : data_(std::move(data)) {}

bool Trajectory::empty() const noexcept { return data_ == nullptr; }

double Trajectory::duration() const noexcept { return data_ ? data_->total_duration : 0.0; }
std::size_t Trajectory::dim() const noexcept { return data_ ? data_->dim : 0; }

const Eigen::VectorXd& Trajectory::junctionSpeeds() const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
    return data_->junction_speeds;
}
const std::vector<double>& Trajectory::blendRadii() const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
    return data_->blend_radii;
}
const std::vector<CornerType>& Trajectory::cornerTypes() const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
    return data_->corner_types;
}
BlendShape Trajectory::blendShape() const noexcept { return data_ ? data_->blend_shape : BlendShape::Parabolic; }
CornerHandling Trajectory::cornerHandling() const noexcept {
    return data_ ? data_->corner_handling : CornerHandling::StrictCorners;
}
std::size_t Trajectory::numSegments() const noexcept { return data_ ? data_->segments.size() : 0; }
const std::vector<SegmentInfo>& Trajectory::segments() const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
    return data_->segment_infos;
}

Sample Trajectory::sample(double t) const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
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

Trajectory::SamplesResult Trajectory::samples(double dt) const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
    if (!(dt > 0.0)) throw std::invalid_argument("samples(dt): dt must be > 0.");

    const double T = data_->total_duration;
    const auto n_steps = static_cast<std::size_t>(std::floor(T / dt));
    const double last_step_t = static_cast<double>(n_steps) * dt;

    Eigen::VectorXd times;
    if (last_step_t < T) {
        times.resize(static_cast<Eigen::Index>(n_steps + 2));
        for (std::size_t i = 0; i <= n_steps; ++i)
            times(static_cast<Eigen::Index>(i)) = static_cast<double>(i) * dt;
        times(static_cast<Eigen::Index>(n_steps + 1)) = T;
    } else {
        times.resize(static_cast<Eigen::Index>(n_steps + 1));
        for (std::size_t i = 0; i <= n_steps; ++i)
            times(static_cast<Eigen::Index>(i)) = static_cast<double>(i) * dt;
    }
    return samples(times);
}

Trajectory::SamplesResult Trajectory::samples(const Eigen::Ref<const Eigen::VectorXd>& times) const {
    if (!data_) throw std::logic_error("Trajectory is empty.");
    const auto& d = *data_;
    const Eigen::Index M = times.size();

    SamplesResult r;
    r.t.resize(M);
    r.q.resize(M, static_cast<Eigen::Index>(d.dim));
    r.qd.resize(M, static_cast<Eigen::Index>(d.dim));
    r.qdd.resize(M, static_cast<Eigen::Index>(d.dim));

    if (M == 0) return r;
    if (d.segments.empty()) {
        for (Eigen::Index i = 0; i < M; ++i) {
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

    for (Eigen::Index i = 0; i < M; ++i) {
        double t = std::clamp(times(i), 0.0, d.total_duration);
        r.t(i) = t;

        while (seg_idx < d.segments.size() - 1 && t > t_accum + d.segments[seg_idx].duration) {
            t_accum += d.segments[seg_idx].duration;
            ++seg_idx;
        }
        while (seg_idx > 0 && t < t_accum) {
            --seg_idx;
            t_accum -= d.segments[seg_idx].duration;
        }

        const auto& seg = d.segments[seg_idx];
        double t_local = std::min(t - t_accum, seg.duration);
        sampleAtSegment(seg, t_local, q, qd, qdd);

        const bool is_start = (i == 0 && t == 0.0);
        const bool is_end = (i == M - 1 && t == d.total_duration);
        applyEndpointSnap(d, q, qd, qdd, is_start, is_end);

        r.q.row(i) = q.transpose();
        r.qd.row(i) = qd.transpose();
        r.qdd.row(i) = qdd.transpose();
    }
    return r;
}

}  // namespace justblend
