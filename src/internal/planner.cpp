#include "internal/planner.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "internal/blend.hpp"
#include "internal/scurve_profile.hpp"
#include "internal/trap_profile.hpp"

namespace justblend::internal {

namespace {

// numpy.allclose(a, b, atol=1e-9, rtol=1e-5) semantics.
bool allClose(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double atol = 1e-9, double rtol = 1e-5) {
    if (a.size() != b.size()) return false;
    const auto diff = (a - b).array().abs();
    const auto tol = atol + rtol * b.array().abs();
    return ((diff <= tol).all());
}

SegmentInfo extractInfo(const Segment& seg) {
    SegmentInfo info;
    info.type = seg.type;
    info.duration = seg.duration;
    if (seg.type == SegmentType::Linear) {
        info.v0 = seg.linear.v0;
        info.length = seg.linear.length;
        info.u_dir = seg.linear.u_dir;
        info.q0 = seg.linear.q0;
    } else {
        info.blend_shape = seg.blend.shape;
        info.V = seg.blend.V;
        info.r = seg.blend.r;
        info.d_in = seg.blend.d_in;
        info.d_out = seg.blend.d_out;
        info.q_start = seg.blend.q_start;
    }
    return info;
}

}  // namespace

std::shared_ptr<PlannedTrajectoryData> plan(const Eigen::MatrixXd& waypoints, const Limits& limits,
                                            const GenerationOptions& options, BlendShape effective_blend_shape,
                                            bool use_scurve) {
    const Eigen::Index N = waypoints.rows();
    const Eigen::Index D = waypoints.cols();

    if (N < 2) {
        throw ValidationError("Need at least 2 waypoints.");
    }
    if (limits.v_max.size() != D || limits.a_max.size() != D) {
        throw std::invalid_argument("Limits dimension does not match waypoint dimension.");
    }
    if (use_scurve) {
        if (!limits.j_max.has_value()) {
            throw ValidationError("S-curve profile requires j_max in limits.");
        }
        if (limits.j_max->size() != D) {
            throw std::invalid_argument("j_max dimension does not match waypoint dimension.");
        }
    }
    if (options.corner_handling != CornerHandling::StrictCorners
        && options.corner_handling != CornerHandling::UseBlending
        && options.corner_handling != CornerHandling::Hybrid) {
        throw std::invalid_argument("Unknown corner_handling value.");
    }

    // Per-corner radii size check (1 broadcasts, else must match N - 2).
    const Eigen::Index n_interior = std::max<Eigen::Index>(N - 2, 0);
    if (options.blend_radii.has_value()) {
        const Eigen::Index sz = options.blend_radii->size();
        if (sz != 1 && sz != n_interior) {
            std::ostringstream oss;
            oss << "blend_radii must have size 1 or N-2 = " << n_interior << "; got size " << sz << ".";
            throw std::invalid_argument(oss.str());
        }
    }
    auto radius_for_corner = [&](Eigen::Index k) -> double {
        if (options.blend_radii.has_value()) {
            const auto& v = *options.blend_radii;
            return v.size() == 1 ? v(0) : v(k - 1);
        }
        return options.blend_radius;
    };

    // deltas, L_full, u_dir
    Eigen::MatrixXd deltas = waypoints.bottomRows(N - 1) - waypoints.topRows(N - 1);
    Eigen::VectorXd L_full(N - 1);
    for (Eigen::Index k = 0; k < N - 1; ++k) {
        L_full(k) = deltas.row(k).norm();
        if (L_full(k) < 1e-12) {
            throw ValidationError("Duplicate consecutive waypoints (zero-length segment).");
        }
    }
    Eigen::MatrixXd u_dir(N - 1, D);
    for (Eigen::Index k = 0; k < N - 1; ++k) {
        u_dir.row(k) = deltas.row(k) / L_full(k);
    }

    // sd_max, sdd_max, sddd_max per segment
    Eigen::VectorXd sd_max(N - 1);
    Eigen::VectorXd sdd_max(N - 1);
    Eigen::VectorXd sddd_max(use_scurve ? (N - 1) : 0);
    for (Eigen::Index k = 0; k < N - 1; ++k) {
        Eigen::VectorXd abs_dir = u_dir.row(k).cwiseAbs().transpose().cwiseMax(1e-12);
        sd_max(k) = (limits.v_max.array() / abs_dir.array()).minCoeff();
        sdd_max(k) = (limits.a_max.array() / abs_dir.array()).minCoeff();
        if (use_scurve) {
            sddd_max(k) = (limits.j_max->array() / abs_dir.array()).minCoeff();
        }
    }

    // Classify corners
    std::vector<CornerType> corner_type(static_cast<std::size_t>(N), CornerType::Pass);
    corner_type.front() = CornerType::Endpoint;
    corner_type.back() = CornerType::Endpoint;

    std::vector<double> blend_r(static_cast<std::size_t>(N), 0.0);
    std::vector<double> V_blend_cap(static_cast<std::size_t>(N), 0.0);

    for (Eigen::Index k = 1; k < N - 1; ++k) {
        Eigen::VectorXd u_prev = u_dir.row(k - 1).transpose();
        Eigen::VectorXd u_curr = u_dir.row(k).transpose();
        if (allClose(u_prev, u_curr)) {
            corner_type[k] = CornerType::Pass;
            continue;
        }
        if (options.corner_handling == CornerHandling::StrictCorners) {
            corner_type[k] = CornerType::Stop;
            continue;
        }
        double max_r_geom = std::min(L_full(k - 1), L_full(k)) / 2.0;
        double r_requested = radius_for_corner(k);
        double r = std::min(r_requested, max_r_geom);
        if (r > 1e-9) {
            double V_max = blendVCap(u_prev, u_curr, r, limits.v_max, limits.a_max,
                                     use_scurve ? limits.j_max : std::optional<Eigen::VectorXd>{},
                                     effective_blend_shape);
            if (V_max > 1e-12) {
                corner_type[k] = CornerType::Blend;
                blend_r[k] = r;
                V_blend_cap[k] = V_max;
            } else {
                if (options.corner_handling == CornerHandling::UseBlending) {
                    std::ostringstream oss;
                    oss << "Blend at waypoint " << k << " infeasible (V_max <= 0).";
                    throw ValidationError(oss.str());
                }
                corner_type[k] = CornerType::Stop;
            }
        } else {
            if (options.corner_handling == CornerHandling::UseBlending) {
                std::ostringstream oss;
                oss << "Blend at waypoint " << k << ": adjacent segments shorter than 2 * blend_radius = "
                    << (2.0 * r_requested) << ".";
                throw ValidationError(oss.str());
            }
            corner_type[k] = CornerType::Stop;
        }
    }

    // Forward/backward pass with hybrid blend-collapse retry
    Eigen::VectorXd L_eff(N - 1);
    Eigen::VectorXd U(N);

    while (true) {
        L_eff = L_full;
        for (Eigen::Index k = 1; k < N - 1; ++k) {
            if (corner_type[k] == CornerType::Blend) {
                L_eff(k - 1) -= blend_r[k];
                L_eff(k) -= blend_r[k];
            }
        }
        if ((L_eff.array() < -1e-9).any()) {
            throw ValidationError("Blends overlap; reduce blend_radius.");
        }
        L_eff = L_eff.cwiseMax(0.0);

        Eigen::VectorXd U_cap = Eigen::VectorXd::Zero(N);
        for (Eigen::Index k = 1; k < N - 1; ++k) {
            if (corner_type[k] == CornerType::Pass) {
                U_cap(k) = std::min(sd_max(k - 1), sd_max(k));
            } else if (corner_type[k] == CornerType::Blend) {
                U_cap(k) = V_blend_cap[k];
            }
            // Stop/Endpoint: 0
        }

        U = U_cap;
        for (Eigen::Index k = 1; k < N; ++k) {
            double L = L_eff(k - 1);
            if (L > 1e-12) {
                double reach;
                if (use_scurve) {
                    reach = reachableScurve(U(k - 1), sdd_max(k - 1), sddd_max(k - 1), L, sd_max(k - 1));
                } else {
                    reach = reachableTrap(U(k - 1), sdd_max(k - 1), L);
                }
                U(k) = std::min(U(k), reach);
            } else {
                U(k) = std::min(U(k), U(k - 1));
            }
        }
        for (Eigen::Index k = N - 2; k >= 0; --k) {
            double L = L_eff(k);
            if (L > 1e-12) {
                double reach;
                if (use_scurve) {
                    reach = reachableScurve(U(k + 1), sdd_max(k), sddd_max(k), L, sd_max(k));
                } else {
                    reach = reachableTrap(U(k + 1), sdd_max(k), L);
                }
                U(k) = std::min(U(k), reach);
            } else {
                U(k) = std::min(U(k), U(k + 1));
            }
        }

        bool collapsed = false;
        for (Eigen::Index k = 1; k < N - 1; ++k) {
            if (corner_type[k] == CornerType::Blend && U(k) < 1e-6) {
                if (options.corner_handling == CornerHandling::UseBlending) {
                    std::ostringstream oss;
                    oss << "Blend at waypoint " << k
                        << " collapsed (V forced to 0). Use smaller blend_radius or 'hybrid' mode.";
                    throw ValidationError(oss.str());
                }
                corner_type[k] = CornerType::Stop;
                blend_r[k] = 0.0;
                V_blend_cap[k] = 0.0;
                collapsed = true;
            }
        }
        if (!collapsed) break;
    }

    // Build segment list
    auto out = std::make_shared<PlannedTrajectoryData>();
    out->dim = static_cast<std::size_t>(D);
    out->blend_shape = effective_blend_shape;
    out->corner_handling = options.corner_handling;
    out->is_scurve = use_scurve;
    out->waypoints = waypoints;
    out->junction_speeds = U;
    out->blend_radii = blend_r;
    out->corner_types = corner_type;

    for (Eigen::Index k = 0; k < N - 1; ++k) {
        Eigen::VectorXd start_pos = waypoints.row(k).transpose();
        if (corner_type[k] == CornerType::Blend) {
            start_pos = waypoints.row(k).transpose() + blend_r[k] * u_dir.row(k).transpose();
        }

        double L = L_eff(k);
        if (L > 1e-12) {
            double v0 = U(k);
            double v1 = U(k + 1);
            Segment seg;
            seg.type = SegmentType::Linear;
            seg.linear.u_dir = u_dir.row(k).transpose();
            seg.linear.q0 = start_pos;
            seg.linear.v0 = v0;
            seg.linear.length = L;
            seg.linear.is_scurve = use_scurve;
            if (use_scurve) {
                seg.linear.scurve_phases =
                    scurvePhases(v0, v1, sd_max(k), sdd_max(k), sddd_max(k), L);
                double T = 0.0;
                for (const auto& p : seg.linear.scurve_phases) T += p.duration;
                seg.linear.duration = T;
                seg.duration = T;
            } else {
                seg.linear.trap_phases = trapPhases(v0, v1, sd_max(k), sdd_max(k), L);
                double T = 0.0;
                for (const auto& p : seg.linear.trap_phases) T += p.duration;
                seg.linear.duration = T;
                seg.duration = T;
            }
            out->segments.push_back(std::move(seg));
        }

        Eigen::Index kk = k + 1;
        if (kk < N - 1 && corner_type[kk] == CornerType::Blend) {
            double r_b = blend_r[kk];
            double V = U(kk);
            double T_b = 2.0 * r_b / V;
            Segment seg;
            seg.type = SegmentType::Blend;
            seg.blend.shape = effective_blend_shape;
            seg.blend.duration = T_b;
            seg.blend.V = V;
            seg.blend.r = r_b;
            seg.blend.d_in = u_dir.row(k).transpose();
            seg.blend.d_out = u_dir.row(k + 1).transpose();
            seg.blend.q_start = waypoints.row(kk).transpose() - r_b * u_dir.row(k).transpose();
            seg.blend.q_center = waypoints.row(kk).transpose();
            seg.duration = T_b;
            out->segments.push_back(std::move(seg));
        }
    }

    out->cumulative_t.assign(out->segments.size() + 1, 0.0);
    for (std::size_t i = 0; i < out->segments.size(); ++i) {
        out->cumulative_t[i + 1] = out->cumulative_t[i] + out->segments[i].duration;
    }
    out->total_duration = out->cumulative_t.back();

    out->segment_infos.reserve(out->segments.size());
    for (const auto& seg : out->segments) {
        out->segment_infos.push_back(extractInfo(seg));
    }

    return out;
}

}  // namespace justblend::internal
